#include "StateMachine.h"
#include <ros/ros.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

StateMachine::StateMachine(std::shared_ptr<RobotState> robot_state, std::shared_ptr<PatrolLogic> patrol_logic,
                           std::shared_ptr<AttackLogic> attack_logic, std::shared_ptr<GoalLogic> goal_logic,
                           std::shared_ptr<TrackLogic> track_logic, BufferQueueHandle command_queue)
    : robot_state_(robot_state), patrol_logic_(patrol_logic), attack_logic_(attack_logic),
      goal_logic_(goal_logic), track_logic_(track_logic), command_queue_(command_queue),
      previous_state_(State::IDLE), resume_state_(State::IDLE)
{
    robot_state_->setCommandQueue(command_queue);
}

void StateMachine::processEvent(EventType event, const std::string &data)
{
    State current = robot_state_->getCurrentState();
    State next = current;

    try
    {
        if (event == EventType::START_PATROL && !data.empty())
        {
            json j_data = json::parse(data);
            if (!j_data.is_array())
            {
                throw std::runtime_error("Expected JSON array for patrol points");
            }
            std::vector<PatrolPoint> points;
            for (const auto &p : j_data)
            {
                PatrolPoint point;
                point.pose.position.x = p["position"]["x"].get<double>();
                point.pose.position.y = p["position"]["y"].get<double>();
                point.pose.position.z = p["position"]["z"].get<double>();
                point.pose.orientation.yaw = p["orientation"]["yaw"].get<double>();
                point.tested = false;
                points.push_back(point);
            }
            ROS_INFO("Parsed %lu patrol points from JSON for START_PATROL", points.size());
            patrol_logic_->setPatrolPoints(points);
            next = State::PATROL;
        }
        else if (event == EventType::SET_PATROL_POINTS && !data.empty())
        {
            json j_data = json::parse(data);
            if (!j_data.contains("patrol_points") || !j_data["patrol_points"].is_array())
            {
                throw std::runtime_error("Invalid patrol_points format");
            }
            std::vector<PatrolPoint> points;
            for (const auto &p : j_data["patrol_points"])
            {
                PatrolPoint point;
                point.pose.position.x = p["position"]["x"].get<double>();
                point.pose.position.y = p["position"]["y"].get<double>();
                point.pose.position.z = p["position"]["z"].get<double>();
                point.pose.orientation.yaw = p["orientation"]["yaw"].get<double>();
                point.tested = false;
                points.push_back(point);
            }
            ROS_INFO("Parsed %lu patrol points from JSON for SET_PATROL_POINTS", points.size());
            patrol_logic_->setPatrolPoints(points);
        }
        else if (event == EventType::SET_GOAL && !data.empty())
        {
            json j_data = json::parse(data);
            Pose goal;
            goal.position.x = j_data["position"]["x"].get<double>();
            goal.position.y = j_data["position"]["y"].get<double>();
            goal.position.z = j_data["position"]["z"].get<double>();
            goal.orientation.yaw = j_data["orientation"]["yaw"].get<double>();
            goal_logic_->setGoal(goal);
            next = State::GOTO_GOAL;
        }
        else if (event == EventType::CANCEL_GOAL)
        {
            patrol_logic_->stop();
            goal_logic_->stop();
            track_logic_->stop();
            next = State::IDLE;
        }
        else if (event == EventType::FIRE)
        {
            attack_logic_->fireOnce();
        }
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error parsing event data: %s", e.what());
        return;
    }

    switch (current)
    {
    case State::IDLE:
        if (event == EventType::MOVE || event == EventType::GIMBAL || event == EventType::FIRE)
        {
            next = State::MANUAL_CONTROL;
        }
        break;
    case State::PATROL:
        if (event == EventType::TARGET_DETECTED)
        {
            resume_state_ = State::PATROL;
            next = State::TRACK;
        }
        else if (event == EventType::MOVE || event == EventType::GIMBAL || event == EventType::FIRE)
        {
            resume_state_ = State::PATROL;
            next = State::MANUAL_CONTROL;
        }
        else if (event == EventType::STOP_PATROL || event == EventType::CANCEL_GOAL)
        {
            next = State::IDLE;
        }
        break;
    case State::GOTO_GOAL:
        if (event == EventType::TARGET_DETECTED)
        {
            resume_state_ = State::GOTO_GOAL;
            next = State::TRACK;
        }
        else if (event == EventType::MOVE || event == EventType::GIMBAL || event == EventType::FIRE)
        {
            resume_state_ = State::GOTO_GOAL;
            next = State::MANUAL_CONTROL;
        }
        else if (event == EventType::STOP_PATROL || event == EventType::CANCEL_GOAL)
        {
            next = State::IDLE;
        }
        break;
    case State::ATTACK:
        if (event == EventType::ATTACK_FINISHED)
        {
            next = resume_state_;
            if (next == State::PATROL)
            {
                patrol_logic_->start();
            }
        }
        else if (event == EventType::STOP_PATROL || event == EventType::CANCEL_GOAL)
        {
            next = State::IDLE;
        }
        break;
    case State::TRACK:
    {
        if (event == EventType::TARGET_LOST || event == EventType::TRACK_COMPLETED)
        {
            next = (resume_state_ == State::PATROL) ? State::PATROL : State::IDLE; // 修改：GOTO_GOAL返回IDLE
            if (next == State::PATROL)
            {
                patrol_logic_->start();
            }
        }
        else if (event == EventType::FIRE && robot_state_->isTargetDetected())
        {
            resume_state_ = (resume_state_ == State::PATROL) ? State::PATROL : State::IDLE; // 修改
            next = State::ATTACK;
        }
        else if (event == EventType::STOP_PATROL || event == EventType::CANCEL_GOAL)
        {
            next = State::IDLE;
        }
        break;
    }
    case State::MANUAL_CONTROL:
        if (event == EventType::EXIT_MANUAL)
            next = resume_state_;
        else if (event == EventType::STOP_PATROL || event == EventType::CANCEL_GOAL)
            next = State::IDLE;
        break;
    case State::EMERGENCY_STOP:
        if (event == EventType::STOP_PATROL)
            next = State::IDLE;
        break;
    default:
        break;
    }

    if (event == EventType::LOW_BATTERY)
        next = State::EMERGENCY_STOP;

    if (next != current)
    {
        exitState(current);
        enterState(next);
        previous_state_ = current;
        robot_state_->setCurrentState(next);
        ROS_INFO("State switched from %d to %d", static_cast<int>(current), static_cast<int>(next));
    }
}

void StateMachine::enterState(State state)
{
    switch (state)
    {
    case State::PATROL:
        patrol_logic_->start();
        break;
    case State::GOTO_GOAL:
        goal_logic_->start();
        break;
    case State::ATTACK:
        attack_logic_->start();
        break;
    case State::TRACK:
        track_logic_->start();
        break;
    case State::MANUAL_CONTROL:
        patrol_logic_->stop();
        attack_logic_->stop();
        goal_logic_->stop();
        track_logic_->stop();
        break;
    case State::EMERGENCY_STOP:
        patrol_logic_->stop();
        attack_logic_->stop();
        goal_logic_->stop();
        track_logic_->stop();
        break;
    default:
        break;
    }
}

void StateMachine::exitState(State state)
{
    switch (state)
    {
    case State::PATROL:
        patrol_logic_->stop();
        break;
    case State::GOTO_GOAL:
        goal_logic_->stop();
        break;
    case State::ATTACK:
        attack_logic_->stop();
        break;
    case State::TRACK:
        track_logic_->stop();
        break;
    default:
        break;
    }
}

void StateMachine::tick()
{
    processCommandQueue();
    if (robot_state_->getBatteryLevel() < 10.0f)
    {
        processEvent(EventType::LOW_BATTERY);
    }
    track_logic_->update();
}

void StateMachine::processCommandQueue()
{
    DataQueMsg *msg = GetReadyBuffer(command_queue_, 0);
    if (msg)
    {
        std::string data(msg->buff, msg->szData);
        try
        {
            json j = json::parse(data);
            EventType event = static_cast<EventType>(j["type"].get<int>());
            std::string event_data = j["data"].is_string() ? j["data"].get<std::string>() : j["data"].dump();
            processEvent(event, event_data);
        }
        catch (const std::exception &e)
        {
            ROS_ERROR("Error parsing command queue data: %s", e.what());
        }
        PutFreeBuffer(command_queue_, msg);
    }
}