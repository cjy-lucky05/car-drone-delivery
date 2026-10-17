#include "PatrolLogic.h"
#include <ros/ros.h>
#include <chrono>
#include <std_msgs/Bool.h>

PatrolLogic::PatrolLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<YamlUtils> yaml_utils,
                         std::shared_ptr<RobotState> robot_state)
    : ros_if_(ros_if), yaml_utils_(yaml_utils), robot_state_(robot_state), 
      current_index_(0), patrol_active_(false), pause_start_time_(0.0), is_pausing_(false), 
      is_scanning_(false), goal_tolerance_(0.1), scanning_speed_(0.1), forward_direction_(true) {
    ros::NodeHandle nh;
    nh.param("goal_tolerance", goal_tolerance_, 0.1);
    nh.param("scanning_speed", scanning_speed_, 0.1); // 新增：加载巡查速度
    if (goal_tolerance_ < 0.0) goal_tolerance_ = 0.0;
    ROS_INFO("PatrolLogic initialized with goal_tolerance=%f, scanning_speed=%f", goal_tolerance_, scanning_speed_);
}

void PatrolLogic::start() {
    if (!patrol_points_.empty()) {
        patrol_active_ = true;
        current_index_ = robot_state_->getPatrolIndex();
        is_pausing_ = false;
        is_scanning_ = false;
        forward_direction_ = true;
        ros_if_->setPatrolPoints(convertToPoses(patrol_points_));
        ros_if_->startPatrol(current_index_);
        ROS_INFO("Patrol started with %lu points, initial index=%lu", patrol_points_.size(), current_index_ + 1);
        ROS_INFO("Moving to point %lu: x=%f, y=%f", current_index_ + 1,
                 patrol_points_[current_index_].pose.position.x,
                 patrol_points_[current_index_].pose.position.y);
    } else {
        ROS_WARN("No patrol points set, cannot start patrol");
        patrol_active_ = false;
        robot_state_->setCurrentState(State::IDLE);
    }
}

void PatrolLogic::stop() {
    patrol_active_ = false;
    is_pausing_ = false;
    is_scanning_ = false;
    ros_if_->cancelGoal();
    ros_if_->publishCmdVel(0.0, 0.0);
    robot_state_->setPatrolIndex(current_index_);
    ROS_INFO("Patrol stopped at index %lu", current_index_);
}

void PatrolLogic::update() {
    if (!patrol_active_ || patrol_points_.empty()) return;

    auto map_msg = ros_if_->getMapMsg();
    if (map_msg) {
        Pose current_pose = ros_if_->getRobotInMapOrigin();
        ROS_DEBUG("Robot pose in map: x=%f, y=%f", current_pose.position.x, current_pose.position.y);
    }

    if (is_pausing_) {
        double current_time = ros::Time::now().toSec();
        if (current_time - pause_start_time_ < 5.0) return;
        is_pausing_ = false;
        is_scanning_ = true;
        pause_start_time_ = current_time;
        ROS_INFO("Pause finished at point %lu, starting scanning", current_index_ + 1);
    }

    if (is_scanning_) {
        double current_time = ros::Time::now().toSec();
        if (current_time - pause_start_time_ < 5.0) {
            ros_if_->publishCmdVel(scanning_speed_, 0.0); // 低速巡查
            return;
        }
        is_scanning_ = false;
        ros_if_->publishCmdVel(0.0, 0.0);
        performTestAttack();
        patrol_points_[current_index_].tested = true;
        ROS_INFO("Scanning finished at point %lu, moving to next point", current_index_ + 1);
        moveToNextPoint();
        return;
    }

    if (isGoalReached()) {
        ROS_INFO("Reached patrol point %lu: x=%f, y=%f, tested=%d", current_index_ + 1,
                 patrol_points_[current_index_].pose.position.x,
                 patrol_points_[current_index_].pose.position.y,
                 patrol_points_[current_index_].tested);
        if (!patrol_points_[current_index_].tested) {
            is_pausing_ = true;
            pause_start_time_ = ros::Time::now().toSec();
            ros_if_->publishCmdVel(0.0, 0.0);
            ROS_INFO("Pausing at point %lu for observation", current_index_ + 1);
        } else {
            moveToNextPoint();
        }
    }
}

void PatrolLogic::setPatrolPoints(const std::vector<PatrolPoint>& points) {
    patrol_points_ = points;
    ros_if_->setPatrolPoints(convertToPoses(points));
    yaml_utils_->savePatrolPoints(convertToPoses(points));
    current_index_ = 0;
    robot_state_->setPatrolIndex(0);
    forward_direction_ = true;
    ROS_INFO("Set %lu patrol points", points.size());
}

bool PatrolLogic::isGoalReached() const {
    if (current_index_ >= patrol_points_.size()) return false;
    Pose current_pose = ros_if_->getRobotInMapOrigin();
    const auto& goal = patrol_points_[current_index_].pose;
    double dx = current_pose.position.x - goal.position.x;
    double dy = current_pose.position.y - goal.position.y;
    return std::sqrt(dx * dx + dy * dy) < goal_tolerance_;
}

void PatrolLogic::performTestAttack() {
    std_msgs::Bool msg;
    msg.data = true;
    ros_if_->attack_pub_.publish(msg);
    ROS_INFO("Test attack triggered at point %lu", current_index_ + 1);
    ros::Duration(1.0).sleep();
    msg.data = false;
    ros_if_->attack_pub_.publish(msg);
}

void PatrolLogic::moveToNextPoint() {
    if (!patrol_active_ || patrol_points_.empty()) {
        patrol_active_ = false;
        robot_state_->setCurrentState(State::IDLE);
        ROS_INFO("Patrol stopped");
        return;
    }

    size_t prev_index = current_index_;
    if (forward_direction_) {
        if (current_index_ + 1 < patrol_points_.size()) {
            current_index_++;
        } else {
            forward_direction_ = false;
            if (current_index_ > 0) current_index_--;
        }
    } else {
        if (current_index_ > 0) {
            current_index_--;
        } else {
            forward_direction_ = true;
            if (current_index_ + 1 < patrol_points_.size()) current_index_++;
        }
    }

    robot_state_->setPatrolIndex(current_index_);
    ros_if_->publishGoal(patrol_points_[current_index_].pose);
    ROS_INFO("Moving to point %lu from %lu (direction: %s): x=%f, y=%f", 
             current_index_ + 1, prev_index + 1, forward_direction_ ? "forward" : "backward",
             patrol_points_[current_index_].pose.position.x,
             patrol_points_[current_index_].pose.position.y);
}

std::vector<Pose> PatrolLogic::convertToPoses(const std::vector<PatrolPoint>& points) {
    std::vector<Pose> poses;
    for (const auto& p : points) {
        poses.push_back(p.pose);
    }
    return poses;
}