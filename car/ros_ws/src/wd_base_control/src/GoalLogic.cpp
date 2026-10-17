#include "GoalLogic.h"
#include <ros/ros.h>

GoalLogic::GoalLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<RobotState> robot_state)
    : ros_if_(ros_if), robot_state_(robot_state), goal_active_(false), is_testing_(false),
      test_start_time_(0.0), test_duration_(5.0), scanning_speed_(0.1) {
    ros::NodeHandle nh;
    nh.param("test_duration", test_duration_, 5.0); // 加载测试持续时间
    nh.param("scanning_speed", scanning_speed_, 0.1); // 加载巡查速度
    ROS_INFO("GoalLogic initialized with test_duration=%f, scanning_speed=%f", test_duration_, scanning_speed_);
}

void GoalLogic::start() {
    if (goal_.position.x != 0 || goal_.position.y != 0) {
        goal_active_ = true;
        is_testing_ = false;
        ros_if_->publishGoal(goal_);
        ROS_INFO("Moving to goal: x=%f, y=%f, yaw=%f", 
                 goal_.position.x, goal_.position.y, goal_.orientation.yaw);
    } else {
        ROS_WARN("Invalid goal point");
        goal_active_ = false;
        robot_state_->setCurrentState(State::IDLE);
    }
}

void GoalLogic::stop() {
    goal_active_ = false;
    is_testing_ = false;
    ros_if_->cancelGoal();
    ros_if_->publishCmdVel(0.0, 0.0); // 停止运动
    ROS_INFO("Goal navigation stopped");
}

void GoalLogic::setGoal(const Pose& goal) {
    goal_ = goal;
    robot_state_->setCurrentState(State::GOTO_GOAL);
    start();
}

void GoalLogic::update() {
    if (!goal_active_) return;

    if (is_testing_) {
        double current_time = ros::Time::now().toSec();
        if (current_time - test_start_time_ < test_duration_) {
            ros_if_->publishCmdVel(scanning_speed_, 0.0); // 低速巡查
            ROS_DEBUG("Testing at goal: scanning with speed=%f", scanning_speed_);
            return;
        }
        // 测试结束，返回IDLE
        is_testing_ = false;
        goal_active_ = false;
        ros_if_->publishCmdVel(0.0, 0.0);
        robot_state_->setCurrentState(State::IDLE);
        ROS_INFO("Testing finished at goal, returning to IDLE");
        return;
    }

    if (isGoalReached()) {
        ROS_INFO("Goal reached: x=%f, y=%f, yaw=%f", 
                 goal_.position.x, goal_.position.y, goal_.orientation.yaw);
        goal_active_ = false;
        is_testing_ = true;
        test_start_time_ = ros::Time::now().toSec();
        ros_if_->publishCmdVel(scanning_speed_, 0.0); // 开始测试（巡查）
        ROS_INFO("Starting testing at goal with duration=%f", test_duration_);
    }

    // 使用地图信息验证当前位置
    auto map_msg = ros_if_->getMapMsg();
    if (map_msg) {
        Pose current_pose = ros_if_->getRobotInMapOrigin();
        ROS_DEBUG("Robot pose in map: x=%f, y=%f", current_pose.position.x, current_pose.position.y);
    }
}

bool GoalLogic::isGoalReached() const {
    return ros_if_->isGoalReached();
}