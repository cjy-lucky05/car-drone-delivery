// src/AttackLogic.cpp
#include "AttackLogic.h"
#include <ros/ros.h>
#include <std_msgs/Bool.h>

AttackLogic::AttackLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<RobotState> robot_state)
    : ros_if_(ros_if), robot_state_(robot_state), attack_active_(false), attack_start_time_(0.0) {}

void AttackLogic::start() {
    attack_active_ = true;
    attack_start_time_ = ros::Time::now().toSec();
    ros_if_->publishCmdVel(0.0, 0.0); // 停止运动
    std_msgs::Bool msg;
    msg.data = true;
    ros_if_->attack_pub_.publish(msg); // 触发发射
    ROS_INFO("Attack started");
}

void AttackLogic::stop() {
    attack_active_ = false;
    ros_if_->cancelGoal(); // 取消导航
    std_msgs::Bool msg;
    msg.data = false;
    ros_if_->attack_pub_.publish(msg); // 停止发射
    ROS_INFO("Attack stopped");
}

void AttackLogic::fireOnce() {
    std_msgs::Bool msg;
    msg.data = true;
    ros_if_->attack_pub_.publish(msg);
    ROS_INFO("Instant fire triggered");
    ros::Duration(1.0).sleep(); // 阻塞移到 Logic 中
    msg.data = false;
    ros_if_->attack_pub_.publish(msg);
}

void AttackLogic::update() {
    if (!attack_active_) return;

    double current_time = ros::Time::now().toSec();
    if (current_time - attack_start_time_ > 3.0) { // 攻击3秒
        stop();
        DataQueMsg* msg = GetFreeBuffer(robot_state_->getCommandQueue(), 1000);
        if (msg) {
            json event_json = {
                {"type", static_cast<int>(EventType::ATTACK_FINISHED)},
                {"data", ""}
            };
            std::string event_data = event_json.dump();
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(robot_state_->getCommandQueue(), msg);
            ROS_INFO("Attack finished event triggered");
        }
    }
}