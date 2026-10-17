#include "TrackLogic.h"
#include <ros/ros.h>
#include <nlohmann/json.hpp>

TrackLogic::TrackLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<RobotState> robot_state,
                       std::shared_ptr<AttackLogic> attack_logic)
    : ros_if_(ros_if), robot_state_(robot_state), attack_logic_(attack_logic), 
      track_active_(false), track_start_time_(0.0), track_timeout_(30.0),
      fire_count_(0), max_fire_count_(2), track_interval_(5.0), last_fire_time_(0.0) {
    ros::NodeHandle nh;
    nh.param("track_timeout", track_timeout_, 30.0);
    nh.param("max_fire_count", max_fire_count_, 2);
    nh.param("track_interval", track_interval_, 5.0);
    ROS_INFO("TrackLogic initialized with timeout=%f, max_fire_count=%d, track_interval=%f",
             track_timeout_, max_fire_count_, track_interval_);
}

void TrackLogic::start() {
    track_active_ = true;
    track_start_time_ = ros::Time::now().toSec();
    fire_count_ = 0;
    last_fire_time_ = track_start_time_;
    ROS_INFO("Tracking started with max_fire_count=%d, interval=%f", max_fire_count_, track_interval_);
}

void TrackLogic::stop() {
    track_active_ = false;
    ros_if_->publishCmdVel(0.0, 0.0);
    robot_state_->setTargetDetected(false);
    fire_count_ = 0;
    ROS_INFO("Tracking stopped");
}

void TrackLogic::update() {
    if (!track_active_) return;

    double current_time = ros::Time::now().toSec();

    // 检查超时
    if (current_time - track_start_time_ > track_timeout_) {
        stop();
        DataQueMsg* msg = GetFreeBuffer(robot_state_->getCommandQueue(), 1000);
        if (msg) {
            nlohmann::json event_json = {
                {"type", static_cast<int>(EventType::TRACK_COMPLETED)},
                {"data", ""}
            };
            std::string event_data = event_json.dump();
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(robot_state_->getCommandQueue(), msg);
            ROS_INFO("Tracking timeout, event triggered");
        }
        return;
    }

    // 间歇发射逻辑
    if (robot_state_->isTargetDetected() && fire_count_ < max_fire_count_ && 
        current_time - last_fire_time_ > track_interval_) {
        ros_if_->publishCmdVel(0.0, 0.0); // 停止运动
        ROS_INFO("Stopping for fire during tracking, count=%d", fire_count_ + 1);
        attack_logic_->fireOnce(); // 模拟发射
        fire_count_++;
        last_fire_time_ = current_time;
        ROS_INFO("Fire simulated during tracking, continuing track");
    } else if (fire_count_ >= max_fire_count_) {
        stop();
        DataQueMsg* msg = GetFreeBuffer(robot_state_->getCommandQueue(), 1000);
        if (msg) {
            nlohmann::json event_json = {
                {"type", static_cast<int>(EventType::TRACK_COMPLETED)},
                {"data", ""}
            };
            std::string event_data = event_json.dump();
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(robot_state_->getCommandQueue(), msg);
            ROS_INFO("Max fire count reached, tracking completed");
        }
    }
}