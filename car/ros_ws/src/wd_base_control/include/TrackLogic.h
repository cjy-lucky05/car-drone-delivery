#ifndef TRACK_LOGIC_H
#define TRACK_LOGIC_H

#include "RosInterface.h"
#include "RobotState.h"
#include "AttackLogic.h"
#include <ros/ros.h>
#include <memory>

class TrackLogic {
private:
    std::shared_ptr<RosInterface> ros_if_;
    std::shared_ptr<RobotState> robot_state_;
    std::shared_ptr<AttackLogic> attack_logic_; // 新增 AttackLogic 依赖
    bool track_active_;
    double track_start_time_;
    double track_timeout_;
    int fire_count_; // 当前发射次数
    int max_fire_count_; // 最大发射次数
    double track_interval_; // 追踪间隔
    double last_fire_time_; // 上次发射时间

public:
    TrackLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<RobotState> robot_state, 
               std::shared_ptr<AttackLogic> attack_logic);
    void start();
    void stop();
    void update();
    void setTrackTimeout(double timeout) { track_timeout_ = timeout; }
};

#endif