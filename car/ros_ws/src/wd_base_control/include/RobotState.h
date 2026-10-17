// include/RobotState.h
#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include "types.h"
#include "BufferQueue.h"
#include <mutex>

enum class State { 
    IDLE, PATROL, ATTACK, GOTO_GOAL, 
    MANUAL_CONTROL, EMERGENCY_STOP,
    TRACK // 新增 TRACK 状态
     };

class RobotState {
public:
    RobotState();

    // 获取/设置机器人位置
    Pose getPose() const;
    void setPose(const Pose& pose);

    // 获取/设置电量
    float getBatteryLevel() const;
    void setBatteryLevel(float level);

    // 获取/设置当前状态
    State getCurrentState() const;
    void setCurrentState(State state);

    // 巡逻点索引
    int getPatrolIndex() const;
    void setPatrolIndex(int index);

    // 获取命令队列
    BufferQueueHandle getCommandQueue() const;
    void setCommandQueue(BufferQueueHandle queue);

    bool isTargetDetected() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return target_detected_;
    }
    void setTargetDetected(bool detected) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_detected_ = detected;
    }
    double getTargetArea() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return target_area_;
    }
    void setTargetArea(double area) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_area_ = area;
    }
    Pose getTargetPose() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return target_pose_;
    }
    void setTargetPose(const Pose& pose) {
        std::lock_guard<std::mutex> lock(mutex_);
        target_pose_ = pose;
    }
private:
    Pose pose_;
    float battery_level_;
    State current_state_;
    int patrol_index_;
    BufferQueueHandle command_queue_;
    bool target_detected_; // 是否检测到目标
    double target_area_;
    Pose target_pose_;    // 目标位置（可选，用于记录最后检测位置）
    mutable std::mutex mutex_;
};

#endif // ROBOT_STATE_H