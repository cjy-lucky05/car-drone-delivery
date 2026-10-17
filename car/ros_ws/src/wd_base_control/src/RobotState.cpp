// src/RobotState.cpp
#include "RobotState.h"

RobotState::RobotState() : current_state_(State::IDLE), battery_level_(100.0f), patrol_index_(0), command_queue_(nullptr) {}

Pose RobotState::getPose() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return pose_;
}

void RobotState::setPose(const Pose& pose) {
    std::lock_guard<std::mutex> lock(mutex_);
    pose_ = pose;
}

float RobotState::getBatteryLevel() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return battery_level_;
}

void RobotState::setBatteryLevel(float level) {
    std::lock_guard<std::mutex> lock(mutex_);
    battery_level_ = level;
}

State RobotState::getCurrentState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return current_state_;
}

void RobotState::setCurrentState(State state) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_state_ = state;
}

int RobotState::getPatrolIndex() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return patrol_index_;
}

void RobotState::setPatrolIndex(int index) {
    std::lock_guard<std::mutex> lock(mutex_);
    patrol_index_ = index;
}

BufferQueueHandle RobotState::getCommandQueue() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return command_queue_;
}

void RobotState::setCommandQueue(BufferQueueHandle queue) {
    std::lock_guard<std::mutex> lock(mutex_);
    command_queue_ = queue;
}