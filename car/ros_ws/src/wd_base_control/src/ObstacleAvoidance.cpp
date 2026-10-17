// src/ObstacleAvoidance.cpp
#include "ObstacleAvoidance.h"

ObstacleAvoidance::ObstacleAvoidance(std::shared_ptr<RosInterface> ros_if) : ros_if_(ros_if) {}

bool ObstacleAvoidance::planPath(const Pose& start, const Pose& goal, std::vector<Pose>& path) {
    // 使用 ROS move_base 或自研避障算法
    return true; // 占位
}