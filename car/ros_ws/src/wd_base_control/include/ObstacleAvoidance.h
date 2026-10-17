#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include "types.h"
#include "RosInterface.h"
#include <memory>

class ObstacleAvoidance {
public:
    ObstacleAvoidance(std::shared_ptr<RosInterface> ros_if);

    bool planPath(const Pose& start, const Pose& goal, std::vector<Pose>& path);

private:
    std::shared_ptr<RosInterface> ros_if_;
};

#endif // OBSTACLE_AVOIDANCE_H