// GoalLogic.h
#ifndef GOAL_LOGIC_H
#define GOAL_LOGIC_H

#include "types.h"
#include "RosInterface.h"
#include <memory>
#include <nlohmann/json.hpp>
using nlohmann::json;

class GoalLogic {
public:
    GoalLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<RobotState> robot_state);

    void start();
    void stop();
    void setGoal(const Pose& goal);
    void update();

private:
    bool isGoalReached() const;
    std::shared_ptr<RosInterface> ros_if_;
    std::shared_ptr<RobotState> robot_state_;
    Pose goal_;
    bool goal_active_;
    bool is_testing_;        // 新增：测试（巡查）状态
    double test_start_time_; // 新增：测试开始时间
    double test_duration_;   // 新增：测试持续时间（秒）
    double scanning_speed_;  // 新增：测试时的巡查速度
};

#endif // GOAL_LOGIC_H