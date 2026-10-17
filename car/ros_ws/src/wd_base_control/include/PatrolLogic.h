// include/PatrolLogic.h
#ifndef PATROL_LOGIC_H
#define PATROL_LOGIC_H

#include "RosInterface.h"
#include "RobotState.h"
#include "YamlUtils.h"
#include <memory>
#include <vector>

class PatrolLogic {
public:
    PatrolLogic(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<YamlUtils> yaml_utils,
                std::shared_ptr<RobotState> robot_state);
    void start();
    void stop();
    void update();
    void setPatrolPoints(const std::vector<PatrolPoint>& points);
    bool isGoalReached() const;
    void setSurveyDuration(double duration) { survey_duration_ = duration; }
private:
    void performTestAttack();
    void moveToNextPoint();
    std::vector<Pose> convertToPoses(const std::vector<PatrolPoint>& points);

    std::shared_ptr<RosInterface> ros_if_;
    std::shared_ptr<YamlUtils> yaml_utils_;
    std::shared_ptr<RobotState> robot_state_;
    std::vector<PatrolPoint> patrol_points_;
    size_t current_index_;
    bool patrol_active_;
    double pause_start_time_;
    bool is_pausing_;
    bool is_random_moving_;
    double goal_tolerance_; // 目标点范围
    bool forward_direction_; // 新增：控制巡逻方向

    bool is_surveying_;  // 新增：巡查状态
    double survey_start_time_;  // 巡查开始时间
    double survey_duration_;  // 巡查持续时间（默认5秒）

        // 🚀 需要新增的
    bool is_scanning_;       // 表示是否处于巡查状态
    double scanning_speed_;  // 巡查时的速度
};

#endif // PATROL_LOGIC_H