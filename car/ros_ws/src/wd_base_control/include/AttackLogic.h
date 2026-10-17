#ifndef ATTACK_LOGIC_H
#define ATTACK_LOGIC_H

#include "types.h"
#include "RosInterface.h"
#include "RobotState.h"
#include <memory>
#include <nlohmann/json.hpp>
using nlohmann::json;

class AttackLogic {
public:
    AttackLogic(std::shared_ptr<RosInterface> ros_if,
                std::shared_ptr<RobotState> robot_state);

    void start();   // 开始攻击
    void stop();    // 停止攻击
void fireOnce() ;
    void update();  // 更新攻击状态（超时自动停止并触发事件）

private:
    std::shared_ptr<RosInterface> ros_if_;
    std::shared_ptr<RobotState> robot_state_;

    bool attack_active_;     // 是否处于攻击状态
    double attack_start_time_; // 攻击开始时间
};

#endif // ATTACK_LOGIC_H
