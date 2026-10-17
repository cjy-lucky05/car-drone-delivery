// include/StateMachine.h
#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include "RobotState.h"
#include "PatrolLogic.h"
#include "AttackLogic.h"
#include "GoalLogic.h"
#include "BufferQueue.h"
#include <memory>
#include "TrackLogic.h"

class StateMachine {
public:
    StateMachine(std::shared_ptr<RobotState> robot_state, std::shared_ptr<PatrolLogic> patrol_logic,
                 std::shared_ptr<AttackLogic> attack_logic, std::shared_ptr<GoalLogic> goal_logic,
                 std::shared_ptr<TrackLogic> track_logic, BufferQueueHandle command_queue);

    void processEvent(EventType event, const std::string& data = "");
    void tick();

private:
    void enterState(State state);
    void exitState(State state);
    void processCommandQueue();

    std::shared_ptr<RobotState> robot_state_;
    std::shared_ptr<PatrolLogic> patrol_logic_;
    std::shared_ptr<AttackLogic> attack_logic_;
    std::shared_ptr<GoalLogic> goal_logic_;
    BufferQueueHandle command_queue_;
    State previous_state_;
    State resume_state_;
    std::shared_ptr<TrackLogic> track_logic_;
};

#endif // STATE_MACHINE_H