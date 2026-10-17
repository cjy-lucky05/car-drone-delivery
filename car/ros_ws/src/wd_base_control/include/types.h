#ifndef TYPES_H
#define TYPES_H

#include <string>
#include"Protocol.h"
// 巡逻点/目标点位置
struct Pose {
    struct Position {
        double x, y, z;
    } position;
    struct Orientation {
        double yaw;
    } orientation;
};



// 目标识别点（未使用）
struct ObjectPose {
    std::string name;
    struct Position {
        double x, y, z;
    } position;
};
struct PatrolPoint {
    Pose pose;
    bool tested = false; // 新增：标记是否测试过
};
// 事件类型
enum class EventType {
    START_PATROL,
    STOP_PATROL,
    SET_GOAL,
    CANCEL_GOAL, // 新增：取消导航
    MANUAL_CONTROL,
    SET_PATROL_POINTS, // 新增：设置巡逻点
    MOVE,        // 小车运动
    GIMBAL,      // 云台控制
    FIRE ,        // 发射器控制
    LOW_BATTERY,
    TARGET_DETECTED, // 新增：目标检测
    ATTACK_FINISHED,
    TARGET_LOST, // 新增目标丢失事件
    TRACK_COMPLETED, // 新增追踪完成事件
    EXIT_MANUAL // 新增：退出手动模式
};

// 手动控制数据
struct ManualControlData {
    struct Move {
        double linear_x;
        double linear_y;
        double angular_z;
    } move;
    struct Gimbal {
        int pwm_x;
        int pwm_y;
    } gimbal;
    struct Fire {
        std::string action; // "start" or "stop"
    } fire;
};

// 机器人事件（用于命令队列）
struct RobotEvent {
    EventType type;
    std::string data; // JSON 格式数据
    std::string source; // HTTP 或 TCP
};

#endif // TYPES_H