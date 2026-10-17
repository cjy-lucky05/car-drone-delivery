#include "RobotState.h"
#include "StateMachine.h"
#include "PatrolLogic.h"
#include "AttackLogic.h"
#include "GoalLogic.h"
#include "TrackLogic.h" // 新增 TrackLogic 头文件
#include "RosInterface.h"
#include "HttpServer.h"
#include "TcpServer.h"
#include "UdpServer.h"
#include "Logger.h"
#include "BufferQueue.h"
#include <thread>
#include <chrono>

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");
    auto robot_state = std::make_shared<RobotState>();
    auto yaml_utils = std::make_shared<YamlUtils>("/home/bye/ros_car/src/wd_base_control/config/config.yaml");
    auto ros_if = std::make_shared<RosInterface>(robot_state);
    auto patrol_logic = std::make_shared<PatrolLogic>(ros_if, yaml_utils, robot_state);
    auto attack_logic = std::make_shared<AttackLogic>(ros_if, robot_state);
    auto goal_logic = std::make_shared<GoalLogic>(ros_if, robot_state);
    auto track_logic = std::make_shared<TrackLogic>(ros_if, robot_state, attack_logic); // 修改：添加 attack_logic
    BufferQueueHandle command_queue = CreateBufferQueue(100, 1024, true);
    auto state_machine = std::make_shared<StateMachine>(robot_state, patrol_logic, attack_logic, goal_logic, track_logic, command_queue);
    auto http_server = std::make_shared<HttpServer>(robot_state, state_machine, ros_if, yaml_utils, command_queue, "0.0.0.0", 8080);
    auto tcp_server = std::make_shared<TcpServer>(ros_if, state_machine, command_queue, "0.0.0.0", 9000);
    BufferQueueHandle video_queue = CreateBufferQueue(100, 1024 * 1024, true);
    auto udp_server = std::make_shared<UdpServer>(video_queue, "0.0.0.0", 9001);
    auto logger = std::make_shared<Logger>("robot.log");

    // 设置命令队列
    robot_state->setCommandQueue(command_queue);

    // 启动服务
    http_server->start();
    // tcp_server->start();
    // udp_server->start();
    ros::AsyncSpinner spinner(2);
    spinner.start();

    // 检查自动巡逻
    if (yaml_utils->loadAutoPatrol()) {
        state_machine->processEvent(EventType::START_PATROL);
    }

    // 主线程循环
    while (ros::ok()) {
        state_machine->tick();
        patrol_logic->update();
        attack_logic->update();
        goal_logic->update();
        track_logic->update(); // 添加 TrackLogic 更新
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 清理
    http_server->stop();
    // tcp_server->stop();
    // udp_server->stop();
    DestroyBufferQueue(command_queue);
    DestroyBufferQueue(video_queue);

    return 0;
}