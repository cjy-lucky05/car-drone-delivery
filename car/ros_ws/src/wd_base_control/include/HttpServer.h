#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H
#include "types.h"
#include "RobotState.h"
#include "StateMachine.h"
#include "RosInterface.h"
#include "YamlUtils.h"
#include "BufferQueue.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nlohmann/json.hpp>
#include <pistache/endpoint.h>
#include <pistache/router.h>    
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pistache/http.h>  

class HttpServer {
public:
    HttpServer(std::shared_ptr<RobotState> robot_state, std::shared_ptr<StateMachine> state_machine,
               std::shared_ptr<RosInterface> ros_if, std::shared_ptr<YamlUtils> yaml_utils,
               BufferQueueHandle command_queue, const std::string& address, uint16_t port);

    void start();
    void stop();

private:
    void setupRoutes();
    void handleGetMapData(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleGetGlobalFrame(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleGetObjectPoses(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleGetRobotPose(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleSendGoal(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleCancelGoal(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleSetPatrolPoints(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleStopPatrol(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleStartPatrol(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleExitManual(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleControlGimbal(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleControlCar(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);
    void handleFire(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response);

    std::shared_ptr<RobotState> robot_state_;
    std::shared_ptr<StateMachine> state_machine_;
    std::shared_ptr<RosInterface> ros_if_;
    std::shared_ptr<YamlUtils> yaml_utils_;
    BufferQueueHandle command_queue_;
    std::shared_ptr<Pistache::Http::Endpoint> endpoint_;
    Pistache::Rest::Router router_;
};

#endif // HTTP_SERVER_H