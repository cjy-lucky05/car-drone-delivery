#include "HttpServer.h"
#include <pistache/router.h>
#include <tf2/LinearMath/Quaternion.h>
#include <ros/ros.h>

using json = nlohmann::json;

HttpServer::HttpServer(std::shared_ptr<RobotState> robot_state, std::shared_ptr<StateMachine> state_machine,
                       std::shared_ptr<RosInterface> ros_if, std::shared_ptr<YamlUtils> yaml_utils,
                       BufferQueueHandle command_queue, const std::string& address, uint16_t port)
    : robot_state_(robot_state), state_machine_(state_machine), ros_if_(ros_if), yaml_utils_(yaml_utils),
      command_queue_(command_queue) {
    Pistache::Address addr(address, port);
    endpoint_ = std::make_shared<Pistache::Http::Endpoint>(addr);
}

void HttpServer::start() {
    auto opts = Pistache::Http::Endpoint::options().threads(2);
    endpoint_->init(opts);
    setupRoutes();
    endpoint_->setHandler(router_.handler());
    endpoint_->serveThreaded();
}

void HttpServer::stop() {
    endpoint_->shutdown();
}

void HttpServer::setupRoutes() {
    using namespace Pistache::Rest;
    Routes::Get(router_, "/get_map_data", Routes::bind(&HttpServer::handleGetMapData, this));
    Routes::Get(router_, "/get_global_frame", Routes::bind(&HttpServer::handleGetGlobalFrame, this));
    Routes::Get(router_, "/get_object_poses", Routes::bind(&HttpServer::handleGetObjectPoses, this));
    Routes::Get(router_, "/get_robot_pose", Routes::bind(&HttpServer::handleGetRobotPose, this));
    Routes::Post(router_, "/send_goal", Routes::bind(&HttpServer::handleSendGoal, this));
    Routes::Post(router_, "/cancel_goal", Routes::bind(&HttpServer::handleCancelGoal, this));
    Routes::Post(router_, "/set_patrol_points", Routes::bind(&HttpServer::handleSetPatrolPoints, this));
    Routes::Post(router_, "/start_patrol", Routes::bind(&HttpServer::handleStartPatrol, this));
    Routes::Post(router_, "/stop_patrol", Routes::bind(&HttpServer::handleStopPatrol, this));
    Routes::Post(router_, "/exit_manual", Routes::bind(&HttpServer::handleExitManual, this));
    Routes::Post(router_, "/control_gimbal", Routes::bind(&HttpServer::handleControlGimbal, this));
    Routes::Post(router_, "/control_car", Routes::bind(&HttpServer::handleControlCar, this));
}

void HttpServer::handleControlGimbal(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto body = request.body();
        if (body.empty()) {
            ROS_ERROR("Empty request body in control_gimbal");
            response.send(Pistache::Http::Code::Bad_Request, R"({"error":"Empty request body"})");
            return;
        }

        json j_data = json::parse(body);
        if (!j_data.contains("pwm_x") || !j_data.contains("pwm_y")) {
            ROS_ERROR("Missing pwm_x or pwm_y in control_gimbal");
            response.send(Pistache::Http::Code::Bad_Request, R"({"error":"Missing pwm_x or pwm_y"})");
            return;
        }

        int pwm_x = j_data["pwm_x"].get<int>();
        int pwm_y = j_data["pwm_y"].get<int>();
        ROS_INFO("Received gimbal control: pwm_x=%d, pwm_y=%d", pwm_x, pwm_y);

        ros_if_->publishGimbalCmd(pwm_x, pwm_y);

        json event_json = {
            {"type", static_cast<int>(EventType::GIMBAL)},
            {"data", j_data.dump()}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            size_t data_size = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), data_size);
            msg->szData = data_size;   // ✅ 关键的一行
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, R"({"message":"Gimbal command sent"})");
        } else {
            ROS_ERROR("No free buffer in control_gimbal");
            response.send(Pistache::Http::Code::Internal_Server_Error, R"({"error":"No free buffer"})");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in control_gimbal: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleControlCar(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto body = request.body();
        if (body.empty()) {
            ROS_ERROR("Empty request body in control_car");
            response.send(Pistache::Http::Code::Bad_Request, R"({"error":"Empty request body"})");
            return;
        }

        json j_data = json::parse(body);
        if (!j_data.contains("cmd") || !j_data["cmd"].contains("linear_x") || !j_data["cmd"].contains("angular_z")) {
            ROS_ERROR("Missing cmd, linear_x, or angular_z in control_car");
            response.send(Pistache::Http::Code::Bad_Request, R"({"error":"Missing cmd, linear_x, or angular_z"})");
            return;
        }

        double linear_x = j_data["cmd"]["linear_x"].get<double>();
        double angular_z = j_data["cmd"]["angular_z"].get<double>();
        ROS_INFO("Received car control: linear_x=%f, angular_z=%f", linear_x, angular_z);

        ros_if_->publishCmdVel(linear_x, angular_z);

        json event_json = {
            {"type", static_cast<int>(EventType::MOVE)},
            {"data", j_data.dump()}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            size_t data_size = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), data_size);
            msg->szData = data_size;   // ✅ 关键的一行
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, R"({"message":"Car command sent"})");
        } else {
            ROS_ERROR("No free buffer in control_car");
            response.send(Pistache::Http::Code::Internal_Server_Error, R"({"error":"No free buffer"})");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in control_car: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleSetPatrolPoints(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto body = json::parse(request.body());
        if (!body.contains("patrol_points") || !body["patrol_points"].is_array()) {
            throw std::runtime_error("Invalid patrol_points format");
        }
        json event_json = {
            {"type", static_cast<int>(EventType::SET_PATROL_POINTS)},
            {"data", body.dump()}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, json({{"message", "Patrol points set"}}).dump());
        } else {
            response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", "No free buffer"}}).dump());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in set_patrol_points: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleStartPatrol(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto body = request.body();
        if (body.empty()) {
            ROS_ERROR("Empty request body in start_patrol");
            response.send(Pistache::Http::Code::Bad_Request, R"({"error":"Empty request body"})");
            return;
        }

        json points = json::parse(body);
        if (!points.is_array() || points.empty()) {
            ROS_ERROR("No patrol points received or invalid format");
            response.send(Pistache::Http::Code::Bad_Request, R"({"error":"No patrol points received or invalid format"})");
            return;
        }

        ROS_INFO_STREAM("StartPatrol JSON (from client): " << points.dump());

        json event_json = {
            {"type", static_cast<int>(EventType::START_PATROL)},
            {"data", points.dump()}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, R"({"message":"Patrol started"})");
        } else {
            ROS_ERROR("No free buffer in start_patrol");
            response.send(Pistache::Http::Code::Internal_Server_Error, R"({"error":"No free buffer"})");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in start_patrol: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleStopPatrol(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        json event_json = {
            {"type", static_cast<int>(EventType::STOP_PATROL)},
            {"data", ""}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, json({{"message", "Patrol stopped"}}).dump());
        } else {
            response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", "No free buffer"}}).dump());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in stop_patrol: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleSendGoal(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto body = json::parse(request.body());
        json event_json = {
            {"type", static_cast<int>(EventType::SET_GOAL)},
            {"data", body.dump()} // 直接传递原始 JSON
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, json({{"message", "Goal set"}}).dump());
        } else {
            response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", "No free buffer"}}).dump());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in send_goal: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleCancelGoal(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        json event_json = {
            {"type", static_cast<int>(EventType::CANCEL_GOAL)},
            {"data", ""}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, json({{"message", "Goal canceled"}}).dump());
        } else {
            response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", "No free buffer"}}).dump());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in cancel_goal: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleExitManual(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        json event_json = {
            {"type", static_cast<int>(EventType::EXIT_MANUAL)},
            {"data", ""}
        };
        std::string event_data = event_json.dump();
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
            response.send(Pistache::Http::Code::Ok, json({{"message", "Manual mode exited"}}).dump());
        } else {
            response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", "No free buffer"}}).dump());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in exit_manual: %s", e.what());
        response.send(Pistache::Http::Code::Bad_Request, json({{"error", e.what()}}).dump());
    }
}


void HttpServer::handleGetMapData(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto map_msg = ros_if_->getMapMsg();
        if (map_msg) {
            tf2::Quaternion q;
            tf2::fromMsg(map_msg->info.origin.orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            json map_data = {
                {"map_data", std::vector<int8_t>(map_msg->data.begin(), map_msg->data.end())},
                {"map_info", {
                    {"width", map_msg->info.width},
                    {"height", map_msg->info.height},
                    {"resolution", map_msg->info.resolution},
                    {"origin", {
                        {"position", {
                            {"x", map_msg->info.origin.position.x},
                            {"y", map_msg->info.origin.position.y},
                            {"z", map_msg->info.origin.position.z}
                        }},
                        {"orientation", {
                            {"roll", roll},
                            {"pitch", pitch},
                            {"yaw", yaw}
                        }}
                    }}
                }}
            };
            ROS_INFO("Map data: width=%u, height=%u, resolution=%f, origin_x=%f, origin_y=%f",
                     map_msg->info.width, map_msg->info.height, map_msg->info.resolution,
                     map_msg->info.origin.position.x, map_msg->info.origin.position.y);
            response.send(Pistache::Http::Code::Ok, map_data.dump());
        } else {
            ROS_ERROR("Map data not available");
            response.send(Pistache::Http::Code::Bad_Request, json({{"error", "Map data not available"}}).dump());
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error in get_map_data: %s", e.what());
        response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleGetGlobalFrame(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        ros_if_->getGlobalToMapOriginMat();
        auto& mat = ros_if_->global_in_map_origin_mat_;
        json global_frame = {
            {"rotation", {
                {mat(0,0), mat(0,1), mat(0,2)},
                {mat(1,0), mat(1,1), mat(1,2)},
                {mat(2,0), mat(2,1), mat(2,2)}
            }},
            {"translation", {mat(0,3), mat(1,3), mat(2,3)}}
        };
        response.send(Pistache::Http::Code::Ok, global_frame.dump());
    } catch (const std::exception& e) {
        ROS_ERROR("Error in get_global_frame: %s", e.what());
        response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleGetObjectPoses(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto object_poses = ros_if_->getObjectPoseList();
        json poses = json::array();
        for (const auto& pose : object_poses) {
            poses.push_back({
                {"name", pose.name},
                {"position", {
                    {"x", pose.position.x},
                    {"y", pose.position.y},
                    {"z", pose.position.z}
                }}
            });
        }
        response.send(Pistache::Http::Code::Ok, poses.dump());
    } catch (const std::exception& e) {
        ROS_ERROR("Error in get_object_poses: %s", e.what());
        response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", e.what()}}).dump());
    }
}

void HttpServer::handleGetRobotPose(const Pistache::Rest::Request& request, Pistache::Http::ResponseWriter response) {
    try {
        auto pose = ros_if_->getRobotInMapOrigin();
        json robot_pose = {
            {"position", {
                {"x", pose.position.x},
                {"y", pose.position.y},
                {"z", pose.position.z}
            }},
            {"orientation", {
                {"yaw", pose.orientation.yaw}
            }}
        };
        response.send(Pistache::Http::Code::Ok, robot_pose.dump());
    } catch (const std::exception& e) {
        ROS_ERROR("Error in get_robot_pose: %s", e.what());
        response.send(Pistache::Http::Code::Internal_Server_Error, json({{"error", e.what()}}).dump());
    }
}