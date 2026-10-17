// src/TcpServer.cpp
#include "TcpServer.h"
#include <ros/ros.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

TcpServer::TcpServer(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<StateMachine> state_machine,
                     BufferQueueHandle command_queue, const std::string& address, uint16_t port)
    : ros_if_(ros_if), state_machine_(state_machine), command_queue_(command_queue),
      acceptor_(io_context_, boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string(address), port)),
      receive_buffer_(1024) {
    startAccept();
}

void TcpServer::start() {
    std::thread([this]() { io_context_.run(); }).detach();
}

void TcpServer::stop() {
    io_context_.stop();
}

void TcpServer::startAccept() {
    auto socket = std::make_shared<boost::asio::ip::tcp::socket>(io_context_);
    acceptor_.async_accept(*socket, [this, socket](const boost::system::error_code& error) {
        handleAccept(socket, error);
    });
}

void TcpServer::handleAccept(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                            const boost::system::error_code& error) {
    if (!error) {
        socket->async_read_some(
            boost::asio::buffer(receive_buffer_),
            [this, socket](const boost::system::error_code& ec, std::size_t bytes) {
                handleReceive(socket, ec, bytes);
            });
        startAccept(); // 继续接受新连接
    } else {
        ROS_ERROR("TCP accept error: %s", error.message().c_str());
    }
}

void TcpServer::handleReceive(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                             const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error) {
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            msg->szData = std::min(bytes_transferred, static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, receive_buffer_.data(), msg->szData);
            std::vector<ParsedMessage> messages;
            PacketCodec::Decode(msg, messages);

            for (const auto& parsed : messages) {
                processMessage(parsed, socket);
            }

            PutReadyBuffer(command_queue_, msg);
            socket->async_read_some(
                boost::asio::buffer(receive_buffer_),
                [this, socket](const boost::system::error_code& ec, std::size_t bytes) {
                    handleReceive(socket, ec, bytes);
                });
        } else {
            ROS_WARN("No free buffer for TCP receive");
            socket->async_read_some(
                boost::asio::buffer(receive_buffer_),
                [this, socket](const boost::system::error_code& ec, std::size_t bytes) {
                    handleReceive(socket, ec, bytes);
                });
        }
    } else {
        ROS_ERROR("TCP receive error: %s", error.message().c_str());
    }
}

void TcpServer::handleSend(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                          const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (error) {
        ROS_ERROR("TCP send error: %s", error.message().c_str());
    }
}

void TcpServer::sendResponse(const std::string& response) {
    auto socket = std::make_shared<boost::asio::ip::tcp::socket>(io_context_);
    auto packet = PacketCodec::Encode(MessageType::STATUS_RESP, response);
    boost::asio::async_write(
        *socket,
        boost::asio::buffer(packet),
        [this, socket](const boost::system::error_code& ec, std::size_t bytes) {
            handleSend(socket, ec, bytes);
        });
}

void TcpServer::processMessage(const ParsedMessage& message, std::shared_ptr<boost::asio::ip::tcp::socket> socket) {
    try {
        std::string body(message.body.begin(), message.body.end());
        RobotEvent event;
        event.source = "TCP";
        event.data = body;

        switch (message.type) {
            case MessageType::MOVE: {
                event.type = EventType::MOVE;
                json j = json::parse(body);
                double linear_x = j.value("linear_x", 0.0);
                double linear_y = j.value("linear_y", 0.0);
                double angular_z = j.value("angular_z", 0.0);
                ros_if_->publishCmdVel(linear_x, angular_z); // 直接调用 ROS 接口
                ROS_INFO("Received MOVE: linear_x=%f, linear_y=%f, angular_z=%f", linear_x, linear_y, angular_z);
                break;
            }
            case MessageType::GIMBAL: {
                event.type = EventType::GIMBAL;
                json j = json::parse(body);
                int pwm_x = j.value("pwm_x", 1500);
                int pwm_y = j.value("pwm_y", 1500);
                // 假设云台控制通过 ROS 话题发布（需实现具体话题）
                ROS_INFO("Received GIMBAL: pwm_x=%d, pwm_y=%d", pwm_x, pwm_y);
                break;
            }
            case MessageType::FIRE: {
                event.type = EventType::FIRE;
                json j = json::parse(body);
                std::string action = j.value("action", "stop");
                std_msgs::Bool msg;
                msg.data = (action == "start");
                ros_if_->attack_pub_.publish(msg);
                ROS_INFO("Received FIRE: action=%s", action.c_str());
                break;
            }
            case MessageType::STATUS_REQ: {
                json j;
                j["state"] = static_cast<int>(ros_if_->robot_state_->getCurrentState());
                j["pose"] = {{"x", ros_if_->getRobotInMapOrigin().position.x},
                             {"y", ros_if_->getRobotInMapOrigin().position.y}};
                j["battery"] = ros_if_->robot_state_->getBatteryLevel();
                sendResponse(j.dump());
                ROS_INFO("Received STATUS_REQ");
                return; // 状态请求不进入命令队列
            }
            default:
                ROS_WARN("Unknown message type: %d", static_cast<int>(message.type));
                return;
        }

        // 将事件存入命令队列
        DataQueMsg* msg = GetFreeBuffer(command_queue_, 1000);
        if (msg) {
            std::string event_data = json({{"type", static_cast<int>(event.type)}, {"data", event.data}}).dump();
            msg->szData = std::min(event_data.size(), static_cast<size_t>(msg->szBuf));
            std::memcpy(msg->buff, event_data.data(), msg->szData);
            PutReadyBuffer(command_queue_, msg);
        } else {
            ROS_WARN("No free buffer for command queue");
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Error processing TCP message: %s", e.what());
    }
}