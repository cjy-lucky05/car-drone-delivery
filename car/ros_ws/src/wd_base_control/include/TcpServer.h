#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include "types.h"
#include "RosInterface.h"
#include "StateMachine.h"
#include "BufferQueue.h"
#include "PacketCodec.h"
#include <boost/asio.hpp>
#include <memory>
#include <vector>

class TcpServer {
public:
    TcpServer(std::shared_ptr<RosInterface> ros_if, std::shared_ptr<StateMachine> state_machine,
              BufferQueueHandle command_queue, const std::string& address, uint16_t port);

    void start();
    void stop();
    void sendResponse(const std::string& response); // 发送状态响应

private:
    void startAccept();
    void handleAccept(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                     const boost::system::error_code& error);
    void handleReceive(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                      const boost::system::error_code& error, std::size_t bytes_transferred);
    void handleSend(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
                   const boost::system::error_code& error, std::size_t bytes_transferred);
    void processMessage(const ParsedMessage& message, std::shared_ptr<boost::asio::ip::tcp::socket> socket);

    std::shared_ptr<RosInterface> ros_if_;
    std::shared_ptr<StateMachine> state_machine_;
    BufferQueueHandle command_queue_;
    boost::asio::io_context io_context_;
    boost::asio::ip::tcp::acceptor acceptor_;
    std::vector<uint8_t> receive_buffer_;
};

#endif // TCP_SERVER_H