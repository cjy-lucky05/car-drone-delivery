#ifndef UDP_SERVER_H
#define UDP_SERVER_H

#include "BufferQueue.h"
#include <boost/asio.hpp>
#include <memory>

class UdpServer {
public:
    UdpServer(BufferQueueHandle queue, const std::string& address, uint16_t port);

    void start();
    void stop();
    void sendFrame(DataQueMsg* frame); // 视频帧发送接口

private:
    void handleReceive(const boost::system::error_code& error, std::size_t bytes_transferred);
    void handleSend(const boost::system::error_code& error, std::size_t bytes_transferred);
    void unpackData(const char* data, std::size_t size); // 协议解包接口

    BufferQueueHandle queue_;
    boost::asio::io_context io_context_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint remote_endpoint_;   // 🔹 缺少的成员变量
    std::array<char, 1024> buffer_;
};

#endif // UDP_SERVER_H