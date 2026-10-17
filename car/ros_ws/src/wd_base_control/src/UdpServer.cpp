// src/UdpServer.cpp
#include "UdpServer.h"

UdpServer::UdpServer(BufferQueueHandle queue, const std::string& address, uint16_t port)
    : queue_(queue), socket_(io_context_, boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(address), port)) {
    socket_.async_receive_from(boost::asio::buffer(buffer_), remote_endpoint_,
                               [this](const boost::system::error_code& ec, std::size_t bytes) {
                                   handleReceive(ec, bytes);
                               });
}

void UdpServer::start() {
    std::thread([this]() { io_context_.run(); }).detach();
}

void UdpServer::stop() {
    io_context_.stop();
}

void UdpServer::sendFrame(DataQueMsg* frame) {
    socket_.async_send_to(boost::asio::buffer(frame->buff, frame->szData), remote_endpoint_,
                          [this](const boost::system::error_code& ec, std::size_t bytes) {
                              handleSend(ec, bytes);
                          });
    PutFreeBuffer(queue_, frame);
}

void UdpServer::handleReceive(const boost::system::error_code& error, std::size_t bytes_transferred) {
    if (!error) {
        unpackData(buffer_.data(), bytes_transferred);
        socket_.async_receive_from(boost::asio::buffer(buffer_), remote_endpoint_,
                                   [this](const boost::system::error_code& ec, std::size_t bytes) {
                                       handleReceive(ec, bytes);
                                   });
    }
}

void UdpServer::handleSend(const boost::system::error_code& error, std::size_t bytes_transferred) {
    // 发送完成回调
}

void UdpServer::unpackData(const char* data, std::size_t size) {
    // 协议解包
}