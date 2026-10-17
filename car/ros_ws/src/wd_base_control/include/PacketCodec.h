#ifndef PACKET_CODEC_H
#define PACKET_CODEC_H

#include <cstdint>
#include <vector>
#include <string>
#include "BufferQueue.h"
#include <ros/ros.h>

enum class MessageType : uint16_t {
    MOVE        = 1,  // 小车运动
    GIMBAL      = 2,  // 云台控制
    FIRE        = 3,  // 发射器控制
    STATUS_REQ  = 4,  // 状态查询
    STATUS_RESP = 5   // 状态响应
};

struct ParsedMessage {
    MessageType type;
    std::vector<uint8_t> body; // JSON字符串
};

class PacketCodec {
public:
    static std::vector<uint8_t> Encode(MessageType type, const std::vector<uint8_t>& body);
    static std::vector<uint8_t> Encode(MessageType type, const std::string& body_str);
    static void Decode(DataQueMsg* msg, std::vector<ParsedMessage>& out_messages);
};

#endif // PACKET_CODEC_H