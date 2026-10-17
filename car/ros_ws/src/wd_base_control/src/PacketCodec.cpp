#include "PacketCodec.h"
#include <arpa/inet.h>
#include <cstring>

std::vector<uint8_t> PacketCodec::Encode(MessageType type, const std::vector<uint8_t>& body) {
    uint16_t net_type = htons(static_cast<uint16_t>(type));
    uint16_t net_len = htons(static_cast<uint16_t>(body.size()));

    std::vector<uint8_t> packet;
    packet.resize(4 + body.size());

    memcpy(packet.data(), &net_type, 2);
    memcpy(packet.data() + 2, &net_len, 2);
    if (!body.empty()) {
        memcpy(packet.data() + 4, body.data(), body.size());
    }

    return packet;
}

std::vector<uint8_t> PacketCodec::Encode(MessageType type, const std::string& body_str) {
    return Encode(type, std::vector<uint8_t>(body_str.begin(), body_str.end()));
}

void PacketCodec::Decode(DataQueMsg* msg, std::vector<ParsedMessage>& out_messages) {
    if (!msg || msg->szData < 4) return;

    size_t offset = 0;
    while (msg->szData - offset >= 4) {
        uint16_t net_type, net_len;
        memcpy(&net_type, msg->buff + offset, 2);
        memcpy(&net_len, msg->buff + offset + 2, 2);

        uint16_t type = ntohs(net_type);
        uint16_t len = ntohs(net_len);

        if (msg->szData - offset < 4 + len) break;

        ParsedMessage parsed;
        parsed.type = static_cast<MessageType>(type);
        parsed.body.insert(parsed.body.end(), msg->buff + offset + 4, msg->buff + offset + 4 + len);

        out_messages.push_back(std::move(parsed));
        offset += 4 + len;
    }

    if (offset < msg->szData) {
        memmove(msg->buff, msg->buff + offset, msg->szData - offset);
        msg->szData -= offset;
    } else {
        msg->szData = 0;
    }
}