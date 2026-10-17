// Protocol.cpp
#include "Protocol.h"

uint8_t crc8_maxim(const std::vector<uint8_t>& data) {
    uint8_t crc = 0;
    for (uint8_t byte : data) {
        crc ^= byte;
        for (int i = 0; i < 8; ++i) {
            if (crc & 0x01) {
                crc = (crc >> 1) ^ 0x8C;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

std::vector<uint8_t> build_frame(uint8_t func_code, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame = {0x5A, 0x00, 0x01, func_code};  // 帧头 | 长度占位 | ID | 功能码
    frame.insert(frame.end(), data.begin(), data.end());  // 数据
    frame.push_back(0x00);  // 预留位
    frame[1] = frame.size() + 1;  // 更新长度（含 CRC）
    uint8_t crc = crc8_maxim(frame);
    frame.push_back(crc);
    return frame;
}