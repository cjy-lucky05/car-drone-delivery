// Protocol.h
#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <vector>
#include <cstdint>

uint8_t crc8_maxim(const std::vector<uint8_t>& data);
std::vector<uint8_t> build_frame(uint8_t func_code, const std::vector<uint8_t>& data);

#endif // PROTOCOL_H