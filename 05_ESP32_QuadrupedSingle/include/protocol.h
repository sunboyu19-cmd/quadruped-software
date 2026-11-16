#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>

enum class Mode : uint8_t { Off=0, Calibrate=1, Run=2, Restore=3 };
enum class State : uint8_t { Off=0, Calibrating=1, Ready=2, Running=3, Fault=4 };

inline uint16_t crc16(const uint8_t* p, size_t n) {
  uint16_t crc = 0xffff;
  while (n--) {
    crc ^= uint16_t(*p++) << 8;
    for (int i=0;i<8;++i) crc = crc & 0x8000 ? (crc << 1)^0x1021 : crc << 1;
  }
  return crc;
}

