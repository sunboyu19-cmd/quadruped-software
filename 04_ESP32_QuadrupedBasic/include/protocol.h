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

#pragma pack(push,1)
struct Packet {
  uint16_t magic;
  uint8_t version, role; // 1=front master command, 2=rear follower status
  uint32_t boot, sequence, echoBoot, epoch;
  uint8_t mode, state;
  uint16_t fault;
  float target[2], angle[2], velocity[2];
  uint8_t mag[2];
  uint16_t hz, crc;
};
#pragma pack(pop)
static_assert(sizeof(Packet)==54, "Link packet layout changed");

inline void sealPacket(Packet& p) {
  p.magic=0x715a; p.version=2;
  p.crc=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(Packet,crc));
}
inline bool validPacket(const Packet& p) {
  return p.magic==0x715a && p.version==2 && (p.role==1 || p.role==2) &&
    p.mode<=3 && p.state<=4 &&
    p.crc==crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(Packet,crc));
}
// Fixed-length frames, CRC and byte-wise resynchronisation after noise.
class PacketParser {
  uint8_t buffer[sizeof(Packet)] = {};
  size_t used=0;
public:
  uint32_t rejectedWindows=0;
  bool feed(uint8_t b, Packet& out) {
    buffer[used++]=b;
    if (used<sizeof(Packet)) return false;
    memcpy(&out,buffer,sizeof(out));
    if (validPacket(out)) { used=0; return true; }
    ++rejectedWindows; // Sliding windows, NOT a count of lost packets.
    memmove(buffer,buffer+1,sizeof(buffer)-1); used=sizeof(buffer)-1;
    return false;
  }
};
