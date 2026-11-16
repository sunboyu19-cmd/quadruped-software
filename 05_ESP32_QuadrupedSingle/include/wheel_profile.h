#pragma once
#include "protocol.h"
struct WheelDirectionProfile {
  int8_t direction[4]={1,1,1,1};
  uint8_t mask=0,version=1;
  uint16_t crc=0;
};
static_assert(sizeof(WheelDirectionProfile)==8,"Wheel profile format changed");
inline void sealWheelProfile(WheelDirectionProfile& p) {
  p.version=1; p.crc=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(WheelDirectionProfile,crc));
}
inline bool validWheelProfile(const WheelDirectionProfile& p) {
  if(p.version!=1 || p.mask>15 || p.crc!=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(WheelDirectionProfile,crc))) return false;
  for(int8_t sign:p.direction) if(sign!=1 && sign!=-1) return false;
  return true;
}
