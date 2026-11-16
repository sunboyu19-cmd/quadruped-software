#pragma once
#include <stdint.h>
#include <stdlib.h>
#include "protocol.h"

struct TeachingRecord {
  int16_t stand=-1, lie=-1;
  uint16_t offset=0;
  uint8_t phase=0, mask=0; // bit0 standing, bit1 lying
  uint16_t version=1, crc=0;
};
static_assert(sizeof(TeachingRecord)==12,"Teaching storage format changed");
inline void sealTeaching(TeachingRecord& r) {
  r.version=1; r.crc=crc16(reinterpret_cast<const uint8_t*>(&r),offsetof(TeachingRecord,crc));
}
inline bool validTeachingStorage(const TeachingRecord& r) {
  return r.version==1 && r.mask<=3 && r.offset<=8191 &&
    (!(r.mask&1) || (r.stand>=0 && r.stand<=4095)) &&
    (!(r.mask&2) || (r.lie>=0 && r.lie<=4095)) &&
    r.crc==crc16(reinterpret_cast<const uint8_t*>(&r),offsetof(TeachingRecord,crc));
}
inline const char* teachingGeometry(const TeachingRecord& r,int guard) {
  if(!r.mask) return "EMPTY";
  if(!(r.mask&1)) return "NEED_STAND";
  if(!(r.mask&2)) return "NEED_LIE";
  if(abs(int(r.stand)-r.lie)>2048) return "WRAP";
  if(abs(int(r.stand)-r.lie)<16) return "TOO_CLOSE";
  if(r.stand<guard || r.stand>4095-guard || r.lie<guard || r.lie>4095-guard) return "NEAR_ZERO";
  return "VALID";
}
