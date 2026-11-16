#pragma once
#include <stdint.h>
#include <stddef.h>
namespace ServoProtocol {
constexpr bool validFrame(const uint8_t* b,size_t size,uint8_t id,uint8_t payload) {
  if(!b || size!=size_t(payload)+6 || b[0]!=0xff || b[1]!=0xff || b[2]!=id || b[3]!=payload+2) return false;
  uint8_t sum=0;
  for(size_t i=2;i<size;++i) sum=uint8_t(sum+b[i]);
  return sum==0xff;
}
constexpr bool validReply(const uint8_t* b,size_t size,uint8_t id,uint8_t payload,const uint8_t* tx,size_t txSize) {
  if(!validFrame(b,size,id,payload)) return false;
  if(size!=txSize) return true;
  for(size_t i=0;i<size;++i) if(b[i]!=tx[i]) return true;
  return false; // An exact echoed request is not confirmation of a device reply.
}
// Exercise the actual parser at compile time; no runtime actuator commands.
constexpr uint8_t ping[]={0xff,0xff,1,2,0,0xfc};
constexpr uint8_t read[]={0xff,0xff,4,3,0,60,0xbc};
constexpr uint8_t bad[]={0xff,0xff,1,2,0,0xfd};
constexpr uint8_t pingRequest[]={0xff,0xff,1,2,1,0xfb};
constexpr uint8_t idRequest[]={0xff,0xff,1,4,2,5,1,0xf2};
static_assert(validFrame(ping,sizeof(ping),1,0),"Valid ping rejected");
static_assert(validFrame(read,sizeof(read),4,1),"Valid read rejected");
static_assert(!validFrame(ping,sizeof(ping),2,0),"Wrong ID accepted");
static_assert(!validFrame(read,sizeof(read),4,0),"Wrong payload length accepted");
static_assert(!validFrame(ping,sizeof(ping)-1,1,0),"Truncated frame accepted");
static_assert(!validFrame(bad,sizeof(bad),1,0),"Bad checksum accepted");
static_assert(validReply(ping,sizeof(ping),1,0,pingRequest,sizeof(pingRequest)),"Real ping reply rejected");
static_assert(!validReply(pingRequest,sizeof(pingRequest),1,0,pingRequest,sizeof(pingRequest)),"Ping echo accepted as real reply");
static_assert(!validReply(idRequest,sizeof(idRequest),1,1,idRequest,sizeof(idRequest)),"ID READ request accepted as reply");
}
