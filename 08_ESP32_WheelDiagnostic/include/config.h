#pragma once
#include <stdint.h>
namespace Config {
constexpr int SDA=21,SCL=22,ENABLE=13;
constexpr uint8_t MUX=0x70,CHANNEL[4]={7,2,6,1}; // FL,FR,RL,RR; IMU ch0 is never selected.
constexpr int PWM[4][3]={{25,26,27},{18,19,23},{14,16,17},{32,33,4}};
constexpr float VM=6.0f,LIMIT_V=2.5f,ALIGN_V=1.5f;
constexpr int POLES=7;
constexpr float MAX_SPEED=12,OVERSPEED=30,DEFAULT_SECONDS=3,MAX_SECONDS=10;
constexpr uint32_t MAIN_TIMEOUT_MS=500;
}
constexpr bool validRun(float speed,float seconds) {
  return speed!=0 && speed>=-Config::MAX_SPEED && speed<=Config::MAX_SPEED &&
    seconds>=0.1f && seconds<=Config::MAX_SECONDS;
}
constexpr bool expired(uint32_t now,uint32_t deadline) { return int32_t(now-deadline)>=0; }
static_assert(Config::ALIGN_V>0 && Config::ALIGN_V<=Config::LIMIT_V && Config::LIMIT_V<=Config::VM/2,"Invalid voltage limits");
static_assert(validRun(0.5f,3) && validRun(-12,10),"Valid test request rejected");
static_assert(!validRun(13,3) && !validRun(0,3) && !validRun(1,0) && !validRun(1,11),"Unsafe test request accepted");
static_assert(!expired(0xfffffff0u,0x10u) && expired(0x10u,0xfffffff0u),"Timer rollover handling failed");
