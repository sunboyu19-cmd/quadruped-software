#pragma once
#include <Arduino.h>
#include "protocol.h"
struct WheelRequest {
  uint32_t epoch=0, refreshed=0;
  Mode mode=Mode::Off;
  float target[2]={0,0};
};
struct WheelStatus {
  uint32_t epoch=0;
  State state=State::Off;
  uint16_t fault=0, hz=0;
  float angle[2]={0,0}, velocity[2]={0,0};
  uint8_t mag[2]={0,0};
};
void startWheels();
void requestWheels(const WheelRequest& r);
WheelStatus wheelStatus();
