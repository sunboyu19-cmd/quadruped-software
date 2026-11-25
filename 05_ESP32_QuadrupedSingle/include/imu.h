#pragma once
#include <Arduino.h>

enum class ImuAction:uint8_t { Calibrate,Cancel,Save };
struct ImuStatus {
  bool valid=false,calibrated=false,saved=false,calibrating=false;
  uint16_t samples=0;
  uint32_t sampled=0,errors=0;
  float ax=0,ay=0,az=0,gx=0,gy=0,gz=0,temperature=0;
  float roll=0,pitch=0,yaw=0,yawRate=0; // Angle in degrees, yawRate in degrees/s.
};
// Only the wheel task calls begin/tick, after Wire has been configured.
void imuBegin();
void imuTick(bool wheelsOff,bool calibrationSafe);
ImuStatus imuStatus();
bool imuBusy();
bool requestImuAction(ImuAction action);
void printImu();
