#pragma once
#include <Arduino.h>
#include "servos.h"

// Uses the existing IMU reference. Never writes IMU calibration or servo EEPROM.
class BodyLevel {
  ServoBank& servos_;
  bool active_=false,centering_=false,settled_=false,driveReady_=false;
  int reference_=0;
  float rollTarget_=0,pitchTarget_=0,rollCorrection_=0,pitchCorrection_=0,bestError_=1000;
  uint32_t lastTick_=0,stageStart_=0,stableSince_=0,saturatedSince_=0,dynamicSince_=0;
  const char* state_="OFF";
  bool fail(const char* reason);
public:
  explicit BodyLevel(ServoBank& servos):servos_(servos) {}
  bool preflight();
  bool start(int reference=-1,float roll=0,float pitch=0);
  void stop(const char* reason);
  bool tick(bool calibrating,bool driving,bool driveTransition,bool hostAlive);
  void print() const;
  bool active() const { return active_; }
  bool settled() const { return settled_; }
  bool driveReady() const { return driveReady_; }
  bool adjusting() const { return active_ && !settled_; }
  bool canRetarget(int reference) const { return active_ && !centering_ && reference==reference_; }
};
