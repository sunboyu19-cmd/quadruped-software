#pragma once
#include <Arduino.h>
#include <SCServo.h>
#include "teaching.h"
#include "auto_follow.h"
enum class LevelStep { Failed,Waiting,Applied };
struct JointStatus {
  bool valid=false;
  int position=0, temperature=0;
  int loadRaw=0, currentRaw=0, moving=0;
  float voltage=0;
  uint8_t error=0;
  uint32_t sampled=0;
};
class ServoBank {
  HardwareSerial uart{1};
  SMS_STS bus;
  bool active_=false, fault_=false;
  bool centerValid_=false;
  bool setupActive_=false, setupStopping_=false;
  uint32_t setupStart_=0, setupPoll_=0;
  int setupStable_=0, setupIndex_=3;
  int center[4]={-1,-1,-1,-1}, goal[4]={-1,-1,-1,-1}, low[4]={}, high[4]={};
  bool byteWrite(int id,int reg,int value);
  bool wordWrite(int id,int reg,int value);
  bool sendGoal(int index,int goal);
  bool sendGoals(const int values[4],bool fastPosture=false);
  bool positionInWindow(int index,int position);
  TeachingRecord teaching_[4];
  int hardwareOffset_[4]={-1,-1,-1,-1}, hardwarePhase_[4]={-1,-1,-1,-1};
  bool hardwareKnown_[4]={false,false,false,false};
  uint8_t hardwareReadError_[4]={0,0,0,0};
  bool taughtMoving_=false,taughtCentering_=false,taughtReached_=false,releaseAfterLower_=false;
  int taughtGoal_[4]={};
  int taughtOrigin_[4]={};
  float taughtProgress_=0,taughtStep_=1;
  uint32_t taughtTick_=0,taughtStart_=0;
  int taughtStable_=0;
  AutoFollow::Gate automaticFollow_;
  void loadTeaching();
  bool storeTeaching(int index);
  bool refreshTeachingHardware(int index);
  bool taughtTargetAllowed(int index,int target) const;
public:
  JointStatus status[4];
  void begin();
  bool poll(int index);
  bool arm();
  bool release();
  void lowerThenRelease();
  bool pose(const float degrees[4],const int8_t direction[4]);
  bool jog(int index,float degrees,int8_t direction);
  LevelStep levelFractions(const float fractions[4]);
  bool active() const { return active_; }
  bool fault() const { return fault_; }
  void print();
  void trialPositionPid(int index,int proportional,int integral,int limit);
  void inspect(int index);
  bool midpointInPlace(int index,bool restoreOriginal=false);
  bool setupCenter(int index);
  bool setupCenter4() { return setupCenter(3); }
  bool stopSetup();
  bool stopSetup4();
  void tickSetup();
  bool setupActive() const { return setupActive_; }
  const char* teachingState(int index) const;
  int teachingDirection(int index) const;
  bool teachingLimits();
  bool captureTeaching(bool standing,int index=-1);
  bool saveTeaching();
  bool clearTeaching(int index=-1);
  void printTeaching();
  bool beginTaughtMove(float standFraction,uint8_t mask=15,bool relative=false,bool automaticCentering=false);
  void tickTaughtMove();
  void stopTaughtMove(const char* reason);
  bool taughtMoving() const { return taughtMoving_; }
  bool taughtReached() const { return taughtReached_; }
};
