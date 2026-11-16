#include "host_console.h"
#include <math.h>
#include <Preferences.h>
#include <algorithm>
#include <string.h>
#include "config.h"
#include "servos.h"

void ServoBank::begin() {
  uart.begin(Config::SERVO_BAUD,SERIAL_8N1,Config::SERVO_RX,Config::SERVO_TX);
  bus.pSerial=&uart; bus.IOTimeOut=8; bus.Level=0;
  loadTeaching();
}
bool ServoBank::byteWrite(int id,int reg,int value) {
  bus.writeByte(id,reg,value); uart.flush(); delay(3);
  return bus.readByte(id,reg)==value;
}
bool ServoBank::wordWrite(int id,int reg,int value) {
  bus.writeWord(id,reg,value); uart.flush(); delay(3);
  return bus.readWord(id,reg)==value;
}
bool ServoBank::sendGoal(int i,int value) {
  int id=Config::SERVO_IDS[i];
  bus.WritePosEx(id,value,Config::SERVO_SPEED,Config::SERVO_ACC); uart.flush(); delay(3);
  if(bus.readWord(id,SMS_STS_GOAL_POSITION_L)!=value) return false;
  goal[i]=value; return true;
}
bool ServoBank::poll(int i) {
  auto& s=status[i]; s.valid=false;
  int received=-1;
  for(int attempt=0;attempt<3;++attempt) {
    received=bus.FeedBack(Config::SERVO_IDS[i]);
    if(received>=0) break;
    if(attempt<2) delay(2);
  }
  if(received<0) {
    hostConsole.printf("SERVO_FEEDBACK_FAIL id=%d after=3_attempts\n",Config::SERVO_IDS[i]);
    if(active_) fault_=true;
    return false;
  }
  s.error=bus.Error; s.position=bus.ReadPos(-1);
  s.sampled=millis();
  s.voltage=bus.ReadVoltage(-1)*0.1f; s.temperature=bus.ReadTemper(-1);
  s.valid=s.error==0 && s.position>=0 && s.position<=4095 &&
    s.voltage>=Config::SERVO_V_MIN && s.voltage<=Config::SERVO_V_MAX &&
    s.temperature<Config::SERVO_TEMP_MAX;
  if(active_ && !s.valid) fault_=true;
  return s.valid;
}
bool ServoBank::sendGoals(const int values[4]) {
  u8 ids[4],acc[4]; s16 positions[4]; u16 speeds[4];
  for(int i=0;i<4;++i) {
    if(!taughtTargetAllowed(i,values[i])) return false;
    ids[i]=Config::SERVO_IDS[i]; positions[i]=values[i];
    speeds[i]=Config::SERVO_SPEED; acc[i]=Config::SERVO_ACC;
  }
  bus.SyncWritePosEx(ids,4,positions,speeds,acc); uart.flush(); delay(3);
  // Broadcast has no ACK: verify each target register before updating cache.
  for(int i=0;i<4;++i) {
    if(bus.readWord(ids[i],SMS_STS_GOAL_POSITION_L)!=values[i]) return false;
  }
  for(int i=0;i<4;++i) goal[i]=values[i];
  return true;
}
bool ServoBank::positionInWindow(int i,int position) {
  const int guard=lroundf(Config::SERVO_RANGE_DEG*4096/360)+8;
  if(position<low[i] || position>high[i] || position<guard || position>4095-guard) {
    hostConsole.printf("BLOCKED servo=%d pos=%d: near 0/4095 wrap or outside limits. Arrange supported mechanical/encoder neutral before enabling; do not force motion.\n",
      Config::SERVO_IDS[i],position);
    return false;
  }
  if(teaching_[i].mask==3) {
    const int lo=std::min(teaching_[i].stand,teaching_[i].lie),hi=std::max(teaching_[i].stand,teaching_[i].lie);
    if(strcmp(teachingState(i),"VALID") || position<lo-4 || position>hi+4) {
      hostConsole.printf("BLOCKED servo=%d: taught interval/state=%s; read limits/reteach first.\n",Config::SERVO_IDS[i],teachingState(i)); return false;
    }
  }
  return true;
}
bool ServoBank::arm() {
  if(setupActive_) { hostConsole.println("BLOCKED: finish/stop unloaded servo4 setup first."); return false; }
  if(active_) { hostConsole.println("SERVOS already active; RELAX first to re-capture centres."); return false; }
  fault_=false; centerValid_=false;
  // All preflight reads before any torque operation.
  for(int i=0;i<4;++i) {
    int id=Config::SERVO_IDS[i];
    if(!poll(i) || bus.readByte(id,SMS_STS_MODE)!=0) return false;
    if(!refreshTeachingHardware(i) || !positionInWindow(i,status[i].position)) return false;
  }
  // Disable ALL torques before capturing the positions we will actually hold.
  // A supported, stationary mechanism is required throughout this operation.
  for(int i=0;i<4;++i) {
    int id=Config::SERVO_IDS[i];
    if(!byteWrite(id,SMS_STS_TORQUE_ENABLE,0)) { release(); return false; }
  }
  for(int i=0;i<4;++i) {
    if(!poll(i) || !positionInWindow(i,status[i].position)) { release(); return false; }
    center[i]=status[i].position;
    for(int sample=0;sample<3;++sample) {
      delay(20);
      if(!poll(i) || !positionInWindow(i,status[i].position) || abs(status[i].position-center[i])>4) {
        hostConsole.printf("BLOCKED servo=%d: position moved while torque OFF; support and stabilise the mechanism.\n",Config::SERVO_IDS[i]);
        release(); return false;
      }
    }
    center[i]=status[i].position;
    if(teaching_[i].mask==3) center[i]=std::max(int(std::min(teaching_[i].stand,teaching_[i].lie)),
      std::min(int(std::max(teaching_[i].stand,teaching_[i].lie)),center[i]));
  }
  for(int i=0;i<4;++i) {
    if(!wordWrite(Config::SERVO_IDS[i],SMS_STS_TORQUE_LIMIT_L,Config::SERVO_TORQUE) || !sendGoal(i,center[i])) {
      release(); return false;
    }
  }
  for(int i=0;i<4;++i) {
    if(!poll(i) || !positionInWindow(i,status[i].position) || abs(status[i].position-center[i])>4) {
      hostConsole.printf("BLOCKED servo=%d: position changed before enabling; no old-centre jump allowed.\n",Config::SERVO_IDS[i]);
      release(); return false;
    }
    hostConsole.printf("servo=%d enable_center=%d goal_readback=%d\n",Config::SERVO_IDS[i],center[i],goal[i]);
  }
  for(int i=0;i<4;++i) if(!byteWrite(Config::SERVO_IDS[i],SMS_STS_TORQUE_ENABLE,1)) {
    release(); return false;
  }
  active_=true; centerValid_=true; return true;
}
bool ServoBank::release() {
  stopTaughtMove("RELAX");
  bool ok=true;
  for(int id:Config::SERVO_IDS) if(!byteWrite(id,SMS_STS_TORQUE_ENABLE,0)) ok=false;
  active_=!ok; fault_=!ok;
  if(ok) { setupActive_=false; setupStopping_=false; }
  return ok;
}
bool ServoBank::pose(const float degrees[4],const int8_t direction[4]) {
  if(!active_ || fault_ || taughtMoving_) return false;
  int next[4];
  for(int i=0;i<4;++i) {
    if(!isfinite(degrees[i]) || fabsf(degrees[i])>Config::SERVO_RANGE_DEG ||
       (direction[i]!=1 && direction[i]!=-1)) return false;
    next[i]=center[i]+lroundf(degrees[i]*direction[i]*4096/360);
    if(next[i]<low[i] || next[i]>high[i] || !taughtTargetAllowed(i,next[i]) ||
       abs(next[i]-goal[i])>lroundf(Config::SERVO_STEP_DEG*4096/360)) return false;
    if(!poll(i)) return false;
  }
  if(!sendGoals(next)) { fault_=true; return false; }
  return true;
}
bool ServoBank::jog(int i,float degrees,int8_t direction) {
  if(!active_ || fault_ || taughtMoving_ || i<0 || i>3 || !isfinite(degrees) ||
     fabsf(degrees)>Config::SERVO_STEP_DEG || (direction!=1 && direction!=-1)) return false;
  int next=goal[i]+lroundf(degrees*direction*4096/360);
  if(next<low[i] || next>high[i] || !taughtTargetAllowed(i,next) ||
     (teaching_[i].mask!=3 && abs(next-center[i])>lroundf(Config::SERVO_RANGE_DEG*4096/360))) return false;
  if(!poll(i) || !sendGoal(i,next)) { fault_=true; return false; }
  return true;
}
void ServoBank::print() {
  for(int i=0;i<4;++i) {
    hostConsole.printf("servo=%d valid=%d pos=%d V=%.1f C=%d error=%02X goal_cache=%d center=%d center_set=%d age_ms=%ld\n",
      Config::SERVO_IDS[i],status[i].valid,status[i].position,status[i].voltage,
      status[i].temperature,status[i].error,goal[i],center[i],centerValid_,
      status[i].sampled ? long(millis()-status[i].sampled) : -1L);
  }
  hostConsole.printf("servos_active=%d servo_fault=%d; STOP/DISARM keep servo torque; RELAX releases\n",active_,fault_);
  hostConsole.printf("unloaded_setup=%d setup_servo=%d; SETUPOFF stops only the selected setup servo\n",setupActive_,Config::SERVO_IDS[setupIndex_]);
}
bool ServoBank::setupCenter(int i) {
  if(i<0 || i>3) return false;
  if(active_ || setupActive_) { hostConsole.println("BLOCKED: normal servos must be OFF; setup must not already be active."); return false; }
  const int id=Config::SERVO_IDS[i];
  if(!poll(i) || bus.readByte(id,33)!=0 ||
     bus.readWord(id,9)>2048 || bus.readWord(id,11)<2048) return false;
  if(!byteWrite(id,40,0)) return false;
  if(!poll(i)) return false;
  if(!clearTeaching(i)) { hostConsole.println("SETUP blocked: cannot invalidate old teaching in NVS."); return false; }
  hostConsole.printf("TEACH invalidated id=%d: mechanical remount setup requires new STAND/LIE points\n",id);
  const int current=status[i].position;
  setupIndex_=i;
  setupActive_=true; setupStopping_=false; setupStable_=0;
  setupStart_=millis(); setupPoll_=0;
  // Explicit UNLOADED setup only: preload current position before torque ON.
  if(!wordWrite(id,48,Config::SERVO_TORQUE) || !sendGoal(i,current) || !poll(i) ||
     abs(status[i].position-current)>4 || !byteWrite(id,40,1) || !sendGoal(i,2048)) {
    stopSetup(); return false;
  }
  hostConsole.printf("SETUP id=%d moving UNLOADED to 2048; other servos untouched. Auto torque OFF on arrival/30s timeout; SETUPOFF aborts.\n",id);
  return true;
}
bool ServoBank::stopSetup() {
  const int id=Config::SERVO_IDS[setupIndex_];
  const bool ok=byteWrite(id,40,0);
  setupActive_=!ok; setupStopping_=!ok; setupPoll_=millis();
  hostConsole.printf("SETUP id=%d torque_off_verified=%d; no EEPROM/ID/offset changes\n",id,ok);
  return ok;
}
bool ServoBank::stopSetup4() {
  if(setupActive_ && setupIndex_!=3) { hostConsole.println("Use SETUPOFF to stop the currently selected setup servo."); return false; }
  setupIndex_=3; return stopSetup();
}
void ServoBank::tickSetup() {
  if(!setupActive_ || millis()-setupPoll_<100) return;
  if(setupStopping_) {
    if(millis()-setupPoll_>=500) stopSetup();
    return;
  }
  setupPoll_=millis();
  const int i=setupIndex_,id=Config::SERVO_IDS[i];
  if(millis()-setupStart_>30000 || !poll(i)) {
    hostConsole.printf("SETUP id=%d stopped: timeout or invalid feedback; inspect unloaded mechanism.\n",id);
    stopSetup(); return;
  }
  setupStable_=abs(status[i].position-2048)<=5 && abs(bus.ReadSpeed(-1))<=5 ? setupStable_+1 : 0;
  if(setupStable_>=3) {
    hostConsole.printf("SETUP id=%d CENTER REACHED pos=%d; support body, power OFF, refit horn/linkage at desired neutral.\n",id,status[i].position);
    stopSetup();
  }
}
void ServoBank::inspect(int i) {
  if(i<0 || i>3) return;
  const int id=Config::SERVO_IDS[i];
  const int mode=bus.readByte(id,33), torque=bus.readByte(id,40);
  const int pos=bus.readWord(id,56), hardwareGoal=bus.readWord(id,42);
  const int limit=bus.readWord(id,48), phase=bus.readByte(id,18), offset=bus.readWord(id,31);
  const int minAngle=bus.readWord(id,9), maxAngle=bus.readWord(id,11);
  const int voltage=bus.readByte(id,62), temperature=bus.readByte(id,63), statusByte=bus.readByte(id,65);
  hostConsole.printf("SERVOINFO id=%d mode=%d torque=%d pos_raw=%d actual_goal_raw=%d torque_limit=%d phase_raw=%d offset_raw=%d\n",
    id,mode,torque,pos,hardwareGoal,limit,phase,offset);
  hostConsole.printf("SERVOINFO id=%d min=%d max=%d voltage_raw=%d temperature=%d status_raw=%d; -1 means read failed; READ ONLY\n",
    id,minAngle,maxAngle,voltage,temperature,statusByte);
}
bool ServoBank::midpointInPlace(int i,bool restoreOriginal) {
  if(i<0 || i>3 || active_ || setupActive_) {
    hostConsole.println("MIDPOINT blocked: normal servo control/setup must be OFF; support and fix the mechanism."); return false;
  }
  const int id=Config::SERVO_IDS[i];
  bool feedbackOK=false;
  int mode=-1,minAngle=-1,maxAngle=-1;
  const unsigned long previousTimeout=bus.IOTimeOut;
  bus.IOTimeOut=20; // Wheels are OFF; tolerate a slow read without bypassing checks.
  for(int attempt=0;attempt<3;++attempt) {
    feedbackOK=poll(i);
    mode=bus.readByte(id,33); minAngle=bus.readWord(id,9); maxAngle=bus.readWord(id,11);
    if(feedbackOK && mode==0 && minAngle==0 && maxAngle==4095) break;
    delay(10);
  }
  bus.IOTimeOut=previousTimeout;
  hostConsole.printf("MIDPOINT PREFLIGHT id=%d feedback_ok=%d mode=%d min=%d max=%d pos=%d V=%.1f C=%d error=%02X; -1=read failed\n",
    id,feedbackOK,mode,minAngle,maxAngle,status[i].position,status[i].voltage,status[i].temperature,status[i].error);
  if(!feedbackOK || mode!=0 || minAngle!=0 || maxAngle!=4095) {
    hostConsole.println("MIDPOINT blocked: see PREFLIGHT values above; no servo offset written. Fix failed reads/actual settings, do not force calibration."); return false;
  }
  if(!byteWrite(id,40,0)) { hostConsole.println("MIDPOINT blocked: torque OFF readback failed; offset unchanged."); return false; }
  if(!poll(i)) { hostConsole.println("MIDPOINT blocked: fresh feedback failed after torque OFF; offset unchanged."); return false; }
  const int position=status[i].position;
  for(int sample=0;sample<3;++sample) {
    delay(20);
    if(!poll(i) || abs(status[i].position-position)>4) {
      hostConsole.println("MIDPOINT blocked: physical position not stationary. Support/fix the desired neutral first."); return false;
    }
  }
  const int oldOffset=bus.readWord(id,31), oldPhase=bus.readByte(id,18), oldGoal=bus.readWord(id,42);
  if(oldOffset<0 || oldOffset>8191 || oldPhase<0 || oldGoal<0) {
    hostConsole.printf("MIDPOINT blocked: backup register read failed/invalid offset=%d phase=%d goal=%d; offset unchanged.\n",oldOffset,oldPhase,oldGoal);
    return false;
  }
  char key[12]; snprintf(key,sizeof(key),"original%d",id);
  Preferences backup;
  if(!backup.begin("single-mid",false)) { hostConsole.println("MIDPOINT blocked: cannot open NVS backup."); return false; }
  int restoreOffset=-1;
  if(restoreOriginal) {
    if(backup.isKey(key)) restoreOffset=backup.getUShort(key);
    backup.end();
    if(restoreOffset<0 || restoreOffset>8191) { hostConsole.println("No original offset backup for this ID on this A board."); return false; }
  } else {
    bool saved=true;
    if(!backup.isKey(key)) saved=backup.putUShort(key,oldOffset)==2;
    snprintf(key,sizeof(key),"previous%d",id); saved=(backup.putUShort(key,oldOffset)==2)&&saved;
    snprintf(key,sizeof(key),"position%d",id); saved=(backup.putUShort(key,position)==2)&&saved;
    backup.end();
    if(!saved) { hostConsole.println("MIDPOINT blocked: backup write failed; servo offset unchanged."); return false; }
  }
  hostConsole.printf("MIDPOINT BACKUP id=%d old_offset_raw=%d old_position=%d old_goal=%d phase=%d; original saved in A-board NVS\n",
    id,oldOffset,position,oldGoal,oldPhase);
  if(!poll(i) || abs(status[i].position-position)>4 || bus.readByte(id,40)!=0) {
    hostConsole.println("MIDPOINT blocked: pose/torque changed during backup; servo offset unchanged."); return false;
  }
  if(!byteWrite(id,55,0)) { bus.LockEprom(id); hostConsole.println("MIDPOINT failed to unlock; no calibration command sent."); return false; }
  if(!clearTeaching(i)) { bus.LockEprom(id); hostConsole.println("MIDPOINT blocked: cannot invalidate old teaching in NVS; offset unchanged."); return false; }
  hostConsole.printf("TEACH invalidated id=%d: coordinate change requires new STAND/LIE points\n",id);
  if(restoreOriginal) bus.writeWord(id,31,restoreOffset);
  else bus.CalibrationOfs(id); // Manufacturer action: register40=128, current position becomes 2048.
  uart.flush(); delay(30);
  const bool off=byteWrite(id,40,0);
  const int newOffset=bus.readWord(id,31);
  const bool readOK=poll(i);
  hardwareOffset_[i]=newOffset; hardwarePhase_[i]=oldPhase;
  low[i]=minAngle; high[i]=maxAngle;
  hardwareKnown_[i]=readOK && newOffset>=0 && newOffset<=8191;
  const bool positionOK=restoreOriginal ? newOffset==restoreOffset : abs(status[i].position-2048)<=5;
  // Keep torque OFF and replace the stale target in the NEW coordinate system.
  const bool goalOK=off && readOK && sendGoal(i,status[i].position);
  const bool locked=byteWrite(id,55,1);
  centerValid_=false; center[i]=-1;
  if(!off || !readOK || !positionOK || !goalOK || !locked || newOffset<0) {
    byteWrite(id,40,0);
    hostConsole.printf("MIDPOINT UNCONFIRMED id=%d offset=%d off=%d position_ok=%d goal_ok=%d locked=%d. Do not enable; inspect/power-cycle before retry.\n",
      id,newOffset,off,positionOK,goalOK,locked);
    return false;
  }
  hostConsole.printf("MIDPOINT %s id=%d position=%d offset_raw=%d actual_goal=%d torque=0 lock=1. Power-cycle and SERVOINFO to verify persistence; no automatic enable.\n",
    restoreOriginal ? "RESTORED" : "CALIBRATED",id,status[i].position,newOffset,goal[i]);
  return true;
}
