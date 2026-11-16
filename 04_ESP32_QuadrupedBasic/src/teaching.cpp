#include "host_console.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include "config.h"
#include "servos.h"

void ServoBank::loadTeaching() {
  Preferences p;
  if(!p.begin("servo-teach",true)) return;
  for(int i=0;i<4;++i) {
    char key[8]; snprintf(key,sizeof(key),"joint%d",Config::SERVO_IDS[i]);
    TeachingRecord r;
    if(p.getBytesLength(key)==sizeof(r) && p.getBytes(key,&r,sizeof(r))==sizeof(r) && validTeachingStorage(r)) teaching_[i]=r;
  }
  p.end();
}
bool ServoBank::storeTeaching(int i) {
  Preferences p;
  if(!p.begin("servo-teach",false)) return false;
  char key[8]; snprintf(key,sizeof(key),"joint%d",Config::SERVO_IDS[i]);
  sealTeaching(teaching_[i]);
  bool ok=p.putBytes(key,&teaching_[i],sizeof(teaching_[i]))==sizeof(teaching_[i]);
  p.end(); return ok;
}
bool ServoBank::refreshTeachingHardware(int i) {
  const int id=Config::SERVO_IDS[i];
  hardwareKnown_[i]=false;
  for(int attempt=0;attempt<3;++attempt) {
    const int min=bus.readWord(id,9),max=bus.readWord(id,11);
    const int offset=bus.readWord(id,31),phase=bus.readByte(id,18);
    hardwareReadError_[i]=(min<0 ? 1:0)|(max<0 ? 2:0)|(offset<0 ? 4:0)|(phase<0 ? 8:0);
    if(!hardwareReadError_[i] && (max>4095 || min>=max || offset>8191 || phase>255)) hardwareReadError_[i]=16;
    if(!hardwareReadError_[i]) {
      low[i]=min; high[i]=max; hardwareOffset_[i]=offset; hardwarePhase_[i]=phase;
      hardwareKnown_[i]=true; return true;
    }
    if(attempt<2) delay(2);
  }
  hostConsole.printf("SERVO_CONFIG_FAIL id=%d mask=%u; 1=min_read 2=max_read 4=offset_read 8=phase_read 16=invalid_values; teaching points unchanged\n",id,hardwareReadError_[i]);
  return false;
}
const char* ServoBank::teachingState(int i) const {
  const char* geometry=teachingGeometry(teaching_[i],lroundf(Config::SERVO_RANGE_DEG*4096/360)+8);
  if(strcmp(geometry,"VALID")) return geometry;
  if(!hardwareKnown_[i]) return hardwareReadError_[i] ? "CONFIG_READ_FAILED":"UNVERIFIED";
  if(teaching_[i].offset!=hardwareOffset_[i] || teaching_[i].phase!=hardwarePhase_[i]) return "STALE_ZERO";
  if(teaching_[i].stand<low[i] || teaching_[i].stand>high[i] || teaching_[i].lie<low[i] || teaching_[i].lie>high[i]) return "HW_LIMIT";
  return "VALID";
}
int ServoBank::teachingDirection(int i) const {
  if(strcmp(teachingState(i),"VALID")) return 0;
  return teaching_[i].stand>teaching_[i].lie ? 1 : -1;
}
bool ServoBank::taughtTargetAllowed(int i,int target) const {
  if(teaching_[i].mask!=3) return true; // Incomplete teaching retains the original small test window.
  if(strcmp(teachingState(i),"VALID")) return false;
  const int lo=std::min(teaching_[i].stand,teaching_[i].lie),hi=std::max(teaching_[i].stand,teaching_[i].lie);
  return target>=lo && target<=hi;
}
bool ServoBank::teachingLimits() {
  bool ok=true;
  for(int i=0;i<4;++i) {
    const bool feedback=poll(i);
    const bool config=refreshTeachingHardware(i); // Do not silently skip on bad feedback.
    if(!feedback || !config) ok=false;
  }
  printTeaching(); print();
  hostConsole.printf("SERVO_CONFIG_CHECK ok=%d; read-only configuration check, NOT stand/lie capture\n",ok);
  return ok;
}
void ServoBank::printTeaching() {
  for(int i=0;i<4;++i) {
    const auto& t=teaching_[i];
    const int swlo=t.mask==3 ? std::min(t.stand,t.lie) : -1;
    const int swhi=t.mask==3 ? std::max(t.stand,t.lie) : -1;
    const int guard=lroundf(Config::SERVO_RANGE_DEG*4096/360)+8;
    const bool nearZero=status[i].valid && (status[i].position<guard || status[i].position>4095-guard);
    hostConsole.printf("TEACHDATA id=%d pos=%d feedback=%d hw_min=%d hw_max=%d hw_known=%d stand=%d lie=%d sw_min=%d sw_max=%d dir=%d state=%s offset=%d taught_offset=%u near_zero=%d age_ms=%ld\n",
      Config::SERVO_IDS[i],status[i].position,status[i].valid,low[i],high[i],hardwareKnown_[i],
      t.mask&1 ? t.stand:-1,t.mask&2 ? t.lie:-1,swlo,swhi,teachingDirection(i),teachingState(i),hardwareOffset_[i],t.offset,nearZero,
      status[i].sampled ? long(millis()-status[i].sampled):-1L);
  }
}
bool ServoBank::captureTeaching(bool standing,int selected) {
  if(active_ || setupActive_ || taughtMoving_ || selected< -1 || selected>3) {
    hostConsole.println("TEACH blocked: support body, RELAX and physically fix the desired pose first."); return false;
  }
  const int first=selected<0 ? 0:selected,last=selected<0 ? 3:selected;
  TeachingRecord candidates[4];
  int initial[4]={};
  for(int i=first;i<=last;++i) {
    if(!poll(i) || !refreshTeachingHardware(i) || bus.readByte(Config::SERVO_IDS[i],33)!=0 ||
       bus.readByte(Config::SERVO_IDS[i],40)!=0) {
      hostConsole.printf("TEACH blocked id=%d: require valid feedback, mode0 and actual torque OFF.\n",Config::SERVO_IDS[i]); return false;
    }
    if(status[i].position<low[i] || status[i].position>high[i]) {
      hostConsole.printf("TEACH blocked id=%d: physical position outside internal servo limits.\n",Config::SERVO_IDS[i]); return false;
    }
    initial[i]=status[i].position;
    candidates[i]=teaching_[i];
    if(candidates[i].mask && (candidates[i].offset!=hardwareOffset_[i] || candidates[i].phase!=hardwarePhase_[i])) candidates[i]=TeachingRecord{};
    candidates[i].offset=hardwareOffset_[i]; candidates[i].phase=hardwarePhase_[i];
  }
  for(int sample=0;sample<3;++sample) {
    delay(20);
    for(int i=first;i<=last;++i) {
      if(!poll(i)) return false;
      int delta=abs(status[i].position-initial[i]); delta=std::min(delta,4096-delta);
      if(delta>4) { hostConsole.printf("TEACH blocked id=%d: pose moved; hold still.\n",Config::SERVO_IDS[i]); return false; }
    }
  }
  for(int i=first;i<=last;++i) {
    if(standing) { candidates[i].stand=status[i].position; candidates[i].mask|=1; }
    else { candidates[i].lie=status[i].position; candidates[i].mask|=2; }
    teaching_[i]=candidates[i];
  }
  hostConsole.printf("TEACH captured %s in RAM; no motion or EEPROM writes. TEACHSAVE persists.\n",standing ? "STAND":"LIE");
  printTeaching(); return true;
}
bool ServoBank::saveTeaching() {
  bool ok=true;
  for(int i=0;i<4;++i) if(!storeTeaching(i)) ok=false;
  hostConsole.printf("TEACH saved_to_A_NVS=%d; servo EEPROM limits unchanged\n",ok);
  return ok;
}
bool ServoBank::clearTeaching(int selected) {
  if(selected< -1 || selected>3) return false;
  bool ok=true;
  for(int i=0;i<4;++i) if(selected<0 || selected==i) {
    teaching_[i]=TeachingRecord{};
    if(!storeTeaching(i)) ok=false;
  }
  return ok;
}
bool ServoBank::beginTaughtMove(float blend,uint8_t mask,bool relative) {
  const bool retarget=taughtMoving_;
  if(!active_ || fault_ || setupActive_ || !isfinite(blend) || (relative ? fabsf(blend)>0.02f : (blend<0 || blend>1)) || (mask!=3 && mask!=12 && mask!=15)) {
    hostConsole.printf("POSTURE blocked active=%d fault=%d setup=%d requested_fraction=%.3f; inspect S before re-enabling\n",
      active_,fault_,setupActive_,blend);
    return false;
  }
  int next[4];
  for(int i=0;i<4;++i) {
    if(!poll(i) || !refreshTeachingHardware(i) || strcmp(teachingState(i),"VALID")) {
      hostConsole.printf("POSTURE blocked id=%d state=%s: read limits/reteach or handle zero first.\n",Config::SERVO_IDS[i],teachingState(i)); return false;
    }
    const int lo=std::min(teaching_[i].stand,teaching_[i].lie),hi=std::max(teaching_[i].stand,teaching_[i].lie);
    if(status[i].position<lo-4 || status[i].position>hi+4) {
      hostConsole.printf("POSTURE blocked id=%d: current position outside taught interval.\n",Config::SERVO_IDS[i]); return false;
    }
    if(abs(status[i].position-goal[i])>64) {
      hostConsole.printf("POSTURE blocked id=%d: following error too large; support/check load before moving.\n",Config::SERVO_IDS[i]); return false;
    }
    next[i]=goal[i];
    if(mask&(1<<i)) {
      const int span=int(teaching_[i].stand)-int(teaching_[i].lie);
      next[i]=relative ? lroundf(goal[i]+blend*span) : lroundf(teaching_[i].lie+blend*span);
      next[i]=std::max(lo,std::min(hi,next[i]));
    }
    if(!taughtTargetAllowed(i,next[i])) {
      hostConsole.printf("POSTURE blocked id=%d target=%d state=%s: target outside valid taught range\n",
        Config::SERVO_IDS[i],next[i],teachingState(i));
      return false;
    }
  }
  int maxDistance=1;
  for(int i=0;i<4;++i) {
    taughtGoal_[i]=next[i]; taughtOrigin_[i]=goal[i];
    maxDistance=std::max(maxDistance,abs(next[i]-goal[i]));
  }
  taughtProgress_=0; taughtStep_=25.0f/maxDistance;
  taughtMoving_=true; taughtStart_=millis(); taughtStable_=0;
  // Retargeting must not buy an extra trajectory step. Keep the previous
  // dispatch deadline even when many BLEND commands arrive in one cycle.
  if(!retarget) taughtTick_=millis();
  hostConsole.printf("POSTURE started group=%s stand_fraction=%.3f; shared-progress sync-write, STOP holds last target\n",
    mask==3 ? "FRONT":mask==12 ? "REAR":"ALL",blend);
  return true;
}
void ServoBank::stopTaughtMove(const char* reason) {
  if(taughtMoving_) hostConsole.printf("POSTURE stopped: %s; last servo targets held\n",reason);
  taughtMoving_=false;
}
void ServoBank::tickTaughtMove() {
  if(!taughtMoving_ || millis()-taughtTick_<50) return;
  taughtTick_=millis();
  if(!active_ || fault_ || millis()-taughtStart_>45000) { stopTaughtMove("inactive/fault/45s timeout"); return; }
  bool settled=true;
  int next[4]; bool changed=false;
  const float progress=std::min(1.0f,taughtProgress_+taughtStep_);
  for(int i=0;i<4;++i) {
    if(!poll(i) || !taughtTargetAllowed(i,taughtGoal_[i])) { fault_=true; stopTaughtMove("feedback or range failure"); return; }
    const int lo=std::min(teaching_[i].stand,teaching_[i].lie),hi=std::max(teaching_[i].stand,teaching_[i].lie);
    if(status[i].position<lo-12 || status[i].position>hi+12) { fault_=true; stopTaughtMove("measured position outside taught interval"); return; }
    if(abs(status[i].position-goal[i])>64) {
      hostConsole.printf("POSTURE_FOLLOW id=%d pos=%d goal=%d error_counts=%d voltage=%.1f temperature=%d\n",
        Config::SERVO_IDS[i],status[i].position,goal[i],status[i].position-goal[i],status[i].voltage,status[i].temperature);
      fault_=true; stopTaughtMove("servo cannot follow target; check load/torque"); return;
    }
    next[i]=lroundf(taughtOrigin_[i]+progress*(taughtGoal_[i]-taughtOrigin_[i]));
    if(!taughtTargetAllowed(i,next[i])) { fault_=true; stopTaughtMove("range failure"); return; }
    changed=changed || next[i]!=goal[i];
    if(next[i]!=taughtGoal_[i] || abs(status[i].position-taughtGoal_[i])>8) settled=false;
  }
  if(changed && !sendGoals(next)) { fault_=true; stopTaughtMove("sync target readback failure"); return; }
  taughtProgress_=progress;
  taughtStable_=settled ? taughtStable_+1:0;
  if(taughtStable_>=3) {
    taughtMoving_=false; hostConsole.println("POSTURE reached; servo torque remains ON");
  }
}
