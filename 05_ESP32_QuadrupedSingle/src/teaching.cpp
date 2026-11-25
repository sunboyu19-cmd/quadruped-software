#include "host_console.h"
#include "level_math.h"
#include <Preferences.h>
#include <math.h>
#include <string.h>
#include <algorithm>
#include "config.h"
#include "servos.h"

void ServoBank::loadTeaching() {
  Preferences p;
  if(!p.begin("single-teach",true)) return;
  for(int i=0;i<4;++i) {
    char key[8]; snprintf(key,sizeof(key),"joint%d",Config::SERVO_IDS[i]);
    TeachingRecord r;
    if(p.getBytesLength(key)==sizeof(r) && p.getBytes(key,&r,sizeof(r))==sizeof(r) && validTeachingStorage(r)) teaching_[i]=r;
  }
  p.end();
}
bool ServoBank::storeTeaching(int i) {
  Preferences p;
  if(!p.begin("single-teach",false)) return false;
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
bool ServoBank::beginTaughtMove(float blend,uint8_t mask,bool relative,bool automaticCentering) {
  const bool retarget=taughtMoving_;
  if(!active_ || fault_ || setupActive_ || !isfinite(blend) || (relative ? fabsf(blend)>0.04f : (blend<0 || blend>1)) || (mask!=3 && mask!=12 && mask!=15)) {
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
  taughtCentering_=automaticCentering;
  taughtReached_=releaseAfterLower_=false; automaticFollow_={};
  taughtProgress_=0; taughtStep_=float(Config::POSTURE_STEP_COUNTS)/maxDistance;
  taughtMoving_=true; taughtStart_=millis(); taughtStable_=0;
  // Retargeting must not buy an extra trajectory step. Keep the previous
  // dispatch deadline even when many BLEND commands arrive in one cycle.
  if(!retarget) taughtTick_=millis();
  hostConsole.printf("POSTURE started group=%s stand_fraction=%.3f; shared-progress sync-write, STOP holds last target\n",
    mask==3 ? "FRONT":mask==12 ? "REAR":"ALL",blend);
  return true;
}
void ServoBank::lowerThenRelease() {
  if(releaseAfterLower_) return;
  // Stop close to the taught resting position with room for position feedback error.
  if(!beginTaughtMove(0.04f,15,false,true)) {
    hostConsole.println("RELAX_LIE blocked: active healthy servos and all taught limits are required.");
    return;
  }
  releaseAfterLower_=true;
  hostConsole.println("RELAX_LIE lowering toward the taught resting position.");
}
void ServoBank::stopTaughtMove(const char* reason) {
  if(taughtMoving_) hostConsole.printf("POSTURE stopped: %s; last servo targets held\n",reason);
  if(releaseAfterLower_) hostConsole.println("RELAX_LIE cancelled; servo torque held.");
  taughtMoving_=taughtReached_=releaseAfterLower_=false; automaticFollow_={};
}
void ServoBank::tickTaughtMove() {
  const uint32_t interval=taughtCentering_ ? AutoFollow::CENTER_INTERVAL_MS:50;
  if(!taughtMoving_ || millis()-taughtTick_<interval) return;
  taughtTick_=millis();
  if(!active_ || fault_ || millis()-taughtStart_>45000) { stopTaughtMove("inactive/fault/45s timeout"); return; }
  bool settled=true; int worstLag=0,worstJoint=0;
  const int settleTolerance=taughtCentering_ && !releaseAfterLower_
    ? AutoFollow::CENTER_SETTLE : AutoFollow::POSTURE_SETTLE;
  int next[4]; bool changed=false,finalGoals=true;
  float centerScale=1.0f;
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
    const int lag=abs(status[i].position-goal[i]);
    if(lag>worstLag) { worstLag=lag; worstJoint=i; }
    if(taughtCentering_) {
      const int span=abs(int(teaching_[i].stand)-int(teaching_[i].lie));
      const int step=std::max(1,std::min(Config::POSTURE_STEP_COUNTS,int(lroundf(AutoFollow::CENTER_STEP_FRACTION*span))));
      const int remaining=taughtGoal_[i]-goal[i];
      const int delta=std::max(-step,std::min(step,remaining));
      next[i]=goal[i]+delta;
      if(delta) {
        const int room=AutoFollow::centerStepRoom(status[i].position,goal[i],delta);
        centerScale=std::min(centerScale,float(room)/abs(delta));
      }
    } else {
      next[i]=lroundf(taughtOrigin_[i]+progress*(taughtGoal_[i]-taughtOrigin_[i]));
    }
    if(!taughtTargetAllowed(i,next[i])) { fault_=true; stopTaughtMove("range failure"); return; }
    changed=changed || next[i]!=goal[i];
    finalGoals=finalGoals && goal[i]==taughtGoal_[i];
    if(next[i]!=taughtGoal_[i] || abs(status[i].position-taughtGoal_[i])>settleTolerance) settled=false;
  }
  if(taughtCentering_) {
    bool stepFits=true;
    if(changed) {
      changed=false;
      for(int i=0;i<4;++i) {
        const int delta=next[i]-goal[i];
        const int scaled=int(lroundf(delta*centerScale));
        const int room=AutoFollow::centerStepRoom(status[i].position,goal[i],delta);
        next[i]=goal[i]+std::max(-room,std::min(room,scaled));
        changed=changed || next[i]!=goal[i];
        stepFits=stepFits && AutoFollow::centerCanAdvance(abs(status[i].position-next[i]));
      }
    }
    const bool wasWaiting=automaticFollow_.waiting;
    const bool ready=settled || (changed && stepFits);
    const auto timeout=finalGoals ? AutoFollow::CENTER_SETTLE_TIMEOUT_MS : AutoFollow::TIMEOUT_MS;
    const auto action=automaticFollow_.tick(worstLag,millis(),ready,timeout);
    if(action==AutoFollow::Action::Fault) {
      hostConsole.printf("POSTURE_STALL id=%d pos=%d goal=%d lag=%d wait_ms=%lu; inspect load/power/mechanism\n",
        Config::SERVO_IDS[worstJoint],status[worstJoint].position,goal[worstJoint],worstLag,(unsigned long)(millis()-automaticFollow_.since));
      for(int i=0;i<4;++i) {
        const auto& s=status[i];
        hostConsole.printf("POSTURE_STALL_DETAIL id=%d pos=%d goal=%d load_raw=%d current_raw=%d moving=%d voltage=%.1f temp=%d error=0x%02X\n",
          Config::SERVO_IDS[i],s.position,goal[i],s.loadRaw,s.currentRaw,s.moving,s.voltage,s.temperature,unsigned(s.error));
      }
      // A small residual at the midpoint can prevent handover without
      // making a supervised return to the resting pose unsafe.
      fault_=releaseAfterLower_ || !finalGoals || worstLag>AutoFollow::WINDOW;
      stopTaughtMove("automatic target waiting timeout"); return;
    }
    if(action==AutoFollow::Action::Wait) {
      if(!wasWaiting) hostConsole.printf("POSTURE_WAIT id=%d pos=%d goal=%d lag=%d; holding all four targets until feedback catches up\n",
        Config::SERVO_IDS[worstJoint],status[worstJoint].position,goal[worstJoint],worstLag);
      taughtStable_=0; return;
    }
  }
  if(changed && !sendGoals(next,true)) { fault_=true; stopTaughtMove("sync target readback failure"); return; }
  taughtProgress_=progress;
  taughtStable_=settled ? taughtStable_+1:0;
  if(taughtStable_>=3) {
    taughtMoving_=false;
    if(releaseAfterLower_) {
      releaseAfterLower_=false;
      hostConsole.println("RELAX_LIE reached; releasing servo torque.");
      hostConsole.printf("all_servo_torque_off_verified=%d\n",release());
    } else {
      taughtReached_=true;
      hostConsole.println("POSTURE reached; servo torque remains ON");
    }
  }
}

// One small closed-loop step, always inside the existing taught interval.
LevelStep ServoBank::levelFractions(const float fractions[4]) {
  if(!active_ || fault_ || setupActive_ || taughtMoving_) return LevelStep::Failed;
  int next[4]; bool changed=false; int worstLag=0,worstJoint=0;
  for(int i=0;i<4;++i) {
    if(!isfinite(fractions[i]) || fractions[i]<LevelMath::MIN_FRACTION-0.0001f || fractions[i]>LevelMath::MAX_FRACTION+0.0001f ||
       strcmp(teachingState(i),"VALID") || !poll(i)) return LevelStep::Failed;
    const int lo=std::min(teaching_[i].stand,teaching_[i].lie),hi=std::max(teaching_[i].stand,teaching_[i].lie);
    if(status[i].position<lo-12 || status[i].position>hi+12 || abs(status[i].position-goal[i])>AutoFollow::HARD_LIMIT) {
      fault_=true; hostConsole.printf("IMULEVEL_FOLLOW id=%d pos=%d goal=%d\n",Config::SERVO_IDS[i],status[i].position,goal[i]); return LevelStep::Failed;
    }
    const int lag=abs(status[i].position-goal[i]);
    if(lag>worstLag) { worstLag=lag; worstJoint=i; }
    const int wanted=lroundf(teaching_[i].lie+fractions[i]*(int(teaching_[i].stand)-int(teaching_[i].lie)));
    next[i]=std::max(goal[i]-AutoFollow::STEP,std::min(goal[i]+AutoFollow::STEP,wanted));
    if(!taughtTargetAllowed(i,next[i])) return LevelStep::Failed;
    changed=changed || next[i]!=goal[i];
  }
  const auto action=automaticFollow_.tick(worstLag,millis());
  if(action==AutoFollow::Action::Fault) {
    fault_=true;
    hostConsole.printf("IMULEVEL_STALL id=%d pos=%d goal=%d lag=%d; inspect load/power/mechanism\n",Config::SERVO_IDS[worstJoint],status[worstJoint].position,goal[worstJoint],worstLag);
    return LevelStep::Failed;
  }
  if(action==AutoFollow::Action::Wait) return LevelStep::Waiting;
  return (!changed || sendGoals(next,false)) ? LevelStep::Applied:LevelStep::Failed;
}
