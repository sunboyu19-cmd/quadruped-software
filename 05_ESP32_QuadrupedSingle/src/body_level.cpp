#include "body_level.h"
#include "imu.h"
#include "level_math.h"
#include "host_console.h"
#include <math.h>
#include <string.h>

bool BodyLevel::preflight() {
  const auto imu=imuStatus();
  if(!imu.valid || imuBusy() || !isfinite(imu.roll) || !isfinite(imu.pitch) || fabsf(imu.roll)>20 || fabsf(imu.pitch)>20) {
    hostConsole.println("IMULEVEL blocked: fresh IMU required, no calibration in progress, tilt within +/-20deg."); return false;
  }
  if(!servos_.teachingLimits()) return false;
  for(int i=0;i<4;++i) if(strcmp(servos_.teachingState(i),"VALID")) {
    hostConsole.println("IMULEVEL blocked: all four stand/lie profiles must be valid."); return false;
  }
  return true;
}
bool BodyLevel::start(int reference,float roll,float pitch) {
  if(reference<0) reference=reference_;
  if(reference>3 || !isfinite(roll) || !isfinite(pitch) || fabsf(roll)>5 || fabsf(pitch)>5 ||
     !servos_.active() || servos_.fault() || !preflight()) return false;
  if(canRetarget(reference)) {
    rollTarget_=roll; pitchTarget_=pitch; settled_=false; stableSince_=saturatedSince_=0;
    stageStart_=millis(); bestError_=1000; state_="ADJUSTING";
    hostConsole.println("IMULEVEL target updated; reference leg unchanged."); print(); return true;
  }
  stop("new reference"); servos_.stopTaughtMove("IMU reference midpoint");
  if(!servos_.beginTaughtMove(0.5f,15,false,true)) return false;
  reference_=reference; rollTarget_=roll; pitchTarget_=pitch;
  rollCorrection_=pitchCorrection_=0; bestError_=1000;
  active_=centering_=true; settled_=driveReady_=false; lastTick_=stageStart_=millis();
  stableSince_=saturatedSince_=dynamicSince_=0; state_="CENTERING";
  hostConsole.println("IMULEVEL started: taught midpoint first, then hold reference leg and adjust other three; IMU calibration unchanged.");
  print(); return true;
}
void BodyLevel::stop(const char* reason) {
  if(active_) hostConsole.printf("IMULEVEL_STOP reason=%s; last servo targets held\n",reason);
  if(centering_) servos_.stopTaughtMove(reason);
  active_=centering_=settled_=driveReady_=false; state_="OFF";
}
bool BodyLevel::fail(const char* reason) { stop(reason); state_="FAULT"; print(); return false; }
bool BodyLevel::tick(bool calibrating,bool driving,bool driveTransition,bool hostAlive) {
  if(!active_ || settled_) return true;
  if(!hostAlive) return fail("host heartbeat expired");
  if(!servos_.active() || servos_.fault()) return fail("servo inactive/fault");
  const auto imu=imuStatus();
  if(!imu.valid || imuBusy() || !isfinite(imu.roll) || !isfinite(imu.pitch)) return fail("IMU unavailable/stale");
  if(fabsf(imu.roll)>20 || fabsf(imu.pitch)>20) return fail("tilt exceeds 20deg");
  const uint32_t now=millis();
  if(calibrating) { state_="PAUSED_DRIVE"; lastTick_=stageStart_=now; stableSince_=0; return true; }
  if(centering_) {
    if(now-stageStart_>15000) return fail("midpoint timeout");
    if(servos_.taughtMoving()) return true;
    if(!servos_.taughtReached()) return fail("midpoint following timeout");
    centering_=false; driveReady_=true; stageStart_=lastTick_=now; state_="ADJUSTING";
  }
  if(now-lastTick_<100) return true;
  const float dt=fminf((now-lastTick_)*0.001f,0.1f); lastTick_=now;
  const float norm=sqrtf(imu.ax*imu.ax+imu.ay*imu.ay+imu.az*imu.az);
  const bool unstable=norm<0.85f || norm>1.15f || fabsf(imu.gx)>15 || fabsf(imu.gy)>15 || fabsf(imu.gz)>15;
  if(driveTransition || (driving && unstable)) {
    state_="WAIT_DRIVE_STABLE";
    settled_=false; stableSince_=saturatedSince_=dynamicSince_=0;
    stageStart_=now; bestError_=1000;
    return true;
  }
  if(unstable) {
    state_="WAIT_STILL"; if(!dynamicSince_) dynamicSince_=now;
    if(now-dynamicSince_>2000) return fail("body not stationary");
    return true;
  }
  dynamicSince_=0;
  const float er=rollTarget_-imu.roll,ep=pitchTarget_-imu.pitch;
  const float error=fmaxf(fabsf(er),fabsf(ep));
  if(error<bestError_) bestError_=error;
  if(error<=0.7f) {
    if(!stableSince_) stableSince_=now;
    if(now-stableSince_>=800) { settled_=driveReady_=true; state_="HOLDING"; stageStart_=now; }
  } else {
    stableSince_=0;
    if(settled_) { stageStart_=now; bestError_=error; }
    settled_=false; state_="ADJUSTING";
    if(now-stageStart_>20000) return fail("target not reached in 20s");
    if(now-stageStart_>1000 && error>bestError_+3) return fail("tilt worsening; inspect leg/IMU directions");
  }
  // Reduce correction speed while the wheels are moving.
  const float correctionDt=driving ? dt*0.5f : dt;
  const float nextRoll=LevelMath::advance(rollCorrection_,er,correctionDt);
  const float nextPitch=LevelMath::advance(pitchCorrection_,ep,correctionDt);
  const auto target=LevelMath::mixForImu(nextRoll,nextPitch,reference_);
  const auto result=servos_.levelFractions(target.fraction);
  if(result==LevelStep::Failed) return fail("servo following/range/readback failure");
  if(result==LevelStep::Waiting) {
    state_="WAIT_SERVO"; settled_=false; stableSince_=0;
    return true; // Do not integrate attitude error while a servo catches up.
  }
  rollCorrection_=nextRoll; pitchCorrection_=nextPitch;
  if(target.saturated && error>0.7f) {
    if(!saturatedSince_) saturatedSince_=now;
    if(now-saturatedSince_>3000) return fail("correction range exhausted");
  } else saturatedSince_=0;
  return true;
}
void BodyLevel::print() const {
  hostConsole.printf("IMULEVEL active=%d state=%s reference=%d target_x=%.2f target_y=%.2f settled=%d imu_reference_unchanged=1\n",
    active_,state_,reference_+1,rollTarget_,pitchTarget_,settled_);
}
