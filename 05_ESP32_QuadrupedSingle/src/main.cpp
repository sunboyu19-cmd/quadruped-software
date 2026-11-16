#include "host_console.h"
#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "config.h"
#include "protocol.h"
#include "wheels.h"
#include "servos.h"
#include "wheel_profile.h"
#include "imu.h"

#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
#error "This project is for the one classic ESP32-WROOM-32E controller, not S3."
#endif
#ifndef ROBOT_MASTER
#error "Select single_esp32."
#endif
static_assert(ARDUINO_RUNNING_CORE==0,"Run communication on core 0; wheel FOC owns core 1.");

uint32_t bootId=0, epoch=1, lastReport=0, lastPoll=0, lastDrive=0, armStart=0;
bool calibrating=false;
Mode wanted=Mode::Off;
float targets[4]={0,0,0,0};
uint32_t lastUserContact=0,lastTeachingReport=0;

ServoBank servos;
int8_t wheelDir[4]={1,1,1,1},jointDir[4]={1,1,1,1};
uint8_t wheelVerified=0,jointVerified=0;
float offsets[4]={0,0,0,0};
bool reportEnabled=true;
int8_t savedWheelDir[4]={1,1,1,1};
uint8_t savedWheelMask=0;
bool savedWheelsPresent=false;
const char* savedWheelSource="NONE";

bool stationary() {
  for(float v:targets) if(fabsf(v)>0.001f) return false;
  return true;
}
void zeroTargets() { for(float& v:targets) v=0; lastDrive=0; }
void disarm(const char* why) {
  zeroTargets(); wanted=Mode::Off; calibrating=false;
  servos.stopTaughtMove(why);
  ++epoch;
  WheelRequest r; r.epoch=epoch; r.refreshed=millis(); requestWheels(r);
  hostConsole.printf("WHEELS DISARMED: %s; servo torque unchanged\n",why);
}
void stopDrive(const char* why) {
  servos.stopTaughtMove(why);
  zeroTargets(); hostConsole.printf("STOP: %s (zero wheel speed; servo pose held)\n",why);
}

void maintainWheels() {
  const auto s=wheelStatus();
  if(wanted!=Mode::Off && s.state==State::Fault) {
    hostConsole.printf("wheel fault single=%u\n",s.fault); disarm("wheel fault");
  }
  if(wanted==Mode::Run && (s.state==State::Off || s.epoch!=epoch))
    disarm("wheel group stopped or calibration session changed");
  if(calibrating) {
    if(millis()-armStart>60000) disarm("calibration timeout");
    else if(s.epoch==epoch && s.state==State::Ready) {
      wanted=Mode::Run; calibrating=false;
      hostConsole.println("ALL FOUR WHEELS ready; enabling at ZERO. Wait for state=3.");
    }
  }
  if(lastDrive && millis()-lastDrive>Config::DRIVE_TIMEOUT_MS)
    disarm("1500ms drive lease expired; explicit enable required");
  WheelRequest r; r.epoch=epoch; r.mode=wanted; r.refreshed=millis();
  for(int i=0;i<4;++i) r.target[i]=targets[i];
  r.imuCalibrationSafe=wanted==Mode::Off && !servos.active() && !servos.setupActive() && !servos.taughtMoving();
  requestWheels(r);
}

void printStatus() {
  const auto s=wheelStatus();
  hostConsole.printf("role=SINGLE_CONTROLLER controller=1 epoch=%lu state=%u fault=%u loop_hz=%u transport=%s\n",
    (unsigned long)epoch,unsigned(s.state),s.fault,s.hz,hostTransport());
  for(int i=0;i<4;++i) hostConsole.printf("wheel=%d mag=%02X a=%.3f w=%.3f\n",
    i+1,s.mag[i],s.angle[i],s.velocity[i]);
  hostConsole.printf("WDIR=%d,%d,%d,%d verified=%X; SDIR=%d,%d,%d,%d verified=%X\n",
    wheelDir[0],wheelDir[1],wheelDir[2],wheelDir[3],wheelVerified,
    jointDir[0],jointDir[1],jointDir[2],jointDir[3],jointVerified);
  printImu();
}

bool wheelsRunning() {
  const auto s=wheelStatus();
  return wanted==Mode::Run && s.state==State::Running && s.epoch==epoch;
}
void beginArm(bool restore=false) {
  if(imuBusy()) { hostConsole.println("BLOCKED: finish/cancel IMU action first."); return; }
  if(servos.taughtMoving()) { hostConsole.println("BLOCKED: STOP the posture transition before wheel calibration."); return; }
  if(servos.setupActive()) { hostConsole.println("BLOCKED: stop unloaded servo4 setup first."); return; }
  if(wanted!=Mode::Off) { hostConsole.println("DISARM before a new ARM."); return; }
  if(servos.active() && servos.fault()) { hostConsole.println("BLOCKED: servo fault; support body and RELAX/check first."); return; }
  if(wheelStatus().state!=State::Off) {
    hostConsole.println("Wait for controller state 0 before ARM."); return;
  }
  ++epoch; zeroTargets(); wanted=restore ? Mode::Restore:Mode::Calibrate; calibrating=true; armStart=millis();
  hostConsole.println(restore ? "RESTORING FOC calibration: no alignment sweep; enabling at zero speed only after all FOUR wheels ready.":
    "CALIBRATING FOUR WHEELS: all wheels must be lifted. Blocking inside motor alignment; use physical power switch if necessary.");
}
void drive(float forward,float turn) {
  if(!isfinite(turn) || fabsf(turn)>Config::MAX_TURN) { hostConsole.println("Turn component limited to +/-4 rad/s."); return; }
  if(wheelVerified!=0x0f) { hostConsole.println("BLOCKED: verify WDIR 1..4 first; W tests one wheel at low speed."); return; }
  if(!wheelsRunning()) { hostConsole.println("BLOCKED: ARM and wait for controller state 3."); return; }
  if(!servos.active() || servos.fault()) { hostConsole.println("BLOCKED: SERVOS ON with valid feedback required for whole-robot drive."); return; }
  const float left=forward-turn,right=forward+turn;
  if(!isfinite(left) || !isfinite(right) || fabsf(left)>Config::MAX_SPEED || fabsf(right)>Config::MAX_SPEED) {
    hostConsole.println("Rejected: each wheel must stay within +/-12 rad/s."); return;
  }
  for(int i=0;i<4;++i) targets[i]=(i%2==0 ? left:right)*wheelDir[i];
  lastDrive=millis();
}
void printWheelDirections() {
  for(int i=0;i<4;++i) {
    bool verified=wheelVerified&(1<<i),savedVerified=savedWheelMask&(1<<i);
    bool dirty=!savedWheelsPresent || wheelDir[i]!=savedWheelDir[i] || verified!=savedVerified;
    hostConsole.printf("DIRDATA id=%d value=%d verified=%d saved_value=%d saved_verified=%d saved_present=%d dirty=%d source=%s\n",
      i+1,wheelDir[i],verified,savedWheelDir[i],savedVerified,savedWheelsPresent,dirty,savedWheelSource);
  }
}
bool readSavedWheelProfile(WheelDirectionProfile& profile,const char*& source) {
  source="NONE";
  Preferences pref;
  if(!pref.begin("quad-single",true)) return false;
  const size_t length=pref.getBytesLength("wheel-profile");
  if(length) {
    bool valid=length==sizeof(profile) && pref.getBytes("wheel-profile",&profile,sizeof(profile))==sizeof(profile) && validWheelProfile(profile);
    pref.end();
    if(valid) source="PROFILE";
    else hostConsole.println("WHEEL_PROFILE invalid CRC/format; saved confirmation not trusted.");
    return valid;
  }
  // Migrate previously saved wheel directions without changing their values.
  bool valid=pref.getBytesLength("wheel-dir")==sizeof(profile.direction) &&
    pref.getBytes("wheel-dir",profile.direction,sizeof(profile.direction))==sizeof(profile.direction);
  profile.mask=pref.getUChar("wheel-mask",0);
  pref.end();
  for(int8_t sign:profile.direction) if(sign!=1 && sign!=-1) valid=false;
  if(profile.mask>15) valid=false;
  if(valid) { sealWheelProfile(profile); source="LEGACY"; }
  return valid;
}
bool loadWheelDirections() {
  WheelDirectionProfile profile; const char* source="NONE";
  if(!readSavedWheelProfile(profile,source)) {
    savedWheelsPresent=false; savedWheelSource="NONE"; savedWheelMask=0;
    hostConsole.println("WHEEL_LOAD no valid saved profile; current RAM values unchanged."); printWheelDirections(); return false;
  }
  for(int i=0;i<4;++i) wheelDir[i]=savedWheelDir[i]=profile.direction[i];
  wheelVerified=savedWheelMask=profile.mask; savedWheelsPresent=true; savedWheelSource=source;
  hostConsole.printf("WHEEL_LOAD ok=1 mask=%X source=%s; no motion commands sent\n",wheelVerified,source);
  printWheelDirections(); return true;
}
bool saveWheelDirections() {
  WheelDirectionProfile profile,check;
  for(int i=0;i<4;++i) profile.direction[i]=wheelDir[i];
  profile.mask=wheelVerified; sealWheelProfile(profile);
  Preferences pref; bool ok=false;
  if(pref.begin("quad-single",false)) {
    ok=pref.putBytes("wheel-profile",&profile,sizeof(profile))==sizeof(profile) &&
      pref.getBytes("wheel-profile",&check,sizeof(check))==sizeof(check) && validWheelProfile(check) &&
      memcmp(&profile,&check,sizeof(profile))==0;
    pref.end();
  }
  if(ok) {
    for(int i=0;i<4;++i) savedWheelDir[i]=check.direction[i];
    savedWheelMask=check.mask; savedWheelsPresent=true; savedWheelSource="PROFILE";
  }
  hostConsole.printf("WHEEL_SAVE ok=%d verified_readback=%d mask=%X all_four_confirmed=%d; A-board NVS, no servo/teaching changes\n",
    ok,ok,wheelVerified,wheelVerified==15);
  printWheelDirections(); return ok;
}
bool saveJointDirections() {
  Preferences pref; bool ok=false; int8_t check[4]={};
  if(pref.begin("quad-single",false)) {
    ok=pref.putBytes("joint-dir",jointDir,sizeof(jointDir))==sizeof(jointDir) &&
      pref.putUChar("joint-mask",jointVerified)==1 &&
      pref.getBytes("joint-dir",check,sizeof(check))==sizeof(check) &&
      memcmp(jointDir,check,sizeof(check))==0 && pref.getUChar("joint-mask",255)==jointVerified;
    pref.end();
  }
  hostConsole.printf("JOINT_DIRECTION_SAVE verified_readback=%d; wheel profile unchanged\n",ok); return ok;
}
void saveDirections() { saveWheelDirections(); saveJointDirections(); }
void loadDirections() {
  loadWheelDirections();
  Preferences pref;
  if(pref.begin("quad-single",true)) {
    if(pref.getBytesLength("joint-dir")==sizeof(jointDir)) pref.getBytes("joint-dir",jointDir,sizeof(jointDir));
    jointVerified=pref.getUChar("joint-mask",0)&15; pref.end();
  }
  for(int i=0;i<4;++i) if(jointDir[i]!=1 && jointDir[i]!=-1) { jointDir[i]=1; jointVerified&=~(1<<i); }
}
void applyTeachingDirections() {
  for(int i=0;i<4;++i) {
    const int direction=servos.teachingDirection(i);
    if(direction) { jointDir[i]=direction; jointVerified|=1<<i; }
  }
}

void help() {
  hostConsole.println("H | S | DISARM (wheels off, servo torque unchanged)");
  hostConsole.println("FEATURES TEACHING_V1");
  hostConsole.println("FEATURES FOC_CACHE_V1");
  hostConsole.println("ENABLE = restore saved FOC without alignment; ARM = suspended recalibration and save; single-controller profile; no board link");
  hostConsole.println("FEATURES WHEEL_SAVE_V1");
  hostConsole.println("FEATURES IMU_V1");
  hostConsole.println("FEATURES BLUETOOTH_SPP_V1");
  hostConsole.println("IMU READ | IMU CAL (fixed body, wheels/servo torque OFF) | IMU SAVE | IMU CANCEL; measurement only");
  hostConsole.println("DIRSHOW current/saved wheel calibration | SAVEWHEELS writes+verifies wheel-only NVS | LOADWHEELS restores saved wheels");
  hostConsole.println("LIMITS reads internal limits; TEACH STAND/LIE [id] captures fixed pose with torque OFF");
  hostConsole.println("TEACHSHOW / TEACHSAVE / TEACHCLEAR <id|ALL>; POSTURE STAND/LIE; BLEND 0..1");
  hostConsole.println("Posture transitions require UI heartbeat HB (or repeated commands) within 2s; STOP holds last target");
  hostConsole.println("ARM -> align all four wheels unloaded; W <1..4> <-1..1> tests ONE wheel");
  hostConsole.println("WDIR <id> <-1|1> / SDIR <id> <-1|1> / SAVE directions");
  hostConsole.println("MOVE <forward> <turn> wheel-rad/s; F <speed> / BACK <speed> / LEFT <speed> / RIGHT <speed>");
  hostConsole.println("STOP = zero wheels, keep servo pose; K renews 1500ms movement lease");
  hostConsole.println("SERVOS ON (wheels OFF); J <id> <-5..5> degrees; POSE <FL> <FR> <RL> <RR> offsets");
  hostConsole.println("POSE needs all SDIR confirmed; offsets +/-15, each step <=5 degrees; wheels must be stationary");
  hostConsole.println("RELAX = wheels OFF + release all servos (support body first); CHECK reads four servos");
  hostConsole.println("SERVOINFO <1..4> = read actual mode/torque/goal/offset/limits; wheels must be OFF");
  hostConsole.println("SETUPCENTER4 UNLOADED = servo4 ONLY to 2048; physically detach linkage/horn FIRST. SETUPOFF4 aborts.");
  hostConsole.println("SETUPCENTER <1..4> UNLOADED = selected bare servo to 2048; SETUPOFF aborts current setup.");
  hostConsole.println("SETMID <id> FIXED = calibrate CURRENT fixed physical pose as 2048; changes EEPROM offset, torque stays OFF");
  hostConsole.println("RESTOREMID <id> FIXED = restore original offset backed up on this A board; support/fix mechanism first");
  hostConsole.println("REPORT ON / REPORT OFF; boot NEVER moves automatically");
}
bool number(const char* p,float& value) {
  char* end=nullptr; value=strtof(p,&end);
  return end!=p && !*end && isfinite(value);
}
void command(char* line) {
  char* a[8]={}; int n=0;
  for(char* p=strtok(line," \t");p && n<8;p=strtok(nullptr," \t")) a[n++]=p;
  if(!n) return;
  lastUserContact=millis();
  for(char* p=a[0];*p;++p) *p=toupper(static_cast<unsigned char>(*p));
  const char* c=a[0];
  if(n==1 && !strcmp(c,"H")) { help(); return; }
  if(n==1 && !strcmp(c,"S")) {
    printStatus();
    if(wanted==Mode::Off) for(int i=0;i<4;++i) servos.poll(i);
    servos.print();
    return;
  }
  if(n==1 && !strcmp(c,"DISARM")) { disarm("user"); return; }
  if(n==2 && !strcmp(c,"IMU")) {
    if(!strcmp(a[1],"READ")) { printImu(); return; }
    if(!strcmp(a[1],"CANCEL")) {
      hostConsole.println(requestImuAction(ImuAction::Cancel) ? "IMU cancel queued":"IMU action pending; retry after acknowledgement"); return;
    }
    if(strcmp(a[1],"CAL") && strcmp(a[1],"SAVE")) { hostConsole.println("Use IMU READ/CAL/SAVE/CANCEL"); return; }
    if(wanted!=Mode::Off || wheelStatus().state!=State::Off || servos.active() || servos.setupActive() || servos.taughtMoving()) {
      hostConsole.println("IMU action blocked: support/fix body, RELAX, wait wheels OFF and servo torque OFF"); return;
    }
    if(imuBusy()) { hostConsole.println("IMU busy; wait or IMU CANCEL"); return; }
    if(!strcmp(a[1],"CAL") && !imuStatus().valid) { hostConsole.println("IMU_CAL blocked: fresh data required; inspect channel 0 / 0x68"); return; }
    hostConsole.println(requestImuAction(!strcmp(a[1],"CAL") ? ImuAction::Calibrate:ImuAction::Save) ? "IMU action queued":"IMU action pending"); return;
  }
  if(imuBusy() && strcmp(c,"HB") && strcmp(c,"STOP") && strcmp(c,"RELAX") && strcmp(c,"REPORT")) {
    hostConsole.println("BLOCKED: IMU action in progress; wait or IMU CANCEL"); return;
  }
  if(n==1 && !strcmp(c,"DIRSHOW")) { printWheelDirections(); return; }
  if(n==1 && (!strcmp(c,"SAVEWHEELS") || !strcmp(c,"LOADWHEELS"))) {
    if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before saving/loading wheel calibration; servo torque unchanged.");
    else if(!strcmp(c,"SAVEWHEELS")) saveWheelDirections();
    else loadWheelDirections();
    return;
  }
  if(n==1 && !strcmp(c,"HB")) {
    // Reply independently of REPORT so a quiet console is not a lost host link.
    // Does NOT renew the wheel-drive lease or enable any actuator.
    hostConsole.println("HOST_ALIVE role=SINGLE_CONTROLLER controller=1");
    return;
  }
  if(n==1 && !strcmp(c,"TEACHSHOW")) { servos.printTeaching(); return; }
  if(n==1 && !strcmp(c,"LIMITS")) {
    if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before LIMITS.");
    else { servos.teachingLimits(); applyTeachingDirections(); }
    return;
  }
  if((n==2 || n==3) && !strcmp(c,"TEACH") && (!strcmp(a[1],"STAND") || !strcmp(a[1],"LIE"))) {
    float id=0; int index=-1;
    if(n==3) {
      if(!number(a[2],id) || id<1 || id>4 || floorf(id)!=id) { hostConsole.println("Invalid TEACH ID."); return; }
      index=int(id)-1;
    }
    if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before teaching; support body and RELAX servos.");
    else { servos.captureTeaching(!strcmp(a[1],"STAND"),index); applyTeachingDirections(); }
    return;
  }
  if(n==1 && !strcmp(c,"TEACHSAVE")) {
    if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before saving teaching.");
    else { applyTeachingDirections(); if(servos.saveTeaching()) saveJointDirections(); }
    return;
  }
  if(n==2 && !strcmp(c,"TEACHCLEAR")) {
    if(wanted!=Mode::Off || servos.active() || servos.setupActive()) { hostConsole.println("Support body and RELAX before clearing teaching."); return; }
    float id=0; int index=-1;
    if(strcmp(a[1],"ALL")) {
      if(!number(a[1],id) || id<1 || id>4 || floorf(id)!=id) { hostConsole.println("Use TEACHCLEAR <1..4|ALL>."); return; }
      index=int(id)-1;
    }
    if(servos.clearTeaching(index)) {
      if(index<0) jointVerified=0; else jointVerified&=~(1<<index);
      hostConsole.println("TEACH records cleared in RAM/NVS; servo EEPROM unchanged.");
    }
    servos.printTeaching(); return;
  }
  if(n==1 && !strcmp(c,"POSTURESTOP")) { servos.stopTaughtMove("posture key released"); return; }
  if(n==3 && !strcmp(c,"SHIFT")) {
    float delta=0; uint8_t mask=0;
    if(!strcmp(a[1],"ALL")) mask=15;
    else if(!strcmp(a[1],"FRONT")) mask=3;
    else if(!strcmp(a[1],"REAR")) mask=12;
    if(!mask || !number(a[2],delta) || fabsf(delta)>0.02f) { hostConsole.println("Use SHIFT ALL/FRONT/REAR <-0.02..0.02>."); return; }
    if(calibrating) { hostConsole.println("Wait for calibration before SHIFT."); return; }
    if(!servos.beginTaughtMove(delta,mask,true)) servos.print();
    return;
  }
  if((n==2 && !strcmp(c,"POSTURE")) || ((n==2 || n==3) && !strcmp(c,"BLEND"))) {
    float blend=0; uint8_t mask=15;
    if(n==3) {
      if(!strcmp(a[1],"FRONT")) mask=3;
      else if(!strcmp(a[1],"REAR")) mask=12;
      else if(strcmp(a[1],"ALL")) { hostConsole.println("Use BLEND FRONT/REAR/ALL <0..1>."); return; }
    }
    if(!strcmp(c,"POSTURE")) {
      if(!strcmp(a[1],"STAND")) blend=1;
      else if(!strcmp(a[1],"LIE")) blend=0;
      else { hostConsole.println("Use POSTURE STAND/LIE."); return; }
    } else if(!number(a[n-1],blend) || blend<0 || blend>1) { hostConsole.println("BLEND must be 0..1."); return; }
    if(calibrating) hostConsole.println("Wait for wheel calibration before taught posture motion.");
    else {
      if(!servos.beginTaughtMove(blend,mask)) {
        hostConsole.println("POSTURE rejected: see preceding POSTURE blocked reason; existing fault is not cleared by BLEND.");
        servos.print();
      }
    }
    return;
  }
  if(n==3 && (!strcmp(c,"SETMID") || !strcmp(c,"RESTOREMID")) && !strcmp(a[2],"FIXED")) {
    float id=0;
    if(!number(a[1],id) || id<1 || id>4 || floorf(id)!=id) hostConsole.println("Use SETMID/RESTOREMID <1..4> FIXED.");
    else if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before midpoint calibration.");
    else { servos.midpointInPlace(int(id)-1,!strcmp(c,"RESTOREMID")); servos.printTeaching(); }
    return;
  }
  if(n==1 && !strcmp(c,"SETUPOFF")) {
    if(wanted!=Mode::Off || (servos.active() && !servos.setupActive()))
      hostConsole.println("BLOCKED: normal control active; support body and use normal RELAX first.");
    else servos.stopSetup();
    return;
  }
  if(n==3 && !strcmp(c,"SETUPCENTER") && !strcmp(a[2],"UNLOADED")) {
    float id=0;
    if(!number(a[1],id) || id<1 || id>4 || floorf(id)!=id) hostConsole.println("Use SETUPCENTER <1..4> UNLOADED.");
    else if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before unloaded setup.");
    else if(!servos.setupCenter(int(id)-1)) hostConsole.println("SETUP failed: check torque OFF, mode 0, limits, voltage and position stability.");
    return;
  }
  if(n==1 && !strcmp(c,"SETUPOFF4")) {
    if(wanted!=Mode::Off || (servos.active() && !servos.setupActive()))
      hostConsole.println("BLOCKED: normal wheel/servo control active; support body and use normal RELAX first.");
    else servos.stopSetup4();
    return;
  }
  if(n==2 && !strcmp(c,"SETUPCENTER4") && !strcmp(a[1],"UNLOADED")) {
    if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before unloaded setup.");
    else if(!servos.setupCenter4()) hostConsole.println("SETUP4 failed: check torque OFF, mode 0, limits, voltage and position stability.");
    return;
  }
  if(n==1 && !strcmp(c,"ARM")) { beginArm(); return; }
  if(n==1 && !strcmp(c,"ENABLE")) { beginArm(true); return; }
  if(n==1 && !strcmp(c,"STOP")) { stopDrive("user"); return; }
  if(n==1 && !strcmp(c,"K")) { if(lastDrive) lastDrive=millis(); return; }
  if(n==1 && !strcmp(c,"SAVE")) {
    if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before SAVE (flash write).");
    else saveDirections();
    return;
  }
  if(n==1 && !strcmp(c,"RELAX")) {
    disarm("RELAX"); hostConsole.printf("all_servo_torque_off_verified=%d\n",servos.release()); return;
  }
  if(n==1 && !strcmp(c,"CHECK")) {
    if(wanted!=Mode::Off) { hostConsole.println("DISARM wheels before CHECK."); return; }
    for(int i=0;i<4;++i) servos.poll(i);
    servos.print(); return;
  }
  if(n==2 && !strcmp(c,"REPORT")) {
    if(!strcmp(a[1],"ON")) reportEnabled=true;
    else if(!strcmp(a[1],"OFF")) reportEnabled=false;
    else hostConsole.println("Use REPORT ON/OFF.");
    return;
  }
  if(n==2 && !strcmp(c,"SERVOS") && !strcmp(a[1],"ON")) {
    if(wanted!=Mode::Off) { hostConsole.println("DISARM wheels before SERVOS ON; support the robot."); return; }
    if(servos.active()) {
      hostConsole.println(servos.fault() ? "SERVOS active but fault latched; inspect S and preceding diagnostics; do not re-enable blindly.":"SERVOS already enabled and healthy; ready for joint/posture commands; centres unchanged.");
      servos.print(); return;
    }
    if(servos.arm()) { applyTeachingDirections(); for(float& x:offsets) x=0; hostConsole.println("SERVOS ON at current positions; taught limits active where complete; default +/-15deg elsewhere."); }
    else hostConsole.println("SERVOS ON failed: CHECK mode 0, voltage, IDs 1..4 and position limits.");
    return;
  }
  float v[4]={};
  if(n==2 && number(a[1],v[0])) {
    if(!strcmp(c,"SERVOINFO") && v[0]>=1 && v[0]<=4 && floorf(v[0])==v[0]) {
      if(wanted!=Mode::Off) hostConsole.println("DISARM wheels before SERVOINFO.");
      else servos.inspect(int(v[0])-1);
      return;
    }
    if(v[0]<0 || v[0]>Config::MAX_SPEED) { hostConsole.println("Speed must be 0..12."); return; }
    if(!strcmp(c,"F")) { drive(v[0],0); return; }
    if(!strcmp(c,"BACK")) { drive(-v[0],0); return; }
    if(!strcmp(c,"LEFT")) { drive(0,v[0]); return; }
    if(!strcmp(c,"RIGHT")) { drive(0,-v[0]); return; }
  }
  if(n==3 && number(a[1],v[0]) && number(a[2],v[1])) {
    if(!strcmp(c,"MOVE")) { drive(v[0],v[1]); return; }
    if(v[0]>=1 && v[0]<=4 && floorf(v[0])==v[0]) {
      const int id=int(v[0]);
      const int i=id-1;
      if(!strcmp(c,"WDIR") || !strcmp(c,"SDIR")) {
        const bool wheel=!strcmp(c,"WDIR");
        if(wanted!=Mode::Off || (!wheel && servos.active())) { hostConsole.println("WDIR requires wheels OFF; SDIR requires wheels and servo torque OFF."); return; }
        if(v[1]!=1 && v[1]!=-1) { hostConsole.println("Direction must be +1 or -1."); return; }
        if(wheel) { wheelDir[i]=v[1]; wheelVerified|=1<<i; }
        else { jointDir[i]=v[1]; jointVerified|=1<<i; }
        hostConsole.println("Direction set in RAM; SAVEWHEELS saves wheels, SAVE saves both direction sets.");
        if(wheel) printWheelDirections();
        return;
      }
      if(!strcmp(c,"W")) {
        if(servos.taughtMoving()) { hostConsole.println("STOP posture motion before W."); return; }
        if(!wheelsRunning() || fabsf(v[1])>Config::TEST_SPEED) { hostConsole.println("ARM first; W speed limited to +/-1 rad/s."); return; }
        zeroTargets(); targets[i]=v[1]*wheelDir[i]; lastDrive=millis(); return;
      }
      if(!strcmp(c,"J")) {
        if(calibrating || servos.taughtMoving()) { hostConsole.println("Wait for calibration/posture transition before joint movement."); return; }
        if(servos.jog(i,v[1],jointDir[i])) offsets[i]+=v[1];
        else hostConsole.println("J rejected: SERVOS ON, +/-5deg step, +/-15deg range, limits/feedback.");
        return;
      }
    }
  }
  if(n==5 && !strcmp(c,"POSE")) {
    for(int i=0;i<4;++i) if(!number(a[i+1],v[i])) { hostConsole.println("Invalid POSE number."); return; }
    if(calibrating || servos.taughtMoving() || jointVerified!=15) { hostConsole.println("Wait for calibration/posture transition and confirm all SDIR before POSE."); return; }
    if(servos.pose(v,jointDir)) for(int i=0;i<4;++i) offsets[i]=v[i];
    else hostConsole.println("POSE rejected: +/-15deg range, <=5deg change per joint, valid feedback/limits required.");
    return;
  }
  hostConsole.println("Unknown/invalid command; H for help. One command per line.");
}

void serialCommands() {
  static char buffer[128]; static size_t used=0; static bool overflow=false;
  static uint32_t sourceGeneration=0;
  if(sourceGeneration!=hostGeneration()) { used=0; overflow=false; sourceGeneration=hostGeneration(); }
  for(int budget=0;budget<128 && hostAvailable();++budget) {
    char c=hostRead();
    if(c=='\r' || c=='\n') {
      if(!overflow && used) { buffer[used]=0; command(buffer); }
      if(overflow) hostConsole.println("Command too long; discarded.");
      used=0; overflow=false;
    } else if(!overflow) {
      if(used<sizeof(buffer)-1) buffer[used++]=c;
      else { overflow=true; used=0; }
    }
  }
}

void setup() {
  pinMode(Config::ENABLE,OUTPUT); digitalWrite(Config::ENABLE,LOW);
  // Configure before begin: UART0 printing must not wait for each byte on wire.
  Serial.setTxBufferSize(4096);
  Serial.begin(115200); bootId=esp_random(); if(!bootId) bootId=1;
  beginHostNetwork();
  startWheels();
  loadDirections(); servos.begin();
  hostConsole.printf("Quadruped SINGLE classic ESP32; role=SINGLE_CONTROLLER; boot=%08lX; VM=%.1f; motor outputs OFF\n",
    (unsigned long)bootId,Config::VM);
  hostConsole.printf("TASK_CORES communication=%d wheels=1; build=single-tca9548a-v1\n",xPortGetCoreID());
  hostConsole.printf("WHEEL_OUTPUT voltage_limit=%.2f align_voltage=%.2f current_sense=0; STOP holds zero speed, DISARM disables drivers\n",
    Config::LIMIT_V,Config::ALIGN_V);
  help();
}
void loop() {
  if(pollHostNetwork()) {
    disarm("host transport changed; explicit enable required");
    help(); printStatus(); // Fresh capabilities after discarding old-source input.
  }
  maintainWheels(); serialCommands();
  servos.tickSetup();
  if(servos.taughtMoving() && millis()-lastUserContact>2000) stopDrive("posture host heartbeat expired");
  const bool wasMoving=servos.taughtMoving();
  servos.tickTaughtMove();
  if((wasMoving && !servos.taughtMoving()) || (servos.taughtMoving() && millis()-lastTeachingReport>=1000)) {
    lastTeachingReport=millis(); servos.printTeaching();
  }
  if(millis()-lastPoll>=100 && !calibrating) {
    static int index=0; lastPoll=millis();
    // Read only while active; CHECK performs explicit preflight while off.
    if(servos.active()) {
      const bool healthy=servos.poll(index);
      if((!healthy || servos.fault()) && wanted!=Mode::Off) disarm("servo feedback fault; pose held");
    }
    index=(index+1)%4;
  }
  if(reportEnabled && millis()-lastReport>=1000) { lastReport=millis(); printStatus(); }
  // Refresh after any serial operation; FOC runs on its own core without UART blocking.
  maintainWheels(); delay(1);
}
