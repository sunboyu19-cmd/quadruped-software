#include "imu.h"
#include "imu_math.h"
#include "config.h"
#include "protocol.h"
#include "mux_bus.h"
#include "host_console.h"
#include <Wire.h>
#include <Preferences.h>

namespace {
portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
ImuStatus published,s;
bool pending=false,processing=false;
ImuAction action=ImuAction::Cancel;
bool ready=false,attempted=false;
int initStep=-1;
uint32_t due=0,lastPoll=0,lastSample=0,calStart=0;
ImuMath::Attitude filter;
float bias[3]={},zeroRoll=0,zeroPitch=0,sum[3]={},sumSq[3]={},sumA[3]={},firstA[3]={};
#pragma pack(push,1)
struct Profile {
  uint32_t version=1;
  int8_t axes[3]={Config::IMU_AXES[0],Config::IMU_AXES[1],Config::IMU_AXES[2]};
  uint8_t reserved=0;
  float bias[3]={},roll=0,pitch=0;
  uint16_t crc=0;
};
#pragma pack(pop)
static_assert(sizeof(Profile)==30,"IMU profile format changed");
void publish() {
  portENTER_CRITICAL(&lock); published=s;
  if(pending && action==ImuAction::Calibrate) published.calibrating=true;
  portEXIT_CRITICAL(&lock);
}
bool profileValid(const Profile& p) {
  if(p.version!=1 || p.crc!=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(Profile,crc))) return false;
  for(int i=0;i<3;++i) if(p.axes[i]!=Config::IMU_AXES[i] || !isfinite(p.bias[i]) || fabsf(p.bias[i])>10) return false;
  return isfinite(p.roll) && isfinite(p.pitch) && fabsf(p.roll)<=180 && fabsf(p.pitch)<=90;
}
bool load(Profile& p) {
  Preferences pref; if(!pref.begin("single-imu",true)) return false;
  const bool ok=pref.getBytesLength("profile")==sizeof(p) && pref.getBytes("profile",&p,sizeof(p))==sizeof(p);
  pref.end(); return ok && profileValid(p);
}
void cancel(const char* reason) {
  if(s.calibrating) hostConsole.printf("IMU_CAL aborted: %s; previous calibration retained\n",reason);
  s.calibrating=false; s.samples=0;
}
void lost(const char* reason) {
  ++s.errors; s.valid=false; cancel(reason);
  ready=false; initStep=-1; due=millis()+2000;
  filter.initialized=false; lastSample=0;
  hostConsole.printf("IMU_EVENT unavailable reason=%s; retry only with wheels OFF\n",reason);
}
bool reg(uint8_t r,uint8_t* data,uint8_t length) {
  if(!selectMux(Wire,Config::MUX_ADDRESS,Config::IMU_CHANNEL)) return false;
  Wire.beginTransmission(Config::IMU_ADDRESS); Wire.write(r);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(Config::IMU_ADDRESS,length)!=length) return false;
  for(uint8_t i=0;i<length;++i) { const int value=Wire.read(); if(value<0) return false; data[i]=uint8_t(value); }
  return true;
}
bool writeReg(uint8_t r,uint8_t value) {
  if(!selectMux(Wire,Config::MUX_ADDRESS,Config::IMU_CHANNEL)) return false;
  Wire.beginTransmission(Config::IMU_ADDRESS); Wire.write(r); Wire.write(value);
  return Wire.endTransmission()==0;
}
float mapAxis(const float v[3],int i) {
  const int axis=Config::IMU_AXES[i]; return (axis>0 ? 1.0f:-1.0f)*v[abs(axis)-1];
}
int16_t word(const uint8_t* b) { return int16_t((uint16_t(b[0])<<8)|b[1]); }
void save() {
  Profile p{},check;
  for(int i=0;i<3;++i) p.bias[i]=bias[i];
  p.roll=zeroRoll; p.pitch=zeroPitch;
  p.crc=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(Profile,crc));
  Preferences pref; bool ok=false;
  if(pref.begin("single-imu",false)) { ok=pref.putBytes("profile",&p,sizeof(p))==sizeof(p); pref.end(); }
  s.saved=ok && load(check) && memcmp(&p,&check,sizeof(p))==0;
  hostConsole.printf("IMU_SAVE verified=%d\n",s.saved);
}
void calibrateSample(const float a[3],const float g[3],bool safe,uint32_t now) {
  if(!s.calibrating) return;
  if(!safe || now-calStart>8000) { cancel("outputs enabled or sample timeout"); return; }
  const float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  if(norm<0.9f || norm>1.1f) { cancel("acceleration not stationary"); return; }
  for(int i=0;i<3;++i) {
    if(!s.samples) firstA[i]=a[i];
    if(fabsf(g[i])>10 || fabsf(a[i]-firstA[i])>0.04f) { cancel("motion detected; keep robot fixed"); return; }
  }
  for(int i=0;i<3;++i) { sum[i]+=g[i]; sumSq[i]+=g[i]*g[i]; sumA[i]+=a[i]; }
  ++s.samples;
  if(s.samples<200) return;
  for(int i=0;i<3;++i) {
    const float mean=sum[i]/200;
    if(sumSq[i]/200-mean*mean>0.25f) { cancel("gyro variance too high"); return; }
  }
  for(int i=0;i<3;++i) { bias[i]=sum[i]/200; sumA[i]/=200; }
  filter.reset(sumA[0],sumA[1],sumA[2]); zeroRoll=filter.roll(); zeroPitch=filter.pitch();
  s.calibrating=false; s.calibrated=true; s.saved=false;
  hostConsole.printf("IMU_CAL OK samples=200 bias=%.4f,%.4f,%.4f zero_roll=%.2f zero_pitch=%.2f; IMU SAVE to persist\n",
    bias[0],bias[1],bias[2],zeroRoll,zeroPitch);
}
}
void imuBegin() {
  Profile p; if(load(p)) {
    for(int i=0;i<3;++i) bias[i]=p.bias[i];
    zeroRoll=p.roll; zeroPitch=p.pitch;
    s.calibrated=s.saved=true; hostConsole.println("IMU_CAL loaded verified profile");
  }
  publish();
}
void imuTick(bool wheelsOff,bool safe) {
  const uint32_t now=millis();
  bool haveAction=false; ImuAction request=ImuAction::Cancel;
  portENTER_CRITICAL(&lock);
  if(pending) { request=action; pending=false; processing=true; haveAction=true; }
  portEXIT_CRITICAL(&lock);
  if(haveAction) {
    if(request==ImuAction::Cancel) cancel("user");
    else if(!wheelsOff || !safe) hostConsole.println("IMU action rejected: wheels and servo torque must remain OFF");
    else if(request==ImuAction::Save) {
      if(s.calibrated && !s.calibrating) save(); else hostConsole.println("IMU_SAVE rejected: complete calibration first");
    } else if(!s.valid || now-s.sampled>200) hostConsole.println("IMU_CAL rejected: fresh valid data required");
    else {
      s.calibrating=true; s.samples=0; calStart=now;
      memset(sum,0,sizeof(sum)); memset(sumSq,0,sizeof(sumSq)); memset(sumA,0,sizeof(sumA));
      hostConsole.println("IMU_CAL collecting 200 samples (~4s). Hold supported robot fixed at intended level.");
    }
    publish();
    portENTER_CRITICAL(&lock); processing=false; portEXIT_CRITICAL(&lock);
  }
  if(s.calibrating && (!safe || now-calStart>8000)) { cancel("outputs enabled or timeout"); publish(); }
  if(now-lastPoll<20) return;
  lastPoll=now;
  if(!ready) {
    if(!wheelsOff || (attempted && int32_t(now-due)<0)) return;
    if(initStep<0) {
      attempted=true; uint8_t who=0;
      if(!reg(0x75,&who,1) || who!=0x68 || !writeReg(0x6b,0x80)) { lost("WHO_AM_I/reset failed"); publish(); return; }
      initStep=0; due=now+100; return;
    }
    // Nonblocking reset/setup: one verified register per 20ms tick, only while wheels OFF.
    constexpr uint8_t regs[]={0x6b,0x6c,0x1a,0x19,0x1b,0x1c,0x38};
    constexpr uint8_t values[]={0x01,0x00,0x04,19,0x08,0x08,0x00}; // 50Hz, 21Hz LPF, +/-500dps, +/-4g.
    uint8_t actual=0;
    if(!writeReg(regs[initStep],values[initStep]) || !reg(regs[initStep],&actual,1) || actual!=values[initStep]) {
      lost("configuration readback failed"); publish(); return;
    }
    if(++initStep==int(sizeof(regs))) { ready=true; lastSample=0; hostConsole.println("IMU_EVENT MPU6050 ready channel=0 sample_hz=50"); }
    return;
  }
  uint8_t bytes[14];
  if(!reg(0x3b,bytes,14)) { lost("sample I2C failed"); publish(); return; }
  float sensorA[3],sensorG[3],a[3],g[3];
  for(int i=0;i<3;++i) { sensorA[i]=word(bytes+2*i)/8192.0f; sensorG[i]=word(bytes+8+2*i)/65.5f; }
  for(int i=0;i<3;++i) { a[i]=mapAxis(sensorA,i); g[i]=mapAxis(sensorG,i); }
  const float norm=sqrtf(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
  if(!isfinite(norm) || norm<0.1f) { lost("invalid acceleration"); publish(); return; }
  s.ax=a[0]; s.ay=a[1]; s.az=a[2]; s.gx=g[0]-bias[0]; s.gy=g[1]-bias[1]; s.gz=g[2]-bias[2];
  s.temperature=word(bytes+6)/340.0f+36.53f;
  const float dt=lastSample ? (now-lastSample)*0.001f:0;
  filter.step(a[0],a[1],a[2],s.gx,s.gy,s.gz,dt); lastSample=now;
  s.roll=ImuMath::wrap(filter.roll()-zeroRoll); s.pitch=ImuMath::wrap(filter.pitch()-zeroPitch); s.yaw=filter.yaw();
  // Euler yaw rate from corrected body Y/Z gyro and physical, un-zeroed tilt.
  const float rollRad=filter.roll()/ImuMath::DEG, pitchRad=filter.pitch()/ImuMath::DEG;
  const float pitchCos=cosf(pitchRad);
  s.yawRate=pitchCos>0.5f ? (s.gy*sinf(rollRad)+s.gz*cosf(rollRad))/pitchCos : NAN;
  s.sampled=now; s.valid=true;
  calibrateSample(a,g,safe,now);
  publish();
}
ImuStatus imuStatus() {
  portENTER_CRITICAL(&lock); auto out=published; portEXIT_CRITICAL(&lock);
  if(!out.sampled || millis()-out.sampled>200) out.valid=false;
  return out;
}
bool imuBusy() {
  portENTER_CRITICAL(&lock); const bool busy=pending || processing || published.calibrating; portEXIT_CRITICAL(&lock); return busy;
}
bool requestImuAction(ImuAction value) {
  portENTER_CRITICAL(&lock);
  if(pending || processing) { portEXIT_CRITICAL(&lock); return false; }
  action=value; pending=true;
  if(value==ImuAction::Calibrate) published.calibrating=true;
  portEXIT_CRITICAL(&lock); return true;
}
void printImu() {
  const auto out=imuStatus();
  hostConsole.printf("IMU valid=%d calibrated=%d saved=%d cal_busy=%d samples=%u sample_ms=%lu age_ms=%ld roll=%.2f pitch=%.2f yaw_rel=%.2f ax=%.4f ay=%.4f az=%.4f gx=%.3f gy=%.3f gz=%.3f temp=%.2f errors=%lu\n",
    out.valid,out.calibrated,out.saved,out.calibrating,out.samples,(unsigned long)out.sampled,out.sampled ? long(millis()-out.sampled):-1L,
    out.roll,out.pitch,out.yaw,out.ax,out.ay,out.az,out.gx,out.gy,out.gz,out.temperature,(unsigned long)out.errors);
}
