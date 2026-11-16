#include "host_console.h"
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "config.h"
#include "wheels.h"
#include "mux_bus.h"
#include "imu.h"

static_assert(Config::VM>0 && Config::ALIGN_V>0 && Config::ALIGN_V<=Config::LIMIT_V &&
              Config::LIMIT_V<=Config::VM/2,"Check measured VM and voltage limits");

namespace {
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
WheelRequest desired;
WheelStatus published;
WheelRequest getRequest();
bool aligning=false, pwmFailed=false;
uint32_t aligningEpoch=0;
Mode aligningMode=Mode::Off;
bool alignmentCancelled() {
  if(!aligning) return false;
  const auto r=getRequest();
  if(r.mode==aligningMode && r.epoch==aligningEpoch && millis()-r.refreshed<=Config::CONTROL_TIMEOUT_MS) return false;
  digitalWrite(Config::ENABLE,LOW); return true;
}
// Wire and all mux accesses are owned exclusively by the FOC task.
class MuxSensor final: public Sensor {
  uint8_t channel;
public:
  uint8_t failures=0;
  uint32_t errors=0;
  explicit MuxSensor(uint8_t channel):channel(channel) {}
  void init() override { Sensor::init(); }
  float getSensorAngle() override {
    uint8_t bytes[2];
    if(alignmentCancelled() || !readMuxRegister(Wire,Config::MUX_ADDRESS,channel,0x0c,bytes,2)) {
      if(failures<255) ++failures;
      ++errors;
      return -1; // Sensor retains the previous angle; repeated failures fault the group.
    }
    failures=0;
    return float((uint16_t(bytes[0]&0x0f)<<8)|bytes[1])*(_2PI/4096.0f);
  }
};
MuxSensor sensor0(Config::MUX_CHANNEL[0]),sensor1(Config::MUX_CHANNEL[1]),
          sensor2(Config::MUX_CHANNEL[2]),sensor3(Config::MUX_CHANNEL[3]);
MuxSensor* sensors[4]={&sensor0,&sensor1,&sensor2,&sensor3};
// Shared gate is NOT passed to individual drivers; group transitions own it.
BLDCDriver3PWM driver0(Config::PWM[0][0],Config::PWM[0][1],Config::PWM[0][2]);
BLDCDriver3PWM driver1(Config::PWM[1][0],Config::PWM[1][1],Config::PWM[1][2]);
BLDCDriver3PWM driver2(Config::PWM[2][0],Config::PWM[2][1],Config::PWM[2][2]);
BLDCDriver3PWM driver3(Config::PWM[3][0],Config::PWM[3][1],Config::PWM[3][2]);
BLDCDriver3PWM* drivers[4]={&driver0,&driver1,&driver2,&driver3};
BLDCMotor motor0(Config::POLE_PAIRS),motor1(Config::POLE_PAIRS),motor2(Config::POLE_PAIRS),motor3(Config::POLE_PAIRS);
BLDCMotor* motors[4]={&motor0,&motor1,&motor2,&motor3};
bool sensorOK[4]={},configured[4]={};
#pragma pack(push,1)
struct FocRecord {
  uint8_t version=1,role=3,poles=Config::POLE_PAIRS,reserved=0;
  float zero[4]={}; int8_t direction[4]={}; uint16_t pins=0;
  float vm=Config::VM; uint16_t crc=0;
};
#pragma pack(pop)
uint16_t pinSignature() {
  const int pins[]={Config::SDA,Config::SCL,Config::MUX_ADDRESS,
    Config::MUX_CHANNEL[0],Config::MUX_CHANNEL[1],Config::MUX_CHANNEL[2],Config::MUX_CHANNEL[3],Config::ENABLE,
    Config::PWM[0][0],Config::PWM[0][1],Config::PWM[0][2],Config::PWM[1][0],Config::PWM[1][1],Config::PWM[1][2],
    Config::PWM[2][0],Config::PWM[2][1],Config::PWM[2][2],Config::PWM[3][0],Config::PWM[3][1],Config::PWM[3][2]};
  return crc16(reinterpret_cast<const uint8_t*>(pins),sizeof(pins));
}
bool recordValid(const FocRecord& p) {
  if(p.version!=1 || p.role!=3 || p.poles!=Config::POLE_PAIRS || p.vm!=Config::VM || p.pins!=pinSignature() ||
     p.crc!=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(FocRecord,crc))) return false;
  for(int i=0;i<4;++i) if(!isfinite(p.zero[i]) || p.zero[i]<0 || p.zero[i]>_2PI || (p.direction[i]!=1 && p.direction[i]!=-1)) return false;
  return true;
}
bool loadFoc(FocRecord& p) {
  Preferences pref; if(!pref.begin("single-foc",true)) return false;
  bool ok=pref.getBytesLength("profile")==sizeof(p) && pref.getBytes("profile",&p,sizeof(p))==sizeof(p); pref.end();
  return ok && recordValid(p);
}
bool saveFoc() {
  FocRecord p,check; p.pins=pinSignature();
  for(int i=0;i<4;++i) { p.zero[i]=motors[i]->zero_electric_angle; p.direction[i]=int8_t(motors[i]->sensor_direction); }
  p.crc=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(FocRecord,crc));
  if(!recordValid(p)) return false;
  Preferences pref; if(!pref.begin("single-foc",false)) return false;
  bool ok=pref.putBytes("profile",&p,sizeof(p))==sizeof(p); pref.end();
  return ok && loadFoc(check) && memcmp(&p,&check,sizeof(p))==0;
}

WheelRequest getRequest() {
  portENTER_CRITICAL(&mux); WheelRequest r=desired; portEXIT_CRITICAL(&mux); return r;
}
void publish(const WheelStatus& s) {
  portENTER_CRITICAL(&mux); published=s; portEXIT_CRITICAL(&mux);
}
bool magnet(int i,uint8_t& status) {
  status=0;
  if(!readMuxRegister(Wire,Config::MUX_ADDRESS,Config::MUX_CHANNEL[i],0x0b,&status,1)) return false;
  return (status&0x20) && !(status&0x18);
}
void off() {
  digitalWrite(Config::ENABLE,LOW); // Gate off before changing any individual PWM.
  for(int i=0;i<4;++i) {
    if(configured[i]) motors[i]->disable();
  }
}
bool prepare(WheelStatus& s) {
  off();
  if(pwmFailed) { s.fault=210; return false; }
  for(int i=0;i<4;++i) {
    if(!magnet(i,s.mag[i])) { s.fault=101+i; return false; }
    if(!sensorOK[i]) {
      sensors[i]->init();
      if(sensors[i]->failures) { s.fault=111+i; return false; }
      sensorOK[i]=true;
    }
  }
  // Motor 2/4 timer setup shares a unit with 1/3; initialise ALL before enabling.
  for(int i=0;i<4;++i) if(!configured[i]) {
    auto& d=*drivers[i]; auto& m=*motors[i];
    d.voltage_power_supply=Config::VM; d.voltage_limit=Config::VM; d.pwm_frequency=25000;
    if(!d.init()) { pwmFailed=true; s.fault=201+i; return false; }
    configured[i]=true;
    m.linkDriver(&d); m.linkSensor(sensors[i]);
    m.controller=MotionControlType::velocity; m.torque_controller=TorqueControlType::voltage;
    m.voltage_limit=Config::LIMIT_V; m.voltage_sensor_align=Config::ALIGN_V; m.velocity_limit=Config::MAX_SPEED;
    m.PID_velocity.P=0.2f; m.PID_velocity.I=1; m.PID_velocity.D=0;
    m.PID_velocity.output_ramp=10; m.LPF_velocity.Tf=0.02f;
    m.init(); m.disable();
    if(m.motor_status!=FOCMotorStatus::motor_uncalibrated) { pwmFailed=true; s.fault=211+i; return false; }
  }
  off(); return !alignmentCancelled();
}
bool calibrate(int i,WheelStatus& s,const FocRecord* saved=nullptr) {
  const int id=i+1;
  hostConsole.printf("WHEEL_CAL_BEGIN wheel=%d mode=%s poles=%d align_voltage=%.2f\n",id,saved ? "RESTORE":"ALIGN",Config::POLE_PAIRS,Config::ALIGN_V);
  if(alignmentCancelled()) { s.fault=402; return false; }
  if(!magnet(i,s.mag[i])) { s.fault=101+i; return false; }
  auto& m=*motors[i];
  const uint32_t readErrorsBefore=sensors[i]->errors;
  m.sensor_direction=saved ? static_cast<Direction>(saved->direction[i]) : Direction::UNKNOWN;
  m.zero_electric_angle=saved ? saved->zero[i] : NOT_SET;
  m.enable(); digitalWrite(Config::ENABLE,HIGH);
  const int ok=m.initFOC();
  digitalWrite(Config::ENABLE,LOW); m.disable();
  const bool magnetic=magnet(i,s.mag[i]);
  if(alignmentCancelled()) { s.fault=402; return false; }
  if(!ok || (!saved && !m.pp_check_result) || !magnetic || sensors[i]->errors!=readErrorsBefore) {
    const char* reason=!ok ? "init_foc_failed":(!saved && !m.pp_check_result) ? "pole_pair_check_failed":
      !magnetic ? "encoder_postcheck_failed":"angle_read_failed";
    hostConsole.printf("WHEEL_CAL_FAIL wheel=%d reason=%s init_foc=%d pp_ok=%d mag=0x%02X encoder_ok=%d direction=%d zero=%.6f read_errors=%lu\n",
      id,reason,ok,m.pp_check_result,s.mag[i],magnetic,int(m.sensor_direction),m.zero_electric_angle,
      (unsigned long)(sensors[i]->errors-readErrorsBefore));
    s.fault=301+i; return false;
  }
  hostConsole.printf("WHEEL_CAL_OK wheel=%d\n",id);
  m.PID_velocity.reset(); return true;
}

void task(void*) {
  SimpleFOCDebug::enable(&hostConsole);
  Wire.begin(Config::SDA,Config::SCL); Wire.setClock(400000); Wire.setTimeOut(5);
  imuBegin();
  WheelStatus s;
  uint32_t lastHealth=0,lastHz=millis(),lastTick=millis(),count=0;
  int bad[4]={};
  for(;;) {
    auto r=getRequest(); uint32_t now=millis();
    if(r.mode!=Mode::Off && now-r.refreshed>Config::CONTROL_TIMEOUT_MS) {
      off(); s.state=State::Fault; s.fault=401;
    } else if(r.mode==Mode::Off) {
      if(s.state!=State::Off) off();
      s.state=State::Off; s.epoch=r.epoch; s.fault=0;
    } else if((r.mode==Mode::Calibrate || r.mode==Mode::Restore) && r.epoch!=s.epoch) {
      off(); s.epoch=r.epoch; s.fault=0; s.state=State::Calibrating; publish(s);
      aligning=true; aligningEpoch=r.epoch; aligningMode=r.mode;
      bool success=true; FocRecord saved;
      if(r.mode==Mode::Restore && !loadFoc(saved)) { success=false; s.fault=601; hostConsole.println("FOC_CACHE missing/invalid: run suspended ARM calibration first."); }
      if(r.mode==Mode::Calibrate) {
        Preferences pref;
        if(pref.begin("single-foc",false)) {
          if(pref.isKey("profile") && !pref.remove("profile")) { success=false; s.fault=602; }
          pref.end();
        } else { success=false; s.fault=602; }
      }
      if(success) success=prepare(s);
      for(int i=0;success && i<4;++i) {
        if(!calibrate(i,s,r.mode==Mode::Restore ? &saved:nullptr)) { success=false; break; }
        const auto current=getRequest();
        if(current.mode!=r.mode || current.epoch!=r.epoch ||
           millis()-current.refreshed>Config::CONTROL_TIMEOUT_MS) { success=false; s.fault=402; break; }
      }
      off();
      if(alignmentCancelled()) { success=false; s.fault=402; }
      aligning=false;
      if(!success && !s.fault) s.fault=402;
      if(success && r.mode==Mode::Calibrate && !saveFoc()) { success=false; s.fault=602; }
      if(success) hostConsole.println(r.mode==Mode::Restore ? "FOC_CACHE restored; no alignment sweep":"FOC_CACHE saved and readback verified");
      s.state=success ? State::Ready : State::Fault;
      memset(bad,0,sizeof(bad)); lastTick=millis(); publish(s);
    } else if(r.mode==Mode::Run && r.epoch==s.epoch &&
              (s.state==State::Ready || s.state==State::Running)) {
      if(s.state==State::Ready) {
        for(auto m:motors) { m->PID_velocity.reset(); m->enable(); }
        digitalWrite(Config::ENABLE,HIGH);
        s.state=State::Running; lastTick=now;
      }
      if(now-lastTick>30) { off(); s.state=State::Fault; s.fault=403; }
      else {
        for(int i=0;i<4;++i) {
          const float v=r.target[i];
          if(!isfinite(v) || fabsf(v)>Config::MAX_SPEED) { s.fault=404; break; }
          motors[i]->loopFOC(); motors[i]->move(v);
          if(sensors[i]->failures>=3) { s.fault=511+i; break; }
          if(millis()-now>30) { s.fault=403; break; }
          if(!isfinite(motors[i]->shaft_velocity) || fabsf(motors[i]->shaft_velocity)>Config::OVERSPEED) {
            s.fault=405+i; break;
          }
        }
        if(s.fault) { off(); s.state=State::Fault; }
      }
    } else if(r.mode==Mode::Run && s.state!=State::Fault) {
      off(); s.state=State::Fault; s.fault=409; // RUN without matching calibration.
    }
    lastTick=millis();
    if(now-lastHealth>=20) {
      lastHealth=now;
      for(int i=0;i<4;++i) {
        bool healthy=magnet(i,s.mag[i]);
        bad[i]=healthy ? 0 : bad[i]+1;
        if(healthy && !sensorOK[i]) { sensors[i]->init(); sensorOK[i]=!sensors[i]->failures; }
        if(s.state==State::Running && bad[i]>=3) { off(); s.state=State::Fault; s.fault=501+i; }
      }
    }
    // A previously initialised sensor may now be disconnected. Health probes
    // retry at 20ms; do not flood I2C/console with failed angle reads meanwhile.
    for(int i=0;i<4;++i) if(sensorOK[i] && bad[i]==0) {
      if(s.state!=State::Running) sensors[i]->update();
      s.angle[i]=sensors[i]->getAngle(); s.velocity[i]=sensors[i]->getVelocity();
    }
    imuTick(s.state==State::Off && r.mode==Mode::Off,
      r.imuCalibrationSafe && millis()-r.refreshed<=Config::CONTROL_TIMEOUT_MS);
    ++count;
    if(now-lastHz>=1000) { s.hz=count*1000/(now-lastHz); count=0; lastHz=now; }
    publish(s); vTaskDelay(1);
  }
}
}
void requestWheels(const WheelRequest& r) {
  portENTER_CRITICAL(&mux); desired=r; portEXIT_CRITICAL(&mux);
}
WheelStatus wheelStatus() {
  portENTER_CRITICAL(&mux); auto s=published; portEXIT_CRITICAL(&mux); return s;
}
void startWheels() {
  pinMode(Config::ENABLE,OUTPUT); digitalWrite(Config::ENABLE,LOW);
  if(xTaskCreatePinnedToCore(task,"four_wheel_foc",10240,nullptr,3,nullptr,1)!=pdPASS) {
    WheelStatus s; s.state=State::Fault; s.fault=700; publish(s);
    hostConsole.println("WHEEL_TASK allocation failed; drivers OFF");
  }
}
