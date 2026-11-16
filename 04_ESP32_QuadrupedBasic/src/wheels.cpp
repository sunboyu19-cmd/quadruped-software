#include "host_console.h"
#include <SimpleFOC.h>
#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "config.h"
#include "wheels.h"

static_assert(Config::VM>0 && Config::ALIGN_V>0 && Config::ALIGN_V<=Config::LIMIT_V &&
              Config::LIMIT_V<=Config::VM/2,"Check measured VM and voltage limits");

namespace {
portMUX_TYPE mux=portMUX_INITIALIZER_UNLOCKED;
WheelRequest desired;
WheelStatus published;
TwoWire bus1(1);
TwoWire* buses[2]={&Wire,&bus1};
MagneticSensorI2C sensor0(AS5600_I2C), sensor1(AS5600_I2C);
MagneticSensorI2C* sensors[2]={&sensor0,&sensor1};
BLDCDriver3PWM driver0(Config::PWM[0][0],Config::PWM[0][1],Config::PWM[0][2],Config::EN[0]);
BLDCDriver3PWM driver1(Config::PWM[1][0],Config::PWM[1][1],Config::PWM[1][2],Config::EN[1]);
BLDCDriver3PWM* drivers[2]={&driver0,&driver1};
BLDCMotor motor0(Config::POLE_PAIRS), motor1(Config::POLE_PAIRS);
BLDCMotor* motors[2]={&motor0,&motor1};
bool sensorOK[2]={false,false}, configured[2]={false,false};
#pragma pack(push,1)
struct FocRecord {
  uint8_t version=1,role=ROBOT_MASTER ? 1:2,poles=Config::POLE_PAIRS,reserved=0;
  float zero[2]={}; int8_t direction[2]={}; uint16_t pins=0;
  float vm=Config::VM; uint16_t crc=0;
};
#pragma pack(pop)
uint16_t pinSignature() {
  const int pins[]={Config::SDA[0],Config::SCL[0],Config::SDA[1],Config::SCL[1],
    Config::PWM[0][0],Config::PWM[0][1],Config::PWM[0][2],Config::EN[0],
    Config::PWM[1][0],Config::PWM[1][1],Config::PWM[1][2],Config::EN[1]};
  return crc16(reinterpret_cast<const uint8_t*>(pins),sizeof(pins));
}
bool recordValid(const FocRecord& p) {
  if(p.version!=1 || p.role!=(ROBOT_MASTER ? 1:2) || p.poles!=Config::POLE_PAIRS || p.vm!=Config::VM || p.pins!=pinSignature() ||
     p.crc!=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(FocRecord,crc))) return false;
  for(int i=0;i<2;++i) if(!isfinite(p.zero[i]) || p.zero[i]<0 || p.zero[i]>_2PI || (p.direction[i]!=1 && p.direction[i]!=-1)) return false;
  return true;
}
bool loadFoc(FocRecord& p) {
  Preferences pref; if(!pref.begin("wheel-foc",true)) return false;
  bool ok=pref.getBytesLength("profile")==sizeof(p) && pref.getBytes("profile",&p,sizeof(p))==sizeof(p); pref.end();
  return ok && recordValid(p);
}
bool saveFoc() {
  FocRecord p,check; p.pins=pinSignature();
  for(int i=0;i<2;++i) { p.zero[i]=motors[i]->zero_electric_angle; p.direction[i]=int8_t(motors[i]->sensor_direction); }
  p.crc=crc16(reinterpret_cast<const uint8_t*>(&p),offsetof(FocRecord,crc));
  if(!recordValid(p)) return false;
  Preferences pref; if(!pref.begin("wheel-foc",false)) return false;
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
  auto& b=*buses[i];
  b.beginTransmission(0x36); b.write(0x0b);
  if(b.endTransmission(false)!=0 || b.requestFrom(uint8_t(0x36),uint8_t(1))!=1) return false;
  status=b.read();
  return (status&0x20) && !(status&0x18);
}
void off() {
  for(int i=0;i<2;++i) {
    if(configured[i]) motors[i]->disable();
    digitalWrite(Config::EN[i],LOW);
  }
}
bool calibrate(int i,WheelStatus& s,const FocRecord* saved=nullptr) {
  const int id=(ROBOT_MASTER ? 1:3)+i;
  hostConsole.printf("WHEEL_CAL_BEGIN wheel=%d mode=%s poles=%d align_voltage=%.2f\n",id,saved ? "RESTORE":"ALIGN",Config::POLE_PAIRS,Config::ALIGN_V);
  if(!magnet(i,s.mag[i])) {
    hostConsole.printf("WHEEL_CAL_FAIL wheel=%d reason=encoder_precheck mag=0x%02X\n",id,s.mag[i]);
    s.fault=101+i; return false;
  }
  if(!sensorOK[i]) { sensors[i]->init(buses[i]); sensorOK[i]=true; }
  if(!configured[i]) {
    auto& d=*drivers[i]; auto& m=*motors[i];
    d.voltage_power_supply=Config::VM; d.voltage_limit=Config::VM; d.pwm_frequency=25000;
    if(!d.init()) { hostConsole.printf("WHEEL_CAL_FAIL wheel=%d reason=pwm_driver_init\n",id); s.fault=201+i; return false; }
    m.linkDriver(&d); m.linkSensor(sensors[i]);
    m.controller=MotionControlType::velocity;
    m.torque_controller=TorqueControlType::voltage;
    m.voltage_limit=Config::LIMIT_V; m.voltage_sensor_align=Config::ALIGN_V;
    m.velocity_limit=Config::MAX_SPEED;
    m.PID_velocity.P=0.2f; m.PID_velocity.I=1; m.PID_velocity.D=0;
    m.PID_velocity.output_ramp=10; m.LPF_velocity.Tf=0.02f;
    configured[i]=true;
    m.init(); m.disable();
    if(m.motor_status!=FOCMotorStatus::motor_uncalibrated) { hostConsole.printf("WHEEL_CAL_FAIL wheel=%d reason=motor_init status=%d\n",id,int(m.motor_status)); s.fault=211+i; return false; }
  }
  auto& m=*motors[i];
  m.sensor_direction=saved ? static_cast<Direction>(saved->direction[i]) : Direction::UNKNOWN;
  m.zero_electric_angle=saved ? saved->zero[i] : NOT_SET;
  m.enable(); const int ok=m.initFOC(); m.disable();
  const bool magnetic=magnet(i,s.mag[i]);
  if(!ok || (!saved && !m.pp_check_result) || !magnetic) {
    const char* reason=!ok ? "init_foc_failed":(!saved && !m.pp_check_result) ? "pole_pair_check_failed":"encoder_postcheck_failed";
    hostConsole.printf("WHEEL_CAL_FAIL wheel=%d reason=%s init_foc=%d pp_ok=%d mag=0x%02X encoder_ok=%d direction=%d zero=%.6f\n",
      id,reason,ok,m.pp_check_result,s.mag[i],magnetic,int(m.sensor_direction),m.zero_electric_angle);
    s.fault=301+i; return false;
  }
  hostConsole.printf("WHEEL_CAL_OK wheel=%d\n",id);
  m.PID_velocity.reset(); return true;
}

void task(void*) {
  SimpleFOCDebug::enable(&hostConsole);
  Wire.begin(Config::SDA[0],Config::SCL[0]);
  bus1.begin(Config::SDA[1],Config::SCL[1]);
  for(auto b:buses) { b->setClock(400000); b->setTimeOut(5); }
  WheelStatus s;
  uint32_t lastHealth=0,lastHz=millis(),lastTick=millis(),count=0;
  int bad[2]={0,0};
  for(;;) {
    auto r=getRequest(); uint32_t now=millis();
    if(r.mode!=Mode::Off && now-r.refreshed>Config::LINK_TIMEOUT_MS) {
      off(); s.state=State::Fault; s.fault=401;
    } else if(r.mode==Mode::Off) {
      if(s.state!=State::Off) off();
      s.state=State::Off; s.epoch=r.epoch; s.fault=0;
    } else if((r.mode==Mode::Calibrate || r.mode==Mode::Restore) && r.epoch!=s.epoch) {
      off(); s.epoch=r.epoch; s.fault=0; s.state=State::Calibrating; publish(s);
      bool success=true; FocRecord saved;
      if(r.mode==Mode::Restore && !loadFoc(saved)) { success=false; s.fault=601; hostConsole.println("FOC_CACHE missing/invalid: run suspended ARM calibration first."); }
      if(r.mode==Mode::Calibrate) {
        Preferences pref;
        if(pref.begin("wheel-foc",false)) {
          if(pref.isKey("profile") && !pref.remove("profile")) { success=false; s.fault=602; }
          pref.end();
        } else { success=false; s.fault=602; }
      }
      for(int i=0;success && i<2;++i) {
        if(!calibrate(i,s,r.mode==Mode::Restore ? &saved:nullptr)) { success=false; break; }
        const auto current=getRequest();
        if(current.mode!=r.mode || current.epoch!=r.epoch ||
           millis()-current.refreshed>Config::LINK_TIMEOUT_MS) { success=false; s.fault=402; break; }
      }
      off();
      if(success && r.mode==Mode::Calibrate && !saveFoc()) { success=false; s.fault=602; }
      if(success) hostConsole.println(r.mode==Mode::Restore ? "FOC_CACHE restored; no alignment sweep":"FOC_CACHE saved and readback verified");
      s.state=success ? State::Ready : State::Fault;
      bad[0]=bad[1]=0; lastTick=millis(); publish(s);
    } else if(r.mode==Mode::Run && r.epoch==s.epoch &&
              (s.state==State::Ready || s.state==State::Running)) {
      if(s.state==State::Ready) {
        for(auto m:motors) { m->PID_velocity.reset(); m->enable(); }
        s.state=State::Running; lastTick=now;
      }
      if(now-lastTick>30) { off(); s.state=State::Fault; s.fault=403; }
      else {
        for(int i=0;i<2;++i) {
          const float v=r.target[i];
          if(!isfinite(v) || fabsf(v)>Config::MAX_SPEED) { s.fault=404; break; }
          motors[i]->loopFOC(); motors[i]->move(v);
          if(!isfinite(motors[i]->shaft_velocity) || fabsf(motors[i]->shaft_velocity)>Config::OVERSPEED) {
            s.fault=405+i; break;
          }
        }
        if(s.fault) { off(); s.state=State::Fault; }
      }
    } else if(r.mode==Mode::Run && s.state!=State::Fault) {
      off(); s.state=State::Fault; s.fault=407; // RUN without matching calibration.
    }
    lastTick=millis();
    if(now-lastHealth>=20) {
      lastHealth=now;
      for(int i=0;i<2;++i) {
        bool healthy=magnet(i,s.mag[i]);
        bad[i]=healthy ? 0 : bad[i]+1;
        if(healthy && !sensorOK[i]) { sensors[i]->init(buses[i]); sensorOK[i]=true; }
        if(s.state==State::Running && bad[i]>=3) { off(); s.state=State::Fault; s.fault=501+i; }
      }
    }
    // A previously initialised sensor may now be disconnected. Health probes
    // retry at 20ms; do not flood I2C/console with failed angle reads meanwhile.
    for(int i=0;i<2;++i) if(sensorOK[i] && bad[i]==0) {
      if(s.state!=State::Running) sensors[i]->update();
      s.angle[i]=sensors[i]->getAngle(); s.velocity[i]=sensors[i]->getVelocity();
    }
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
  for(int pin:Config::EN) { pinMode(pin,OUTPUT); digitalWrite(pin,LOW); }
  xTaskCreatePinnedToCore(task,"wheel_foc",8192,nullptr,3,nullptr,1);
}
