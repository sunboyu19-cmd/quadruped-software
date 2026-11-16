#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <atomic>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Use the classic ESP32-WROOM controller, not S3/C3."
#endif
static_assert(ARDUINO_RUNNING_CORE==0,"Serial commands use core0; FOC uses core1");
namespace {
enum class Mode:uint8_t { Off,Calibrate,Run };
struct Request { Mode mode=Mode::Off; int wheel=0; uint32_t generation=1,deadline=0,clock=100000; float target=0; uint32_t check=0; };
struct Status {
  Mode mode=Mode::Off; int wheel=0; uint32_t generation=0; bool calibrated=false,magOK=false;
  uint8_t mag=0; float angle=0,encoderVelocity=0,velocity=0,uq=0,target=0;
  uint32_t errors=0,remaining=0,hz=0; int fault=0; bool enabled=false;
};
portMUX_TYPE lock=portMUX_INITIALIZER_UNLOCKED;
Request desired;
Status published;
std::atomic<uint32_t> mainAlive{0};
bool report=false;
const char* const names[4]={"FL(left-front)","FR(right-front)","RL(left-rear)","RR(right-rear)"};
Request request() { portENTER_CRITICAL(&lock); auto r=desired; portEXIT_CRITICAL(&lock); return r; }
void submit(const Request& r) { portENTER_CRITICAL(&lock); desired=r; portEXIT_CRITICAL(&lock); }
Status status() { portENTER_CRITICAL(&lock); auto s=published; portEXIT_CRITICAL(&lock); return s; }
void publish(const Status& s) { portENTER_CRITICAL(&lock); published=s; portEXIT_CRITICAL(&lock); }
bool enableFor(Mode mode,uint32_t generation) {
  portENTER_CRITICAL(&lock);
  const bool allowed=desired.mode==mode && desired.generation==generation &&
    millis()-mainAlive.load()<=Config::MAIN_TIMEOUT_MS;
  if(allowed) digitalWrite(Config::ENABLE,HIGH);
  portEXIT_CRITICAL(&lock); return allowed;
}
bool aligning=false;
bool magComm[4]={};
uint32_t alignmentGeneration=0;
bool cancelled() {
  if(!aligning) return false;
  const auto r=request();
  if(r.mode==Mode::Calibrate && r.generation==alignmentGeneration && millis()-mainAlive.load()<=Config::MAIN_TIMEOUT_MS) return false;
  digitalWrite(Config::ENABLE,LOW); return true;
}
bool reg(int i,uint8_t address,uint8_t* data,uint8_t length) {
  const uint8_t mask=uint8_t(1u<<Config::CHANNEL[i]);
  Wire.beginTransmission(Config::MUX); Wire.write(mask);
  if(Wire.endTransmission()!=0 || Wire.requestFrom(Config::MUX,uint8_t(1))!=1 || Wire.read()!=mask) return false;
  Wire.beginTransmission(0x36); Wire.write(address);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(uint8_t(0x36),length)!=length) return false;
  for(uint8_t j=0;j<length;++j) { const int value=Wire.read(); if(value<0) return false; data[j]=uint8_t(value); }
  return true;
}
bool magnet(int i,uint8_t& bits) {
  bits=0; magComm[i]=reg(i,0x0b,&bits,1);
  return magComm[i] && (bits&0x20) && !(bits&0x18);
}
class MuxSensor final:public Sensor {
  int wheel;
public:
  uint32_t errors=0; uint8_t failures=0;
  explicit MuxSensor(int wheel):wheel(wheel) {}
  void init() override { Sensor::init(); }
  float getSensorAngle() override {
    if(cancelled()) return -1;
    uint8_t data[2];
    if(!reg(wheel,0x0c,data,2)) { ++errors; if(failures<255) ++failures; return -1; }
    failures=0; return float(((data[0]&15)<<8)|data[1])*(_2PI/4096.0f);
  }
};
MuxSensor sensor0(0),sensor1(1),sensor2(2),sensor3(3);
MuxSensor* sensors[4]={&sensor0,&sensor1,&sensor2,&sensor3};
BLDCDriver3PWM driver0(Config::PWM[0][0],Config::PWM[0][1],Config::PWM[0][2]);
BLDCDriver3PWM driver1(Config::PWM[1][0],Config::PWM[1][1],Config::PWM[1][2]);
BLDCDriver3PWM driver2(Config::PWM[2][0],Config::PWM[2][1],Config::PWM[2][2]);
BLDCDriver3PWM driver3(Config::PWM[3][0],Config::PWM[3][1],Config::PWM[3][2]);
BLDCDriver3PWM* drivers[4]={&driver0,&driver1,&driver2,&driver3};
BLDCMotor motor0(Config::POLES),motor1(Config::POLES),motor2(Config::POLES),motor3(Config::POLES);
BLDCMotor* motors[4]={&motor0,&motor1,&motor2,&motor3};
bool pwmReady[4]={},motorReady[4]={},sensorReady[4]={},calibrated[4]={},pwmFailed=false;
void off() {
  digitalWrite(Config::ENABLE,LOW);
  for(int i=0;i<4;++i) {
    if(motorReady[i]) motors[i]->disable();
    else if(pwmReady[i]) drivers[i]->disable();
  }
}
bool prepareDrivers() {
  off(); if(pwmFailed) return false;
  for(int i=0;i<4;++i) if(!pwmReady[i]) {
    auto& d=*drivers[i]; d.voltage_power_supply=Config::VM; d.voltage_limit=Config::VM; d.pwm_frequency=25000;
    if(!d.init()) { pwmFailed=true; return false; }
    pwmReady[i]=true;
  }
  // Four slots share two MCPWM units. Finish ALL timer setup with EN LOW.
  off(); return true;
}
bool initSensor(int i) {
  if(sensorReady[i]) return true;
  const auto errors=sensors[i]->errors; sensors[i]->init();
  sensorReady[i]=errors==sensors[i]->errors && !sensors[i]->failures;
  return sensorReady[i];
}
bool calibrate(int i,Status& s) {
  calibrated[i]=false;
  if(!magnet(i,s.mag)) {
    Serial.printf("CAL_FAIL wheel=%d ch=%u status_read=%d mag=0x%02X reason=encoder_or_magnet_precheck\n",i+1,Config::CHANNEL[i],magComm[i],s.mag);
    s.fault=101+i; return false;
  }
  if(!initSensor(i)) { s.fault=111+i; return false; }
  if(!prepareDrivers()) { s.fault=201; return false; }
  auto& m=*motors[i];
  if(!motorReady[i]) {
    m.linkDriver(drivers[i]); m.linkSensor(sensors[i]);
    m.controller=MotionControlType::velocity; m.torque_controller=TorqueControlType::voltage;
    m.voltage_limit=Config::LIMIT_V; m.voltage_sensor_align=Config::ALIGN_V; m.velocity_limit=Config::MAX_SPEED;
    m.PID_velocity.P=0.2f; m.PID_velocity.I=1; m.PID_velocity.D=0;
    m.PID_velocity.output_ramp=10; m.LPF_velocity.Tf=0.02f;
    motorReady[i]=true; m.init(); m.disable();
    if(m.motor_status!=FOCMotorStatus::motor_uncalibrated) { motorReady[i]=false; s.fault=211+i; return false; }
  }
  if(cancelled()) { s.fault=402; return false; }
  m.sensor_direction=Direction::UNKNOWN; m.zero_electric_angle=NOT_SET;
  const uint32_t errors=sensors[i]->errors;
  m.enable();
  if(!enableFor(Mode::Calibrate,alignmentGeneration)) { off(); s.fault=402; return false; }
  const int ok=m.initFOC();
  off();
  if(cancelled()) { s.fault=402; return false; }
  const bool field=magnet(i,s.mag);
  if(!ok || !m.pp_check_result || !field || errors!=sensors[i]->errors) {
    Serial.printf("CAL_FAIL wheel=%d init_foc=%d pp_ok=%d mag=0x%02X angle_read_errors=%lu\n",i+1,ok,m.pp_check_result,s.mag,(unsigned long)(sensors[i]->errors-errors));
    s.fault=301+i; return false;
  }
  m.PID_velocity.reset(); calibrated[i]=true; s.fault=0;
  Serial.printf("CAL_OK wheel=%d poles=7 direction=%d zero=%.6f; outputs OFF. Use RUN 0.5 3\n",i+1,int(m.sensor_direction),m.zero_electric_angle);
  return true;
}
void task(void*) {
  Wire.begin(Config::SDA,Config::SCL); Wire.setTimeOut(5); Wire.setClock(100000);
  SimpleFOCDebug::enable(&Serial);
  Status s; uint32_t done=0,seenCheck=0,clock=100000,lastHealth=0,lastLoop=millis(),lastHz=millis(),count=0,stallSince=0;
  int bad=0;
  for(;;) {
    const Request r=request(); const uint32_t now=millis(); const int i=r.wheel;
    if(r.clock!=clock && s.mode==Mode::Off) {
      Wire.setClock(r.clock); clock=r.clock;
      memset(calibrated,0,sizeof(calibrated));
      Serial.printf("I2C speed=%lu; calibrations invalidated\n",(unsigned long)clock);
    }
    if(r.check!=seenCheck && s.mode==Mode::Off && r.mode==Mode::Off) {
      seenCheck=r.check;
      for(int j=0;j<4;++j) {
        uint8_t mag=0,data[2]={}; const bool field=magnet(j,mag);
        const bool angle=reg(j,0x0c,data,2);
        Serial.printf("ENC wheel=%d %s ch=%u status_read=%d angle_read=%d magnet_ok=%d mag=0x%02X raw=%d\n",j+1,names[j],Config::CHANNEL[j],magComm[j],angle,field,mag,angle ? ((data[0]&15)<<8)|data[1]:-1);
      }
    }
    if(r.mode==Mode::Off) {
      if(s.mode!=Mode::Off || s.wheel!=i) off();
      s.mode=Mode::Off; s.wheel=i; s.generation=r.generation; s.target=s.uq=s.velocity=0; s.remaining=0;
      stallSince=0;
      if(r.generation!=done) { s.fault=0; done=r.generation; }
    } else if(r.generation!=done) {
      if(r.mode==Mode::Calibrate) {
        off(); s.mode=Mode::Calibrate; s.wheel=i; s.generation=r.generation; s.fault=0; publish(s);
        aligning=true; alignmentGeneration=r.generation;
        Serial.printf("CAL_BEGIN wheel=%d ch=%u VM=%.2f align=%.2f limit=%.2f; ALL wheels must be lifted\n",i+1,Config::CHANNEL[i],Config::VM,Config::ALIGN_V,Config::LIMIT_V);
        const bool ok=calibrate(i,s); off(); aligning=false;
        done=r.generation; s.mode=Mode::Off; s.target=s.uq=0;
        if(!ok) Serial.printf("DISABLED calibration failed/cancelled fault=%d\n",s.fault);
        bad=0; lastLoop=millis();
      } else if(r.mode==Mode::Run) {
        if(!calibrated[i] || !magnet(i,s.mag) || !validRun(r.target,1) || expired(now,r.deadline)) {
          off(); s.mode=Mode::Off; s.fault=601; done=r.generation;
          Serial.println("RUN_BLOCKED: valid calibration/magnet/target/deadline required");
        } else {
          if(s.mode!=Mode::Run || s.wheel!=i) { off(); motors[i]->PID_velocity.reset(); motors[i]->enable(); }
          if(!enableFor(Mode::Run,r.generation)) {
            off(); s.mode=Mode::Off; s.fault=402; done=r.generation; publish(s); continue;
          }
          s.mode=Mode::Run; s.wheel=i; s.generation=r.generation; s.fault=0; s.target=r.target; done=r.generation; bad=0; stallSince=0; lastLoop=now;
          Serial.printf("RUN_BEGIN wheel=%d target=%.3f remaining_ms=%lu\n",i+1,r.target,(unsigned long)(r.deadline-now));
        }
      }
    }
    if(s.mode==Mode::Run) {
      int fault=0;
      if(millis()-mainAlive.load()>Config::MAIN_TIMEOUT_MS) fault=401;
      else if(now-lastLoop>30) fault=403;
      else if(expired(now,r.deadline)) {
        off(); s.mode=Mode::Off; s.target=s.uq=s.velocity=0; s.remaining=0;
        Serial.printf("RUN_DONE wheel=%d timed test complete; outputs OFF\n",i+1);
      } else {
        motors[i]->loopFOC(); motors[i]->move(s.target);
        s.velocity=motors[i]->shaft_velocity; s.uq=motors[i]->voltage.q;
        s.remaining=r.deadline-now;
        if(millis()-now>30) fault=403;
        else if(sensors[i]->failures>=3) fault=511+i;
        else if(!isfinite(s.velocity) || fabsf(s.velocity)>Config::OVERSPEED) fault=405;
        if(fabsf(s.target)>=0.5f && fabsf(s.velocity)<0.1f && fabsf(s.uq)>=Config::LIMIT_V*0.9f) {
          if(!stallSince) stallSince=now;
          if(now-stallSince>2000) fault=520;
        } else stallSince=0;
      }
      if(fault) { off(); s.mode=Mode::Off; s.target=s.uq=s.velocity=0; s.remaining=0; s.fault=fault; Serial.printf("WHEEL_FAULT wheel=%d fault=%d; outputs OFF\n",i+1,fault); }
    }
    if(now-lastHealth>=20) {
      lastHealth=now; s.magOK=magnet(i,s.mag);
      bad=s.magOK ? 0:bad+1;
      if(s.mode==Mode::Run && bad>=3) {
        off(); s.mode=Mode::Off; s.fault=501+i; s.target=s.uq=s.velocity=0; s.remaining=0;
        Serial.printf("WHEEL_FAULT wheel=%d reason=magnet_or_I2C fault=%d\n",i+1,s.fault);
      }
    }
    if(s.magOK && initSensor(i)) {
      if(s.mode!=Mode::Run) sensors[i]->update();
      s.angle=sensors[i]->getAngle(); s.encoderVelocity=sensors[i]->getVelocity();
    }
    if(s.mode==Mode::Run && millis()-now>30) {
      off(); s.mode=Mode::Off; s.fault=403; s.target=s.uq=s.velocity=0; s.remaining=0;
      Serial.printf("WHEEL_FAULT wheel=%d reason=I2C_or_loop_timeout fault=403\n",i+1);
    }
    s.calibrated=calibrated[i]; s.errors=sensors[i]->errors; s.enabled=digitalRead(Config::ENABLE)==HIGH;
    ++count; if(now-lastHz>=1000) { s.hz=count*1000/(now-lastHz); count=0; lastHz=now; }
    publish(s); lastLoop=now; vTaskDelay(1);
  }
}
void printStatus() {
  const auto s=status(); const auto r=request();
  Serial.printf("wheel=%d %s ch=%u mode=%s en=%d calibrated=%d mag=0x%02X magnet_ok=%d a=%.3f encoder_w=%.3f foc_w=%.3f target=%.3f uq=%.3f remaining_ms=%lu read_errors=%lu fault=%d loop_hz=%lu i2c=%lu\n",
    s.wheel+1,names[s.wheel],Config::CHANNEL[s.wheel],s.mode==Mode::Run ? "run":s.mode==Mode::Calibrate ? "calibrating":"off",s.enabled,s.calibrated,s.mag,s.magOK,s.angle,s.encoderVelocity,s.velocity,s.target,s.uq,(unsigned long)s.remaining,(unsigned long)s.errors,s.fault,(unsigned long)s.hz,(unsigned long)r.clock);
}
void help() {
  Serial.println("I <1..4> select (outputs OFF) | CHECK read 4 encoders | S status | REPORT ON/OFF");
  Serial.println("CAL / A: align SELECTED wheel, then outputs OFF; all wheels unloaded");
  Serial.println("RUN <rad/s> <seconds> (0.1..10s); V0.5 / V-0.5 = 3s test; V0 stops");
  Serial.println("D / STOP / DISARM: common EN LOW; SPEED 100000 / SPEED 400000 (OFF only)");
  Serial.println("VM=6V,7 pole pairs,align=1.5V,output limit=2.5V,max target=12rad/s; no current sense");
  Serial.println("No servo UART, IMU, WiFi, Bluetooth or NVS writes. CAL/RUN are the ONLY enabling commands.");
}
bool number(const char* text,float& value) { char* end=nullptr; value=strtof(text,&end); return end!=text && !*end && isfinite(value); }
void stop() {
  auto r=request(); r.mode=Mode::Off; r.target=0; ++r.generation; submit(r);
  digitalWrite(Config::ENABLE,LOW); // Safe-only cross-core action, independent of I2C/alignment.
  Serial.println("STOP requested: outputs OFF");
}
void command(char* text) {
  for(char* p=text;*p;++p) if(*p>='a' && *p<='z') *p-=32;
  if(!strcmp(text,"H")) { help(); return; }
  if(!strcmp(text,"S")) { printStatus(); return; }
  if(!strcmp(text,"D") || !strcmp(text,"STOP") || !strcmp(text,"DISARM") || !strcmp(text,"V0")) { stop(); return; }
  if(!strcmp(text,"REPORT ON")) { report=true; return; }
  if(!strcmp(text,"REPORT OFF")) { report=false; return; }
  auto r=request(); const auto s=status();
  const bool calBusy=s.mode==Mode::Calibrate || (r.mode==Mode::Calibrate && r.generation!=s.generation);
  if(!strcmp(text,"CAL") || !strcmp(text,"A")) {
    if(s.mode!=Mode::Off || calBusy) { Serial.println("Use D and wait en=0 before CAL"); return; }
    r.mode=Mode::Calibrate; ++r.generation; submit(r); return;
  }
  if(!strcmp(text,"CHECK")) {
    if(s.mode!=Mode::Off || calBusy) { Serial.println("Use D before CHECK"); return; }
    r.mode=Mode::Off; ++r.generation; ++r.check; submit(r); return;
  }
  if(!strncmp(text,"SPEED ",6)) {
    if(s.mode!=Mode::Off || calBusy) { Serial.println("Use D before SPEED"); return; }
    if(strcmp(text+6,"100000") && strcmp(text+6,"400000")) { Serial.println("Use SPEED 100000 or SPEED 400000"); return; }
    r.mode=Mode::Off; r.clock=strtoul(text+6,nullptr,10); ++r.generation; submit(r); return;
  }
  if(!strncmp(text,"I ",2)) {
    float id;
    if(s.mode!=Mode::Off || calBusy) { Serial.println("Use D before selecting another wheel"); return; }
    if(!number(text+2,id) || id<1 || id>4 || floorf(id)!=id) { Serial.println("Use I 1 .. I 4"); return; }
    r.mode=Mode::Off; r.wheel=int(id)-1; ++r.generation; submit(r);
    Serial.printf("SELECTED wheel=%d ch=%u; no motion command sent\n",r.wheel+1,Config::CHANNEL[r.wheel]); return;
  }
  float speed=0,seconds=Config::DEFAULT_SECONDS;
  bool run=false;
  if(text[0]=='V' && number(text+1,speed)) run=true;
  else if(!strncmp(text,"RUN ",4)) {
    char* middle=strchr(text+4,' ');
    if(middle) { *middle=0; run=number(text+4,speed) && number(middle+1,seconds); }
  }
  if(run) {
    if(speed==0) { stop(); return; }
    if(!validRun(speed,seconds)) { Serial.println("RUN limit: nonzero +/-12 rad/s, duration 0.1..10s"); return; }
    if(calBusy || !s.calibrated || s.wheel!=r.wheel || s.fault) { Serial.println("RUN blocked: select wheel, CAL successfully, then RUN"); return; }
    r.mode=Mode::Run; r.target=speed; r.deadline=millis()+uint32_t(seconds*1000); ++r.generation; submit(r); return;
  }
  Serial.println("Invalid command. H for help; one command per line.");
}
}
void setup() {
  pinMode(Config::ENABLE,OUTPUT); digitalWrite(Config::ENABLE,LOW);
  Serial.setTxBufferSize(4096); Serial.begin(115200);
  mainAlive.store(millis());
  Serial.println("QUAD WHEEL DIAGNOSTIC: selected wheel only, outputs OFF, REPORT OFF"); help();
  if(xTaskCreatePinnedToCore(task,"single_wheel_foc",10240,nullptr,3,nullptr,1)!=pdPASS) Serial.println("TASK FAILED: outputs remain OFF");
}
void loop() {
  mainAlive.store(millis());
  static char buffer[96]; static size_t used=0; static bool overflow=false; static uint32_t lastReport=0;
  for(int budget=0;budget<128 && Serial.available();++budget) {
    const char c=Serial.read();
    if(c=='\r' || c=='\n') {
      if(!overflow && used) { buffer[used]=0; command(buffer); }
      if(overflow) Serial.println("Command too long; discarded");
      used=0; overflow=false;
    } else if(!overflow) {
      if(used<sizeof(buffer)-1) buffer[used++]=c; else { used=0; overflow=true; }
    }
  }
  if(report && millis()-lastReport>=500) { lastReport=millis(); printStatus(); }
  delay(1);
}
