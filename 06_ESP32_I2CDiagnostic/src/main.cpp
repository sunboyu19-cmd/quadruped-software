#include <Arduino.h>
#include <Wire.h>
#include <stdlib.h>
#include <string.h>

#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Use the classic ESP32-WROOM controller, not S3/C3."
#endif
namespace {
constexpr int SDA_PIN=21,SCL_PIN=22,ENABLE_PIN=13;
constexpr uint8_t CHANNELS[4]={7,2,6,1};
const char* const NAMES[4]={"FL(left-front)","FR(right-front)","RL(left-rear)","RR(right-rear)"};
uint8_t muxAddress=0x70;
bool watch=false;
uint32_t lastWatch=0,speed=100000,checks=0,errors[5]={};
int previous[4]={-1,-1,-1,-1};
bool upstream[127]={};
uint8_t lastAckError=0;

void lines() {
  Serial.printf("BUS idle SDA21=%d SCL22=%d (normally both 1); speed=%lu timeout_setting_ms=%u\n",
    digitalRead(SDA_PIN),digitalRead(SCL_PIN),(unsigned long)speed,Wire.getTimeOut());
}

bool ack(uint8_t address) {
  Wire.beginTransmission(address); lastAckError=Wire.endTransmission(); return lastAckError==0;
}
bool muxMask(uint8_t mask) {
  Wire.beginTransmission(muxAddress); Wire.write(mask);
  const uint8_t error=Wire.endTransmission();
  if(error!=0) {
    Serial.printf("MUX FAIL addr=0x%02X mask=0x%02X write_error=%u (2=NACK,5=timeout)\n",muxAddress,mask,error);
    lines(); return false;
  }
  if(Wire.requestFrom(muxAddress,uint8_t(1))!=1) {
    Serial.printf("MUX FAIL addr=0x%02X mask=0x%02X readback missing\n",muxAddress,mask);
    lines(); return false;
  }
  const int actual=Wire.read();
  if(actual!=mask) { Serial.printf("MUX FAIL mask wrote=0x%02X got=0x%02X\n",mask,actual); return false; }
  return true;
}
bool select(uint8_t channel) { return channel<8 && muxMask(uint8_t(1u<<channel)); }
bool readReg(uint8_t address,uint8_t reg,uint8_t* data,uint8_t n) {
  Wire.beginTransmission(address); Wire.write(reg);
  if(Wire.endTransmission(false)!=0 || Wire.requestFrom(address,n)!=n) return false;
  for(uint8_t i=0;i<n;++i) { const int b=Wire.read(); if(b<0) return false; data[i]=uint8_t(b); }
  return true;
}
bool writeReg(uint8_t address,uint8_t reg,uint8_t value) {
  Wire.beginTransmission(address); Wire.write(reg); Wire.write(value); return Wire.endTransmission()==0;
}
int signed16(const uint8_t* p) { return int16_t((uint16_t(p[0])<<8)|p[1]); }

void scan() {
  Serial.printf("SCAN SDA=21 SCL=22 speed=%lu mux=0x%02X; IDs below are CHANNELS, not device addresses\n",(unsigned long)speed,muxAddress);
  if(!ack(muxAddress)) {
    Serial.println("MUX MISSING. Check 3.3V/GND/SDA/SCL/RST and A0/A1/A2. ACK candidates:");
    for(uint8_t a=0x70;a<=0x77;++a) if(ack(a)) Serial.printf("  candidate=0x%02X (use MUX %02X if this is your mux)\n",a,a);
    return;
  }
  if(!muxMask(0)) return;
  memset(upstream,0,sizeof(upstream));
  Serial.print("UPSTREAM (all channels OFF):");
  for(uint8_t a=1;a<127;++a) {
    if(ack(a)) { upstream[a]=true; Serial.printf(" 0x%02X",a); }
    if(lastAckError==5) {
      Serial.println(" TIMEOUT: upstream scan incomplete; stop full scan and inspect BUS");
      lines(); muxMask(0); return;
    }
  }
  Serial.println();
  if(upstream[0x36] || upstream[0x68] || upstream[0x69])
    Serial.println("NOTE: sensor visible UPSTREAM; your declared wiring puts all sensors downstream. Check actual wiring.");
  for(uint8_t ch=0;ch<8;++ch) {
    Serial.printf("SELECT channel=%u mask=0x%02X\n",ch,1u<<ch);
    if(!select(ch)) { Serial.println("SCAN stopped: selection failed; use OFF / LINES / TEST n"); muxMask(0); return; }
    Serial.printf("CHANNEL %u downstream:",ch); int found=0;
    bool complete=true;
    for(uint8_t a=1;a<127;++a) if(!upstream[a]) {
      if(ack(a)) { ++found; Serial.printf(" 0x%02X",a); }
      if(lastAckError==5) { complete=false; Serial.print(" TIMEOUT scan incomplete"); break; }
    }
    if(!found && complete) Serial.print(" none");
    Serial.println();
    if(!complete) { lines(); muxMask(0); return; }
  }
  muxMask(0);
  Serial.println("EXPECTED ch0=MPU6050 0x68; ch1=RR 0x36; ch2=FR 0x36; ch6=RL 0x36; ch7=FL 0x36");
}
void checkEncoder(int i) {
  uint8_t mag=0,bytes[2]={}; const uint8_t ch=CHANNELS[i];
  if(!select(ch) || !readReg(0x36,0x0b,&mag,1) || !readReg(0x36,0x0c,bytes,2)) {
    ++errors[i+1]; Serial.printf("ENC wheel=%d %s ch=%u comm=FAIL errors=%lu\n",i+1,NAMES[i],ch,(unsigned long)errors[i+1]); return;
  }
  const int raw=((bytes[0]&15)<<8)|bytes[1]; int delta=0;
  if(previous[i]>=0) { delta=raw-previous[i]; if(delta>2048) delta-=4096; if(delta< -2048) delta+=4096; }
  previous[i]=raw;
  const char* field=!(mag&0x20) ? "NOT_DETECTED":(mag&0x10) ? "TOO_WEAK":(mag&0x08) ? "TOO_STRONG":"OK";
  Serial.printf("ENC wheel=%d %s ch=%u comm=OK mag=0x%02X magnet=%s raw=%d deg=%.2f delta=%d errors=%lu\n",
    i+1,NAMES[i],ch,mag,field,raw,raw*360.0f/4096,delta,(unsigned long)errors[i+1]);
}
uint8_t imuAddress() { return ack(0x68) ? 0x68:ack(0x69) ? 0x69:0; }
void checkImu(bool wake=false) {
  if(!select(0)) { ++errors[0]; return; }
  const uint8_t addr=imuAddress(); uint8_t who=0,power=0,bytes[14]={},ac=0,gc=0;
  if(!addr || !readReg(addr,0x75,&who,1) || !readReg(addr,0x6b,&power,1)) {
    ++errors[0]; Serial.printf("IMU ch=0 comm=FAIL errors=%lu\n",(unsigned long)errors[0]); return;
  }
  if(who!=0x68) { ++errors[0]; Serial.printf("IMU ch=0 addr=0x%02X comm=OK WHO=0x%02X WRONG_ID (MPU6050 expects 0x68)\n",addr,who); return; }
  if(wake) {
    if(!writeReg(addr,0x6b,0x01)) { ++errors[0]; Serial.println("IMU wake FAILED"); return; }
    delay(100); readReg(addr,0x6b,&power,1);
  }
  if(!readReg(addr,0x3b,bytes,14) || !readReg(addr,0x1c,&ac,1) || !readReg(addr,0x1b,&gc,1)) {
    ++errors[0]; Serial.println("IMU comm=FAIL burst/config read"); return;
  }
  const float accelScale[4]={16384,8192,4096,2048},gyroScale[4]={131,65.5f,32.8f,16.4f};
  const float as=accelScale[(ac>>3)&3],gs=gyroScale[(gc>>3)&3];
  Serial.printf("IMU ch=0 addr=0x%02X comm=OK WHO=0x%02X sleep=%d ax=%.3f ay=%.3f az=%.3f g gx=%.2f gy=%.2f gz=%.2f dps T=%.1f errors=%lu\n",
    addr,who,(power&0x40)!=0,signed16(bytes)/as,signed16(bytes+2)/as,signed16(bytes+4)/as,
    signed16(bytes+8)/gs,signed16(bytes+10)/gs,signed16(bytes+12)/gs,signed16(bytes+6)/340.0f+36.53f,(unsigned long)errors[0]);
  if(power&0x40) Serial.println("IMU communication is OK but sleeping. Send IMU ON to wake for live data.");
}
void check() {
  Serial.printf("CHECK %lu speed=%lu\n",(unsigned long)++checks,(unsigned long)speed);
  if(!ack(muxAddress)) { Serial.println("MUX MISSING; downstream test skipped"); return; }
  checkImu(); for(int i=0;i<4;++i) checkEncoder(i); muxMask(0);
}
void testChannel(uint8_t channel) {
  watch=false;
  Serial.printf("TEST channel=%u; isolate all other branches first\n",channel); lines();
  if(!muxMask(0)) { Serial.println("Cannot deselect branches. Stop; check supply/lines or power-cycle logic before retrying."); return; }
  const uint32_t started=millis();
  if(channel==0) checkImu();
  else {
    bool known=false;
    for(int i=0;i<4;++i) if(CHANNELS[i]==channel) { known=true; checkEncoder(i); break; }
    if(!known && select(channel)) Serial.println("Unused channel selected; no sensor expected");
  }
  Serial.printf("TEST channel=%u elapsed_ms=%lu\n",channel,(unsigned long)(millis()-started));
  lines();
  Serial.printf("OFF verified=%d\n",muxMask(0));
}
void help() {
  Serial.println("H | SCAN (all 8 branches) | CHECK (5 expected sensors) | WATCH ON / WATCH OFF (500ms)");
  Serial.println("IMU ON (wake MPU6050 ONLY) | SPEED 100000 / SPEED 400000 | MUX 70..77 (hex) | RESETSTATS");
  Serial.println("LINES | OFF (deselect all branches) | TEST <0..7> (one branch, auto WATCH OFF)");
  Serial.println("No motor PWM, no servo UART, no WiFi. EN13 LOW. Keep 6V motion power OFF.");
}
void command(char* text) {
  for(char* p=text;*p;++p) if(*p>='a' && *p<='z') *p-=32;
  if(!strcmp(text,"H")) help();
  else if(!strcmp(text,"SCAN")) scan();
  else if(!strcmp(text,"CHECK") || !strcmp(text,"S")) check();
  else if(!strcmp(text,"LINES")) lines();
  else if(!strcmp(text,"OFF")) { watch=false; Serial.printf("OFF verified=%d\n",muxMask(0)); lines(); }
  else if(!strncmp(text,"TEST ",5)) {
    char* end=nullptr; const unsigned long ch=strtoul(text+5,&end,10);
    if(end==text+5 || *end || ch>7) { Serial.println("Use TEST 0 .. TEST 7"); return; }
    testChannel(uint8_t(ch));
  }
  else if(!strcmp(text,"WATCH ON")) { watch=true; lastWatch=millis(); check(); }
  else if(!strcmp(text,"WATCH OFF")) { watch=false; Serial.println("WATCH OFF"); }
  else if(!strcmp(text,"IMU ON")) { checkImu(true); muxMask(0); }
  else if(!strcmp(text,"RESETSTATS")) { memset(errors,0,sizeof(errors)); for(auto& p:previous) p=-1; checks=0; Serial.println("STATS RESET"); }
  else if(!strcmp(text,"SPEED 100000") || !strcmp(text,"SPEED 400000")) {
    speed=uint32_t(strtoul(text+6,nullptr,10)); Wire.setClock(speed); Serial.printf("I2C speed=%lu\n",(unsigned long)speed);
  } else if(!strncmp(text,"MUX ",4)) {
    char* end=nullptr; const unsigned long address=strtoul(text+4,&end,16);
    if(end==text+4 || *end || address<0x70 || address>0x77) { Serial.println("Use MUX 70 .. MUX 77 (hex)"); return; }
    muxMask(0); muxAddress=uint8_t(address); Serial.printf("MUX address=0x%02X\n",muxAddress);
  } else Serial.println("Unknown command. H for help; one command per line.");
}
}
void setup() {
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,LOW);
  Serial.begin(115200); delay(1000);
  Wire.begin(SDA_PIN,SCL_PIN); Wire.setClock(speed); Wire.setTimeOut(10);
  Serial.println("QUAD I2C DIAGNOSTIC v2: classic ESP32, motion outputs OFF");
  help(); lines();
  Serial.println("No automatic full scan. Send OFF then TEST 7 / TEST 6 / TEST 2 / TEST 1 / TEST 0, one at a time.");
}
void loop() {
  static char buffer[64]; static size_t used=0; static bool overflow=false;
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
  if(watch && millis()-lastWatch>=500) { lastWatch=millis(); check(); }
  delay(1);
}
