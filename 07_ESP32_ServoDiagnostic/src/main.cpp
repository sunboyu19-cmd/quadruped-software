#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include "servo_protocol.h"
#if !defined(CONFIG_IDF_TARGET_ESP32) || defined(CONFIG_IDF_TARGET_ESP32S3)
#error "This diagnostic is for classic ESP32-WROOM, not S3/C3."
#endif
namespace {
constexpr int TX_PIN=5,RX_PIN=34,ENABLE_PIN=13;
constexpr uint32_t BAUD=1000000,REPLY_TIMEOUT_MS=30;
HardwareSerial bus(1);
bool watch=false,debug=false;
uint32_t lastWatch=0,checks=0,failures[4]={},retries[4]={},echoes[4]={};
uint8_t lastTx[8]={},lastRx[64]={}; size_t lastRxCount=0;
const char* const legs[4]={"FL(left-front)","FR(right-front)","RL(left-rear)","RR(right-rear)"};
// READ is the only instruction emitted. PING alone cannot prove device identity when TX echoes.
enum class ReadInstruction:uint8_t { Read=2 };
bool transaction(uint8_t id,ReadInstruction op,uint8_t reg,uint8_t count,uint8_t* out,uint8_t& status) {
  if(id<1 || id>4 || !count || count>15 || !out || op!=ReadInstruction::Read) return false;
  uint8_t tx[8]={0xff,0xff,id,4,uint8_t(op),reg,count,0};
  const size_t length=8;
  uint8_t sum=0;
  for(size_t i=2;i<length-1;++i) sum=uint8_t(sum+tx[i]);
  tx[length-1]=uint8_t(~sum);
  memcpy(lastTx,tx,sizeof(tx)); lastRxCount=0;
  uint8_t recent[8]={}; size_t recentCount=0;
  while(bus.available()) bus.read();
  bus.write(tx,length); bus.flush();
  const size_t expected=count+6;
  uint8_t frame[21]={}; size_t used=0;
  const uint32_t started=millis();
  while(millis()-started<REPLY_TIMEOUT_MS) {
    if(!bus.available()) { delay(1); continue; }
    const uint8_t incoming=uint8_t(bus.read());
    if(lastRxCount<sizeof(lastRx)) lastRx[lastRxCount++]=incoming;
    if(recentCount<sizeof(recent)) recent[recentCount++]=incoming;
    else { memmove(recent,recent+1,sizeof(recent)-1); recent[sizeof(recent)-1]=incoming; }
    if(recentCount==sizeof(recent) && memcmp(recent,tx,sizeof(recent))==0) ++echoes[id-1];
    frame[used++]=incoming;
    if(used<expected) continue;
    if(ServoProtocol::validReply(frame,expected,id,count,tx,length)) {
      status=frame[4]; if(count) memcpy(out,frame+5,count); return true;
    }
    memmove(frame,frame+1,--used);
  }
  return false;
}
bool query(int id,ReadInstruction op,uint8_t reg,uint8_t count,uint8_t* out,uint8_t& allStatus) {
  for(int attempt=0;attempt<3;++attempt) {
    uint8_t status=0;
    if(transaction(uint8_t(id),op,reg,count,out,status)) { allStatus|=status; return true; }
    if(attempt<2) { ++retries[id-1]; delay(3); }
  }
  ++failures[id-1];
  if(debug) {
    Serial.print("BUS_TX:");for(uint8_t value:lastTx) Serial.printf(" %02X",value);Serial.println();
    Serial.print("BUS_RX:");for(size_t i=0;i<lastRxCount;++i) Serial.printf(" %02X",lastRx[i]);
    if(!lastRxCount) Serial.print(" none");
    Serial.println();
  }
  return false;
}
int readByte(int id,int reg,uint8_t& errors) {
  uint8_t data=0;
  return query(id,ReadInstruction::Read,uint8_t(reg),1,&data,errors) ? data:-1;
}
int readWord(int id,int reg,uint8_t& errors) {
  uint8_t data[2]={};
  return query(id,ReadInstruction::Read,uint8_t(reg),2,data,errors) ? int(data[0])+(int(data[1])<<8):-1;
}
int word(const uint8_t* b) { return int(b[0])+(int(b[1])<<8); }
int signedMagnitude(int value,int bit) { return value&(1<<bit) ? -(value&~(1<<bit)):value; }
bool test(int id) {
  uint8_t errors=0,data[15]={}; const uint32_t before=retries[id-1],echoBefore=echoes[id-1];
  const int actualId=readByte(id,5,errors);
  if(actualId!=id) {
    Serial.printf("SERVO id=%d %s comm=FAIL ID_register=%d; no confirmed servo reply; failures=%lu retries=%lu tx_echoes_now=%lu\n",
      id,legs[id-1],actualId,(unsigned long)failures[id-1],(unsigned long)retries[id-1],(unsigned long)(echoes[id-1]-echoBefore));
    return false;
  }
  if(!query(id,ReadInstruction::Read,56,15,data,errors)) {
    Serial.printf("SERVO id=%d comm=ID_READ_OK feedback=FAIL status=0x%02X failures=%lu\n",id,errors,(unsigned long)failures[id-1]); return false;
  }
  const int position=signedMagnitude(word(data),15),velocity=signedMagnitude(word(data+2),15);
  const int load=signedMagnitude(word(data+4),10),voltage=data[6],temperature=data[7],moving=data[10];
  const int mode=readByte(id,33,errors),torque=readByte(id,40,errors),goal=readWord(id,42,errors);
  const int low=readWord(id,9,errors),high=readWord(id,11,errors);
  const bool config=mode>=0 && torque>=0 && goal>=0 && low>=0 && high>=0;
  const bool sane=position>=0 && position<=4095 && voltage>=50 && voltage<=68 && temperature<60;
  const bool ok=config && sane && !errors && mode==0;
  Serial.printf("SERVO id=%d %s comm=OK feedback=OK config=%s pos=%d deg=%.2f speed_raw=%d load_raw=%d V=%.1f C=%d moving=%d status=0x%02X mode=%d torque=%d goal_raw=%d min=%d max=%d retries_now=%lu failures=%lu result=%s tx_echoes_now=%lu\n",
    id,legs[id-1],config ? "OK":"FAIL",position,position*360.0f/4096,velocity,load,voltage*0.1f,temperature,moving,errors,
    mode,torque,goal,low,high,(unsigned long)(retries[id-1]-before),(unsigned long)failures[id-1],ok ? "READ_ONLY_OK":"CHECK_REQUIRED",(unsigned long)(echoes[id-1]-echoBefore));
  if(torque>0) Serial.printf("NOTE id=%d torque is already ON; diagnostic did not enable or change it.\n",id);
  if(!sane || errors || mode!=0) Serial.printf("NOTE id=%d inspect position/voltage/temperature/status/mode before any motion test.\n",id);
  if(retries[id-1]>before) Serial.printf("NOTE id=%d retry was needed; a successful retry does not prove a stable connection.\n",id);
  return ok;
}
void check() {
  Serial.printf("SERVO_CHECK %lu baud=%lu TX=5 RX=34 READ ONLY\n",(unsigned long)++checks,(unsigned long)BAUD);
  int ok=0; for(int id=1;id<=4;++id) { if(test(id)) ++ok; delay(10); }
  Serial.printf("SUMMARY read_only_ok=%d/4. This does not test movement, torque capability or mechanical limits.\n",ok);
}
void scan() {
  for(int id=1;id<=4;++id) {
    uint8_t error=0; const int actualId=readByte(id,5,error);
    Serial.printf("ID_READ expected=%d actual=%d confirmed=%d status=0x%02X\n",id,actualId,actualId==id,error);
  }
  Serial.println("SCAN reads ID registers 1..4; duplicate same-ID servos cannot be identified reliably.");
}
void help() {
  Serial.println("H | CHECK (S) | TEST <1..4> | SCAN (ID register READ 1..4) | WATCH ON / WATCH OFF (2s) | RESETSTATS");
  Serial.println("DEBUG ON / DEBUG OFF: show failed transaction bytes; echoed TX alone is not a servo response");
  Serial.println("READ ONLY: no torque/position/ID/baud/offset/limit writes. No motor PWM or I2C. EN13 LOW.");
  Serial.println("Support body; servo adapter needs correct 6V and common GND, jumper A, TX5->adapter TX, RX34->adapter RX.");
}
void command(char* text) {
  for(char* p=text;*p;++p) if(*p>='a' && *p<='z') *p-=32;
  if(!strcmp(text,"H")) help();
  else if(!strcmp(text,"CHECK") || !strcmp(text,"S")) check();
  else if(!strcmp(text,"SCAN")) scan();
  else if(!strcmp(text,"DEBUG ON")) { debug=true; Serial.println("DEBUG ON"); }
  else if(!strcmp(text,"DEBUG OFF")) { debug=false; Serial.println("DEBUG OFF"); }
  else if(!strcmp(text,"WATCH ON")) { watch=true; lastWatch=millis(); check(); }
  else if(!strcmp(text,"WATCH OFF")) { watch=false; Serial.println("WATCH OFF"); }
  else if(!strcmp(text,"RESETSTATS")) { memset(retries,0,sizeof(retries)); memset(failures,0,sizeof(failures)); memset(echoes,0,sizeof(echoes)); Serial.println("STATS RESET"); }
  else if(!strncmp(text,"TEST ",5)) {
    char* end=nullptr; const long id=strtol(text+5,&end,10);
    if(end==text+5 || *end || id<1 || id>4) { Serial.println("Use TEST 1 .. TEST 4"); return; }
    watch=false; test(int(id));
  } else Serial.println("Unknown command. H for help. Read-only commands only.");
}
}
void setup() {
  pinMode(ENABLE_PIN,OUTPUT); digitalWrite(ENABLE_PIN,LOW);
  Serial.begin(115200); bus.begin(BAUD,SERIAL_8N1,RX_PIN,TX_PIN); delay(500);
  Serial.println("QUAD SERVO DIAGNOSTIC v2: classic ESP32; IDs 1..4; motion writes DISABLED");
  help(); Serial.println("No automatic scan. Power adapter/servos correctly, then send CHECK.");
}
void loop() {
  static char input[64]; static size_t used=0; static bool overflow=false;
  for(int budget=0;budget<128 && Serial.available();++budget) {
    const char c=Serial.read();
    if(c=='\r' || c=='\n') {
      if(!overflow && used) { input[used]=0; command(input); }
      if(overflow) Serial.println("Command too long; discarded");
      used=0; overflow=false;
    } else if(!overflow) {
      if(used<sizeof(input)-1) input[used++]=c; else { used=0; overflow=true; }
    }
  }
  if(watch && millis()-lastWatch>=2000) { lastWatch=millis(); check(); }
  delay(1);
}
