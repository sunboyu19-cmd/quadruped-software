#include "host_console.h"
#if ROBOT_MASTER
#include <WiFi.h>
#include <freertos/ringbuf.h>
#include <lwip/sockets.h>
#include <errno.h>
namespace {
WiFiServer server(9000); WiFiClient client;
RingbufHandle_t output=nullptr;
volatile bool networkOwner=false;
uint32_t generation=0;
uint8_t pending[256]; size_t used=0,sent=0;
}
#endif
HostConsole hostConsole;
size_t HostConsole::write(uint8_t c) { return write(&c,1); }
size_t HostConsole::write(const uint8_t* data,size_t len) {
  const size_t n=Serial.write(data,len);
#if ROBOT_MASTER
  if(networkOwner && output) xRingbufferSend(output,data,len,0);
#endif
  return n;
}
void beginHostNetwork() {
#if ROBOT_MASTER
  output=xRingbufferCreate(8192,RINGBUF_TYPE_BYTEBUF);
  WiFi.mode(WIFI_AP); WiFi.setSleep(false);
  if(!output || !WiFi.softAP("QUAD-Control","quadrobot2026",1,false,1)) {
    Serial.println("WIFI AP init failed; USB control remains available"); return;
  }
  server.begin(); server.setNoDelay(true);
  Serial.printf("WIFI AP QUAD-Control IP=%s TCP=9000\n",WiFi.softAPIP().toString().c_str());
#endif
}
bool pollHostNetwork() {
#if ROBOT_MASTER
  bool changed=false;
  if(networkOwner && !client.connected()) {
    networkOwner=false; client.stop(); ++generation; changed=true; used=sent=0;
  }
  if(server.hasClient()) {
    WiFiClient incoming=server.available();
    if(networkOwner) incoming.stop();
    else {
      // Discard old telemetry before allowing a new controller to read it.
      if(output) { size_t n; void* p; while((p=xRingbufferReceive(output,&n,0))) vRingbufferReturnItem(output,p); }
      client=incoming; client.setNoDelay(true); networkOwner=true;
      ++generation; changed=true; used=sent=0;
    }
  }
  if(networkOwner) {
    while(Serial.available()) Serial.read(); // USB commands cannot compete with WiFi.
    if(sent==used && output) {
      size_t n=0; void* p=xRingbufferReceiveUpTo(output,&n,0,sizeof(pending));
      if(p) { memcpy(pending,p,n); used=n; sent=0; vRingbufferReturnItem(output,p); }
    }
    if(sent<used) {
      const int n=::send(client.fd(),pending+sent,used-sent,MSG_DONTWAIT);
      if(n>0) sent+=n;
      else if(n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK) client.stop();
    }
  }
  return changed;
#else
  return false;
#endif
}
int hostAvailable() {
#if ROBOT_MASTER
  if(networkOwner) return client.available();
#endif
  return Serial.available();
}
int hostRead() {
#if ROBOT_MASTER
  if(networkOwner) return client.read();
#endif
  return Serial.read();
}
uint32_t hostGeneration() {
#if ROBOT_MASTER
  return generation;
#else
  return 0;
#endif
}
