#include "host_console.h"
#include <WiFi.h>
#include <BluetoothSerial.h>
#include <freertos/ringbuf.h>
#include <freertos/queue.h>
#include <lwip/sockets.h>
#include <errno.h>
#include <atomic>

#if !defined(CONFIG_BT_SPP_ENABLED)
#error "Classic Bluetooth SPP requires the original ESP32, not S3/C3."
#endif
namespace {
enum class Owner:uint8_t { Usb,Wifi,Bluetooth };
std::atomic<Owner> owner{Owner::Usb};
std::atomic<uint32_t> generation{0},btHandle{0},btSession{0};
WiFiServer server(9000); WiFiClient client;
BluetoothSerial bluetooth;
RingbufHandle_t output=nullptr;
struct BtChunk { uint32_t generation; uint16_t size; uint8_t bytes[192]; };
QueueHandle_t btOutput=nullptr;
bool wifiReady=false,btReady=false;
uint32_t acceptedBtHandle=0,acceptedBtSession=0,rejectedBtHandle=0;
uint8_t pendingBytes[256]; size_t used=0,sent=0;

void bluetoothEvent(esp_spp_cb_event_t event,esp_spp_cb_param_t* param) {
  if(event==ESP_SPP_SRV_OPEN_EVT && param->srv_open.status==ESP_SPP_SUCCESS) {
    uint32_t empty=0;
    if(btHandle.compare_exchange_strong(empty,param->srv_open.handle)) ++btSession;
  } else if(event==ESP_SPP_CLOSE_EVT) {
    uint32_t expected=param->close.handle;
    if(btHandle.compare_exchange_strong(expected,0)) ++btSession;
  }
}
void drain(Stream& input) {
  const int count=input.available();
  for(int i=0;i<count && i<2048;++i) input.read();
}
void switchOwner(Owner next) {
  owner.store(Owner::Usb); // Do not enqueue outgoing telemetry during the transition.
  ++generation; used=sent=0;
  drain(Serial);
  if(btReady) drain(bluetooth);
  if(output) {
    size_t n; void* data;
    while((data=xRingbufferReceive(output,&n,0))) vRingbufferReturnItem(output,data);
  }
  if(btOutput) xQueueReset(btOutput);
  owner.store(next);
}
void bluetoothWriter(void*) {
  BtChunk chunk;
  for(;;) {
    if(xQueueReceive(btOutput,&chunk,portMAX_DELAY)==pdTRUE &&
       owner.load()==Owner::Bluetooth && chunk.generation==generation.load() && bluetooth.hasClient()) {
      // BluetoothSerial::write can wait on its internal queue. It must NEVER
      // run in the wheel task or command loop; only this low-priority task calls it.
      bluetooth.write(chunk.bytes,chunk.size);
    }
  }
}
}
HostConsole hostConsole;
size_t HostConsole::write(uint8_t c) { return write(&c,1); }
size_t HostConsole::write(const uint8_t* data,size_t len) {
  const size_t n=Serial.write(data,len);
  const Owner current=owner.load();
  if(current==Owner::Wifi && output) xRingbufferSend(output,data,len,0);
  else if(current==Owner::Bluetooth && btOutput) {
    const uint32_t epoch=generation.load();
    for(size_t offset=0;offset<len;) {
      BtChunk chunk{}; chunk.generation=epoch;
      chunk.size=uint16_t(len-offset<sizeof(chunk.bytes) ? len-offset:sizeof(chunk.bytes));
      memcpy(chunk.bytes,data+offset,chunk.size);
      if(xQueueSend(btOutput,&chunk,0)!=pdTRUE) break; // Drop telemetry on congestion; never block FOC.
      offset+=chunk.size;
    }
  }
  return n;
}
void beginHostNetwork() {
  output=xRingbufferCreate(8192,RINGBUF_TYPE_BYTEBUF);
  WiFi.mode(WIFI_AP); WiFi.setSleep(false);
  wifiReady=output && WiFi.softAP("QUAD-Single","quadrobot2026",1,false,1);
  if(wifiReady) {
    server.begin(); server.setNoDelay(true);
    Serial.printf("WIFI AP QUAD-Single IP=%s TCP=9000\n",WiFi.softAPIP().toString().c_str());
  } else Serial.println("WIFI init failed; other transports remain available");

  btOutput=xQueueCreate(12,sizeof(BtChunk));
  if(btOutput && xTaskCreatePinnedToCore(bluetoothWriter,"bt_console_tx",4096,nullptr,1,nullptr,0)==pdPASS) {
    bluetooth.register_callback(bluetoothEvent);
    btReady=bluetooth.begin("QUAD-Single-BT");
    bluetooth.setTimeout(0);
    if(btReady) {
      bluetooth.setPin("246810");
      Serial.println("BLUETOOTH Classic SPP QUAD-Single-BT ready; pair in Windows, choose outgoing COM port");
    } else Serial.println("BLUETOOTH init failed; USB/WiFi remain available");
  } else Serial.println("BLUETOOTH output task allocation failed; USB/WiFi remain available");
}
bool pollHostNetwork() {
  bool changed=false;
  Owner current=owner.load();
  const uint32_t handle=btHandle.load(),session=btSession.load();
  if(current==Owner::Wifi && !client.connected()) {
    client.stop(); switchOwner(Owner::Usb); changed=true;
  } else if(current==Owner::Bluetooth && (!bluetooth.hasClient() ||
             handle!=acceptedBtHandle || session!=acceptedBtSession)) {
    switchOwner(Owner::Usb); changed=true;
  }
  if(!handle) rejectedBtHandle=0;
  if(wifiReady && server.hasClient()) {
    WiFiClient incoming=server.available();
    if(owner.load()!=Owner::Usb) incoming.stop();
    else {
      client=incoming; client.setNoDelay(true);
      switchOwner(Owner::Wifi); changed=true;
    }
  }
  if(btReady && handle && bluetooth.hasClient()) {
    if(owner.load()==Owner::Usb && handle!=rejectedBtHandle) {
      acceptedBtHandle=handle; acceptedBtSession=session;
      switchOwner(Owner::Bluetooth); changed=true;
    } else if(owner.load()==Owner::Wifi && handle!=rejectedBtHandle) {
      // Do not use BluetoothSerial::disconnect(): it waits for pending writes.
      rejectedBtHandle=handle; esp_spp_disconnect(handle);
      Serial.println("BLUETOOTH rejected: WiFi already owns control; disconnect WiFi first");
    }
  }
  current=owner.load();
  if(current!=Owner::Usb) drain(Serial);
  if(btReady && current!=Owner::Bluetooth) drain(bluetooth);
  if(current==Owner::Wifi) {
    if(sent==used && output) {
      size_t n=0; void* data=xRingbufferReceiveUpTo(output,&n,0,sizeof(pendingBytes));
      if(data) { memcpy(pendingBytes,data,n); used=n; sent=0; vRingbufferReturnItem(output,data); }
    }
    if(sent<used) {
      const int n=::send(client.fd(),pendingBytes+sent,used-sent,MSG_DONTWAIT);
      if(n>0) sent+=n;
      else if(n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK) client.stop();
    }
  }
  return changed;
}
int hostAvailable() {
  const Owner current=owner.load();
  if(current==Owner::Wifi) return client.available();
  if(current==Owner::Bluetooth) return bluetooth.available();
  return Serial.available();
}
int hostRead() {
  const Owner current=owner.load();
  if(current==Owner::Wifi) return client.read();
  if(current==Owner::Bluetooth) return bluetooth.read();
  return Serial.read();
}
uint32_t hostGeneration() { return generation.load(); }
const char* hostTransport() {
  switch(owner.load()) {
    case Owner::Wifi:return "wifi";
    case Owner::Bluetooth:return "bluetooth";
    default:return "serial";
  }
}
