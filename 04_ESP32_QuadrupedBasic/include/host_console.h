#pragma once
#include <Arduino.h>
class HostConsole : public Print {
public:
  using Print::write;
  size_t write(uint8_t c) override;
  size_t write(const uint8_t* data,size_t len) override;
};
extern HostConsole hostConsole;
void beginHostNetwork();
bool pollHostNetwork(); // true when transport changes; caller stops actuators.
int hostAvailable();
int hostRead();
uint32_t hostGeneration();
