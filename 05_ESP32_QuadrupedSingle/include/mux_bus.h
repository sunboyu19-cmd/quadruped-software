#pragma once
#include <stdint.h>

// Exactly one branch at a time: all four AS5600s use address 0x36.
template<class Bus>
bool selectMux(Bus& bus,uint8_t address,uint8_t channel) {
  if(channel>=8) return false;
  bus.beginTransmission(address); bus.write(uint8_t(1u<<channel));
  return bus.endTransmission()==0;
}
// Always select, including after a mux reset; no cached channel assumption.
template<class Bus>
bool readMuxRegister(Bus& bus,uint8_t mux,uint8_t channel,uint8_t reg,uint8_t* result,uint8_t length) {
  if(!result || !length || !selectMux(bus,mux,channel)) return false;
  bus.beginTransmission(0x36); bus.write(reg);
  if(bus.endTransmission(false)!=0) return false;
  if(bus.requestFrom(uint8_t(0x36),length)!=length) return false;
  for(uint8_t i=0;i<length;++i) {
    const int value=bus.read(); if(value<0) return false;
    result[i]=uint8_t(value);
  }
  return true;
}
