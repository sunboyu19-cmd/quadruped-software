#pragma once
#include <stdint.h>

namespace AutoFollow {
constexpr int WINDOW=18,HARD_LIMIT=64,STEP=20;
constexpr uint32_t TIMEOUT_MS=2000,CENTER_INTERVAL_MS=100;
constexpr int CENTER_MAX_LEAD=48,CENTER_SETTLE=16,POSTURE_SETTLE=12;
static_assert(CENTER_SETTLE<=WINDOW,"The midpoint handover must be inside the IMU following window.");
constexpr uint32_t CENTER_SETTLE_TIMEOUT_MS=4500;
constexpr float CENTER_STEP_FRACTION=0.04f;
constexpr bool centerCanAdvance(int proposedLead) { return proposedLead<=CENTER_MAX_LEAD; }
static_assert(CENTER_MAX_LEAD<HARD_LIMIT,"Centering target must leave room below the hard fault limit.");
// Largest forward step that stays within the feedback lead budget.
constexpr int centerStepRoom(int position,int goal,int delta) {
  if(!delta) return 0;
  const int room=CENTER_MAX_LEAD-(goal-position)*(delta>0 ? 1:-1);
  return room>0 ? room:0;
}
static_assert(centerStepRoom(3481,3516,16)==13 && centerStepRoom(2829,2797,-17)==16,
  "Centering must use the remaining lead instead of rejecting the whole step.");
static_assert(centerStepRoom(3481,3529,16)==0,"A stalled servo cannot accumulate targets beyond the lead limit.");
enum class Action { Advance,Wait,Fault };
struct Gate {
  bool waiting=false;
  uint32_t since=0;
  constexpr Action tick(int error,uint32_t now) {
    return tick(error,now,error<=WINDOW);
  }
  constexpr Action tick(int error,uint32_t now,bool ready,uint32_t timeout=TIMEOUT_MS) {
    if(error>HARD_LIMIT) return Action::Fault;
    if(ready) { waiting=false; since=0; return Action::Advance; }
    if(!waiting) { waiting=true; since=now; }
    return uint32_t(now-since)>timeout ? Action::Fault:Action::Wait;
  }
};
static_assert(WINDOW+STEP<HARD_LIMIT,"Automatic targets must retain following-error margin.");
// Exercise the same gate used by the firmware against a slow simulated servo.
constexpr bool slowServo() {
  Gate gate{}; int goal=0,position=0;
  bool waited=false,reached=false;
  for(uint32_t t=0;t<12000;t+=50) {
    if(position<goal) ++position;
    const auto action=gate.tick(goal-position,t);
    if(action==Action::Fault || goal-position>WINDOW+STEP) return false;
    if(action==Action::Wait) waited=true;
    if(action==Action::Advance && goal<160) goal+=STEP;
    if(goal==160 && position==160) reached=true;
  }
  return waited && reached;
}
constexpr bool stalledServo() {
  Gate gate{}; int goal=0; bool fault=false;
  for(uint32_t t=0;t<3000;t+=50) {
    const auto action=gate.tick(goal,t);
    if(action==Action::Fault) { fault=true; break; }
    if(action==Action::Advance) goal+=STEP;
  }
  return fault && goal==STEP;
}
constexpr bool disturbedServo() {
  Gate gate{};
  return gate.tick(65,0)==Action::Fault && gate.tick(12,1)==Action::Advance;
}
constexpr bool timerWrap() {
  Gate gate{};
  return gate.tick(20,0xffffff00u)==Action::Wait && gate.tick(20,0x800u)==Action::Fault;
}
static_assert(slowServo(),"Slow feedback must pause target progress and then finish without accumulating lag.");
static_assert(stalledServo(),"A blocked servo must time out instead of receiving advancing targets.");
static_assert(disturbedServo(),"The 64-count hard protection must remain in effect.");
static_assert(timerWrap(),"Follow timeout must survive millis wrap.");
}
