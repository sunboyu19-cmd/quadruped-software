#pragma once
#include <stdint.h>
namespace Config {
// ESP32-DevKitC V4 / WROOM-32E, NOT S3/C3/WROVER.
constexpr float VM = 6.0f;
constexpr int POLE_PAIRS = 7;
constexpr float ALIGN_V = 1.5f;
// Voltage-mode FOC has no measured current/temperature protection.
constexpr float LIMIT_V = 2.5f;
constexpr float MAX_SPEED = 12.0f; // Logical wheel rad/s.
constexpr float MAX_TURN = 4.0f; // Differential component, wheel rad/s.
constexpr float TEST_SPEED = 1.0f;
constexpr float OVERSPEED = 30.0f;
constexpr int SDA = 25, SCL = 33;
constexpr uint8_t MUX_ADDRESS = 0x70; // TCA9548A A0/A1/A2 -> GND.
constexpr uint8_t MUX_CHANNEL[4] = {7,2,6,1}; // FL, FR, RL, RR; actual wiring 2026-11-14.
constexpr uint8_t IMU_CHANNEL = 0;
constexpr uint8_t IMU_ADDRESS = 0x68; // MPU6050 AD0 -> GND, downstream channel 0.
constexpr int8_t IMU_AXES[3] = {1,2,3}; // Body X/Y/Z from signed sensor axes; X forward,Y left,Z up.
// FL, FR, RL, RR; each row is PWM A/B/C (confirmed wiring 2026-11-17).
constexpr int PWM[4][3] = {{4,16,17}, {13,14,27}, {21,22,23}, {5,18,19}};
constexpr int ENABLE = 26; // Four EN inputs together; external pulldown required, see README.
constexpr unsigned long CONTROL_TIMEOUT_MS = 500; // Local command-task watchdog.
constexpr unsigned long DRIVE_TIMEOUT_MS = 1500;
constexpr int SERVO_TX = 32;  // GPIO32 -> Waveshare Adapter(A) TX; verified adapter labeling.
constexpr int SERVO_RX = 35;  // GPIO35 input only; no internal pull-up/down.
constexpr unsigned long SERVO_BAUD = 1000000;
constexpr int SERVO_IDS[4] = {1,2,3,4}; // FL, FR, RL, RR
constexpr float SERVO_V_MIN = 5.0f;
constexpr float SERVO_V_MAX = 6.8f;
constexpr int SERVO_TEMP_MAX = 60;
constexpr int SERVO_SPEED = 500;
constexpr int SERVO_ACC = 20;
constexpr int SERVO_TORQUE = 800; // 80% register limit, NOT a current limit.
constexpr float SERVO_RANGE_DEG = 15;
constexpr float SERVO_STEP_DEG = 5;

constexpr bool pinsValid() {
  const int pins[]={PWM[0][0],PWM[0][1],PWM[0][2],PWM[1][0],PWM[1][1],PWM[1][2],PWM[2][0],PWM[2][1],PWM[2][2],PWM[3][0],PWM[3][1],PWM[3][2],ENABLE,SDA,SCL,SERVO_TX,SERVO_RX};
  for(unsigned i=0;i<sizeof(pins)/sizeof(pins[0]);++i)
    for(unsigned j=i+1;j<sizeof(pins)/sizeof(pins[0]);++j)
      if(pins[i]==pins[j]) return false;
  for(unsigned i=0;i<4;++i) {
    if(MUX_CHANNEL[i]>7 || MUX_CHANNEL[i]==IMU_CHANNEL) return false;
    for(unsigned j=i+1;j<4;++j) if(MUX_CHANNEL[i]==MUX_CHANNEL[j]) return false;
  }
  return IMU_CHANNEL<8;
}
}
static_assert(Config::pinsValid(),"Duplicate GPIO or invalid mux channel");
constexpr bool imuAxesValid() {
  int sign=1,inversions=0;
  for(int i=0;i<3;++i) {
    int a=Config::IMU_AXES[i]; if(a<0) { a=-a; sign=-sign; }
    if(a<1 || a>3) return false;
    for(int j=i+1;j<3;++j) {
      int b=Config::IMU_AXES[j]; if(b<0) b=-b;
      if(a==b) return false;
      if(a>b) ++inversions;
    }
  }
  return sign*(inversions%2 ? -1:1)==1;
}
static_assert(imuAxesValid(),"IMU axes require a right-handed signed permutation");
