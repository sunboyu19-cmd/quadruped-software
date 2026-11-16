#pragma once
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
constexpr int SDA[2] = {21, 32};
constexpr int SCL[2] = {22, 33};
constexpr int PWM[2][3] = {{25,26,27}, {18,19,23}};
constexpr int EN[2] = {13,14};
constexpr int LINK_RX = 16;
constexpr int LINK_TX = 17;
constexpr unsigned long LINK_BAUD = 115200;
constexpr unsigned long LINK_TIMEOUT_MS = 500;
constexpr unsigned long DRIVE_TIMEOUT_MS = 1500;
constexpr int SERVO_TX = 4;   // Master GPIO4 -> Waveshare Adapter(A) TX
constexpr int SERVO_RX = 34;  // Master GPIO34 -> Waveshare Adapter(A) RX
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
}
