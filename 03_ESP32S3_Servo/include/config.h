#pragma once
namespace Config {
constexpr int UART_TX = 17;  // GPIO17 -> Waveshare Adapter(A) TX
constexpr int UART_RX = 18;  // GPIO18 -> Waveshare Adapter(A) RX
constexpr unsigned long BAUD = 1000000;
constexpr unsigned long TIMEOUT_MS = 30000;
constexpr int DEFAULT_ID = 1;
constexpr int SPEED_RAW = 100; // Never zero (may mean maximum speed).
constexpr int ACC_RAW = 10;
constexpr int TORQUE_LIMIT_RAW = 200; // 20% of servo maximum, NOT a current limit.
constexpr float MAX_STEP_DEG = 5.0f;
constexpr float MAX_TRAVEL_DEG = 30.0f;
constexpr float MIN_VOLTAGE = 5.0f;
constexpr float MAX_VOLTAGE = 6.8f; // Project uses a 6V supply.
constexpr int MAX_TEMPERATURE = 60;
}
