#pragma once

namespace Config {
// Classic ESP32-WROOM-32E DevKitC only (not C3/S3).
constexpr int SDA_PIN = 21;
constexpr int SCL_PIN = 22;
constexpr int PWM_A = 25;
constexpr int PWM_B = 26;
constexpr int PWM_C = 27;
constexpr int ENABLE_PIN = 13;
constexpr int FAULT_PIN = -1;  // Optional active-low nFAULT; -1 = not wired.

// Change to true ONLY after checking the actual MiniFOC schematic:
// 3PWM mode, active-HIGH EN/nSLEEP input and supply compatibility.
constexpr bool DRIVER_WIRING_AND_POWER_CONFIRMED = false;

// Set to the voltage measured between the driver's VM and GND before testing.
// 12.6 is an initial full-3S example, NOT a regulator output or a motor rating.
constexpr float SUPPLY_VOLTAGE = 12.6f;
constexpr int POLE_PAIRS = 7;  // Original 2208 setting; verify actual motor.
constexpr float MOTOR_VOLTAGE_LIMIT = 1.5f;
constexpr float ALIGN_VOLTAGE = 1.0f;
constexpr float MAX_TARGET_RAD_S = 5.0f;
constexpr float OVERSPEED_RAD_S = 30.0f;
constexpr unsigned long COMMAND_TIMEOUT_MS = 10000;
}  // namespace Config
