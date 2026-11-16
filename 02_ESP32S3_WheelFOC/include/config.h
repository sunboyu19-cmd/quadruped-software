#pragma once

#ifndef FOC_LEG_ID
#define FOC_LEG_ID 1
#endif

namespace Config {
// Software label only; it does not change or identify the AS5600 hardware.
static_assert(FOC_LEG_ID == 1 || FOC_LEG_ID == 2, "1=right, 2=left");
constexpr int LEG_ID = FOC_LEG_ID;
constexpr const char* LEG_NAME = LEG_ID == 1 ? "right" : "left";
// ESP32-S3-WROOM-2 N32R16V only. GPIO numbers, not header positions.
// Sequential testing: connect only one leg, using these same pins.
constexpr int SDA_PIN = 8;
constexpr int SCL_PIN = 9;
constexpr int PWM_A = 4;
constexpr int PWM_B = 5;
constexpr int PWM_C = 6;
constexpr int ENABLE_PIN = 7;
constexpr int FAULT_PIN = -1;

// Keep false until the MiniFOC 3PWM mode, active-HIGH EN and VM are verified.
constexpr bool DRIVER_WIRING_AND_POWER_CONFIRMED = true;
// User-selected 6V motor supply; verify the actual VM and board compatibility.
constexpr float SUPPLY_VOLTAGE = 6.0f;
constexpr int POLE_PAIRS = 7;
constexpr float MOTOR_VOLTAGE_LIMIT = 1.5f;
constexpr float ALIGN_VOLTAGE = 1.5f;
constexpr float MAX_TARGET_RAD_S = 5.0f;
constexpr float OVERSPEED_RAD_S = 30.0f;
constexpr unsigned long COMMAND_TIMEOUT_MS = 10000;
}  // namespace Config
