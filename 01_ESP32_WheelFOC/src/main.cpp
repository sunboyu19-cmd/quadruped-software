#include <Arduino.h>
#include <SimpleFOC.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include "config.h"

#ifndef FOC_ENABLE_MOTOR
#define FOC_ENABLE_MOTOR 0
#endif

static_assert(Config::POLE_PAIRS > 0, "Pole pairs must be positive");
static_assert(Config::SUPPLY_VOLTAGE > 0, "Measure and configure supply voltage");
static_assert(Config::ALIGN_VOLTAGE > 0 &&
              Config::ALIGN_VOLTAGE <= Config::MOTOR_VOLTAGE_LIMIT,
              "Alignment voltage must stay within motor voltage limit");
static_assert(Config::MOTOR_VOLTAGE_LIMIT <= Config::SUPPLY_VOLTAGE / 2,
              "Keep initial voltage limit within SinePWM range");

MagneticSensorI2C sensor(AS5600_I2C);
BLDCMotor motor(Config::POLE_PAIRS);
BLDCDriver3PWM driver(Config::PWM_A, Config::PWM_B, Config::PWM_C,
                     Config::ENABLE_PIN);
bool armed = false;
bool driverReady = false;
bool sensorReady = false;
float target = 0;
unsigned long lastCommand = 0;
unsigned long lastHealth = 0;
unsigned long lastPrint = 0;
unsigned int badReads = 0;
uint8_t magnetStatus = 0;

bool readRegister(uint8_t reg, uint8_t& value) {
  Wire.beginTransmission(0x36);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(uint8_t(0x36), uint8_t(1)) != 1) return false;
  value = Wire.read();
  return true;
}

bool magnetHealthy() {
  // AS5600 STATUS: MD bit5, ML bit4, MH bit3.
  return readRegister(0x0B, magnetStatus) && (magnetStatus & 0x20) &&
         !(magnetStatus & 0x18);
}

bool driverFault() {
  return Config::FAULT_PIN >= 0 && digitalRead(Config::FAULT_PIN) == LOW;
}

void disableMotor(const char* reason) {
  target = 0;
  armed = false;
  if (driverReady) motor.disable();
  digitalWrite(Config::ENABLE_PIN, LOW);
  Serial.print("DISABLED: ");
  Serial.println(reason);
}

void printHelp() {
  Serial.println("A = align + arm (wheel MOVES during alignment)");
  Serial.println("V1 / V-1 = target rad/s; V0 = active hold at zero speed");
  Serial.println("D = disable/freewheel; K = refresh 10s command timeout");
  Serial.println("S = status; H = help; send each command with newline");
}

void printStatus() {
  Serial.printf("mode=%s en=%d mag=%02X a=%.3f w=%.3f t=%.2f uq=%.2f\n",
                FOC_ENABLE_MOTOR ? "motor" : "sensor", armed, magnetStatus,
                sensorReady ? sensor.getAngle() : 0.0f,
                sensorReady ? sensor.getVelocity() : 0.0f, target,
                armed ? motor.voltage.q : 0.0f);
}

void armMotor() {
  if (!FOC_ENABLE_MOTOR) {
    Serial.println("Sensor-only firmware: build/upload motor_test to run motor.");
    return;
  }
  if (!Config::DRIVER_WIRING_AND_POWER_CONFIRMED) {
    Serial.println("BLOCKED: verify driver 3PWM/EN and VM supply, then set confirmation in config.h.");
    return;
  }
  if (armed) { Serial.println("Already armed."); return; }
  if (!sensorReady || !magnetHealthy() || driverFault()) {
    Serial.println("BLOCKED: AS5600/magnet or driver fault. Use sensor_test first.");
    return;
  }
  if (!driverReady) {
    driver.voltage_power_supply = Config::SUPPLY_VOLTAGE;
    driver.voltage_limit = Config::SUPPLY_VOLTAGE;
    driver.pwm_frequency = 25000;
    if (!driver.init()) { disableMotor("driver init failed"); return; }
    motor.linkDriver(&driver);
    motor.linkSensor(&sensor);
    motor.controller = MotionControlType::velocity;
    motor.torque_controller = TorqueControlType::voltage;
    motor.voltage_limit = Config::MOTOR_VOLTAGE_LIMIT;
    motor.voltage_sensor_align = Config::ALIGN_VOLTAGE;
    motor.velocity_limit = Config::MAX_TARGET_RAD_S;
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 1.0f;
    motor.PID_velocity.D = 0;
    motor.PID_velocity.output_ramp = 10;
    motor.LPF_velocity.Tf = 0.02f;
    driverReady = true;
    motor.init();  // SimpleFOC 2.3.3 returns void; inspect motor_status instead.
    if (motor.motor_status != FOCMotorStatus::motor_uncalibrated) {
      disableMotor("motor init failed"); return;
    }
    motor.disable();
  }
  Serial.println("ALIGNING: wheel must be unloaded. This blocks commands for several seconds.");
  // Recalibrate after any disable/fault; no stored angle offset assumptions.
  motor.sensor_direction = Direction::UNKNOWN;
  motor.zero_electric_angle = NOT_SET;
  motor.enable();
  if (!motor.initFOC() || !magnetHealthy() || driverFault()) {
    disableMotor("alignment failed: inspect magnet, phases, pole pairs, supply");
    return;
  }
  motor.PID_velocity.reset();
  target = 0;
  armed = true;
  lastCommand = millis();
  badReads = 0;
  Serial.println("ARMED at zero speed. Send V1 for a slow test, D to disable.");
}

void command(char* line) {
  while (isspace(static_cast<unsigned char>(*line))) ++line;
  if (!*line) return;
  const char code = toupper(static_cast<unsigned char>(*line++));
  if (code == 'V') {
    char* end = nullptr;
    const float value = strtof(line, &end);
    if (end == line) { Serial.println("Invalid velocity"); return; }
    while (isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end || !isfinite(value) || fabsf(value) > Config::MAX_TARGET_RAD_S) {
      Serial.println("Invalid/out-of-range target; allowed -5..5 rad/s"); return;
    }
    if (!armed) { Serial.println("Not armed. Send A first."); return; }
    target = value;
    lastCommand = millis();
    Serial.printf("Target %.2f rad/s\n", target);
    return;
  }
  while (isspace(static_cast<unsigned char>(*line))) ++line;
  if (*line) { Serial.println("Unexpected command arguments"); return; }
  switch (code) {
    case 'A': armMotor(); break;
    case 'D': disableMotor("user command"); break;
    case 'K': lastCommand = millis(); break;
    case 'S': printStatus(); break;
    case 'H': printHelp(); break;
    default: Serial.println("Unknown command; send H"); break;
  }
}

void serialCommands() {
  static char buffer[48];
  static size_t used = 0;
  static bool overflow = false;
  // Bound parsing work to avoid starving FOC if serial input is flooded.
  for (int budget = 0; budget < 48 && Serial.available(); ++budget) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!overflow && used) { buffer[used] = '\0'; command(buffer); }
      if (overflow) Serial.println("Command too long; discarded");
      used = 0; overflow = false;
    } else if (!overflow) {
      if (used < sizeof(buffer) - 1) buffer[used++] = c;
      else { overflow = true; used = 0; }
    }
  }
}

void setup() {
  pinMode(Config::ENABLE_PIN, OUTPUT);
  digitalWrite(Config::ENABLE_PIN, LOW);
  if (Config::FAULT_PIN >= 0) pinMode(Config::FAULT_PIN, INPUT_PULLUP);
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);
  Wire.begin(Config::SDA_PIN, Config::SCL_PIN);
  Wire.setClock(400000);
  Wire.setTimeOut(5);
  uint8_t unused;
  if (readRegister(0x0B, unused)) {
    sensor.init(&Wire);
    sensorReady = true;
    magnetHealthy();
  } else Serial.println("AS5600 not found at 0x36: check 3.3V, GND, SDA21, SCL22.");
  Serial.println("Single wheel FOC. Outputs disabled at boot. Restart after fixing missing sensor.");
  Serial.println("Telemetry: a=angle(rad), w=velocity(rad/s), t=target(rad/s), uq=voltage command(V).");
  printHelp();
}

void loop() {
  if (armed) {
    motor.loopFOC();
    motor.move(target);
    if (!isfinite(motor.shaft_velocity) || fabsf(motor.shaft_velocity) > Config::OVERSPEED_RAD_S)
      disableMotor("overspeed / invalid sensor velocity");
    if (driverFault()) disableMotor("driver nFAULT");
    if (armed && millis() - lastCommand > Config::COMMAND_TIMEOUT_MS)
      disableMotor("10s without V or K command");
  } else if (sensorReady) sensor.update();
  if (millis() - lastHealth >= 20) {
    lastHealth = millis();
    if (magnetHealthy()) badReads = 0;
    else if (++badReads >= 3 && armed) disableMotor("AS5600/magnet health failed");
  }
  serialCommands();
  if (millis() - lastPrint >= 250) {
    lastPrint = millis();
    if (Serial.availableForWrite() >= 96) printStatus();
  }
}
