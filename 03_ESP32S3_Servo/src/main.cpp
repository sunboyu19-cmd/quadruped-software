#include <Arduino.h>
#include <SCServo.h>
#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"

#if !defined(CONFIG_IDF_TARGET_ESP32S3)
#error "Use ESP32-S3-WROOM-2-N32R16V for this project."
#endif

HardwareSerial servoUart(1);
SMS_STS servo;
int selected = Config::DEFAULT_ID;
bool armed = false;
bool offPending = false;
int center = 0, target = 0, minPosition = 0, maxPosition = 4095;
unsigned long lastCommand = 0, lastPoll = 0, lastOffRetry = 0;
int badReads = 0;

bool ping(int id) { return servo.Ping(id) == id; }
bool byteWrite(int id, int reg, int value) {
  servo.writeByte(id, reg, value);
  servoUart.flush();
  delay(3); // Allow optional write ACK to arrive; next READ discards it.
  return servo.readByte(id, reg) == value;
}
bool wordWrite(int id, int reg, int value) {
  servo.writeWord(id, reg, value);
  servoUart.flush();
  delay(3);
  return servo.readWord(id, reg) == value;
}

void disable(const char* reason) {
  armed = false;
  offPending = !byteWrite(selected, SMS_STS_TORQUE_ENABLE, 0);
  lastOffRetry = millis();
  Serial.printf("DISABLED id=%d reason=%s torque_off_verified=%d\n",
                selected, reason, !offPending);
  if (offPending) Serial.println("No confirmation: power off physically if necessary; retrying torque OFF.");
}

bool feedback(bool print) {
  if (servo.FeedBack(selected) < 0 || servo.Error != 0) {
    if (print) Serial.printf("id=%d FEEDBACK FAILED error=0x%02X; check power/jumper/baud/wiring/duplicate IDs\n", selected, servo.Error);
    return false;
  }
  const int pos = servo.ReadPos(-1);
  const float voltage = servo.ReadVoltage(-1) * 0.1f;
  const int temperature = servo.ReadTemper(-1);
  if (print) Serial.printf("id=%d armed=%d pos=%d deg=%.2f speed_raw=%d load_raw=%d voltage=%.1f temp=%d target=%d\n",
    selected, armed, pos, pos * 360.0f / 4096.0f, servo.ReadSpeed(-1),
    servo.ReadLoad(-1), voltage, temperature, target);
  return pos >= 0 && pos <= 4095 && voltage >= Config::MIN_VOLTAGE &&
         voltage <= Config::MAX_VOLTAGE && temperature < Config::MAX_TEMPERATURE;
}

bool sendPosition(int position) {
  servo.WritePosEx(selected, position, Config::SPEED_RAW, Config::ACC_RAW);
  servoUart.flush();
  delay(3);
  const bool ok = servo.readWord(selected, SMS_STS_GOAL_POSITION_L) == position;
  if (ok) target = position;
  return ok;
}

void arm() {
  if (armed) { Serial.println("Already armed."); return; }
  if (offPending) { Serial.println("BLOCKED: torque OFF not verified; use D first."); return; }
  if (!ping(selected) || !byteWrite(selected, SMS_STS_TORQUE_ENABLE, 0)) {
    disable("no servo / torque OFF not confirmed"); return;
  }
  if (servo.readByte(selected, SMS_STS_MODE) != 0) {
    Serial.println("BLOCKED: operating mode must be 0 (position servo); EEPROM mode not changed automatically."); return;
  }
  minPosition = servo.readWord(selected, SMS_STS_MIN_ANGLE_LIMIT_L);
  maxPosition = servo.readWord(selected, SMS_STS_MAX_ANGLE_LIMIT_L);
  if (!feedback(true) || minPosition < 0 || maxPosition > 4095 || minPosition >= maxPosition) {
    Serial.println("BLOCKED: feedback/power/temperature/position limits invalid."); return;
  }
  center = servo.ReadPos(-1);
  if (center < minPosition || center > maxPosition) {
    Serial.println("BLOCKED: current position outside servo EEPROM limits."); return;
  }
  if (!wordWrite(selected, SMS_STS_TORQUE_LIMIT_L, Config::TORQUE_LIMIT_RAW) ||
      !sendPosition(center) || !byteWrite(selected, SMS_STS_TORQUE_ENABLE, 1)) {
    disable("arm write/readback failed"); return;
  }
  armed = true;
  lastCommand = millis();
  badReads = 0;
  Serial.printf("ARMED id=%d center=%d; J 5 or J -5; D releases torque.\n", selected, center);
}

void moveRelative(float degrees) {
  if (!armed) { Serial.println("Send A first."); return; }
  if (!isfinite(degrees) || fabsf(degrees) > Config::MAX_STEP_DEG) {
    Serial.println("Rejected: each J step must be within -5..5 degrees."); return;
  }
  const int next = target + lroundf(degrees * 4096.0f / 360.0f);
  const int travel = lroundf(Config::MAX_TRAVEL_DEG * 4096.0f / 360.0f);
  if (next < minPosition || next > maxPosition || abs(next - center) > travel) {
    Serial.println("Rejected: exceeds arm-center +/-30deg or servo position limits; no wrap-around."); return;
  }
  if (!feedback(false) || !sendPosition(next)) { disable("move health/write failed"); return; }
  lastCommand = millis();
  Serial.printf("Goal id=%d pos=%d\n", selected, target);
}

void scan() {
  if (armed || offPending) { Serial.println("Use D and verify torque OFF before SCAN."); return; }
  Serial.println("Scanning IDs 0..253 at current baud; no broadcast. Same-ID duplicates cannot be identified.");
  int count = 0;
  for (int id = 0; id <= 253; ++id) {
    if (ping(id)) { ++count; Serial.printf("FOUND id=%d error=0x%02X\n", id, servo.Error); }
    delay(1);
  }
  Serial.printf("SCAN done: %d responding IDs. Select with I <id>.\n", count);
}

void changeId(int newId) {
  // User must physically connect exactly ONE servo. Scanning cannot detect duplicates.
  if (armed || offPending) { Serial.println("Use D and verify torque OFF before changing ID."); return; }
  if (newId < 0 || newId > 253 || newId == selected) {
    Serial.println("Rejected new ID (0..253, different from current)."); return;
  }
  if (!ping(selected) || ping(newId)) { Serial.println("Old ID missing or new ID occupied."); return; }
  if (!byteWrite(selected, SMS_STS_TORQUE_ENABLE, 0) ||
      !byteWrite(selected, SMS_STS_LOCK, 0)) {
    servo.LockEprom(selected);
    Serial.println("ID change aborted; torque/unlock failed."); return;
  }
  servo.writeByte(selected, SMS_STS_ID, newId);
  delay(30);
  const int oldId = selected;
  if (ping(newId) && servo.readByte(newId, SMS_STS_ID) == newId) {
    selected = newId;
    const bool locked = byteWrite(selected, SMS_STS_LOCK, 1);
    Serial.printf("ID changed %d -> %d; lock_verified=%d. Power-cycle and scan to verify persistence.\n", oldId, selected, locked);
    if (!locked) Serial.println("EEPROM lock unconfirmed: power off and check before further use.");
  } else {
    servo.LockEprom(oldId);
    servo.LockEprom(newId);
    Serial.println("ID change unconfirmed. Power-cycle with ONE servo and SCAN; do not repeat blindly.");
  }
}

void help() {
  Serial.println("H help | SCAN read-only ID scan | I 1 select ID | S status | PING");
  Serial.println("A hold CURRENT position | J 5 / J -5 jog degrees | D torque OFF | K keepalive");
  Serial.println("ID 2 SINGLE = persist selected servo's ID as 2; physically connect ONE servo only.");
  Serial.println("B 1000000 = host UART baud only (38400/57600/76800/115200/128000/250000/500000/1000000)");
  Serial.println("Enter one command per line. 30s without J/K -> torque OFF; support the mechanism.");
}

void command(char* line) {
  char* tokens[5] = {};
  int n = 0;
  for (char* p = strtok(line, " \t"); p && n < 5; p = strtok(nullptr, " \t")) tokens[n++] = p;
  if (!n) return;
  for (char* p = tokens[0]; *p; ++p) *p = toupper(static_cast<unsigned char>(*p));
  const char* c = tokens[0];
  if (n == 1) {
    if (!strcmp(c, "H")) { help(); return; }
    if (!strcmp(c, "S")) { feedback(true); return; }
    if (!strcmp(c, "SCAN")) { scan(); return; }
    if (!strcmp(c, "PING")) { Serial.printf("PING id=%d ok=%d\n", selected, ping(selected)); return; }
    if (!strcmp(c, "A")) { arm(); return; }
    if (!strcmp(c, "D")) { disable("user"); return; }
    if (!strcmp(c, "K")) { lastCommand = millis(); return; }
  }
  if (n == 2 && !strcmp(c, "J")) {
    char* end = nullptr;
    const float value = strtof(tokens[1], &end);
    if (end != tokens[1] && !*end) { moveRelative(value); return; }
  }
  if (n >= 2) {
    char* end = nullptr;
    const long v = strtol(tokens[1], &end, 10);
    if (end != tokens[1] && !*end) {
      if (n == 2 && !strcmp(c, "I") && v >= 0 && v <= 253) {
        if (armed || offPending) Serial.println("Use D and verify torque OFF before selecting another servo.");
        else { selected = v; Serial.printf("Selected id=%d (no motion command sent).\n", selected); }
        return;
      }
      if (n == 3 && !strcmp(c, "ID") && !strcmp(tokens[2], "SINGLE") && v >= 0 && v <= 253) {
        changeId(v); return;
      }
      if (n == 2 && !strcmp(c, "B")) {
        const long rates[] = {38400, 57600, 76800, 115200, 128000, 250000, 500000, 1000000};
        if (armed || offPending) { Serial.println("Use D before baud change."); return; }
        for (long rate : rates) if (v == rate) {
          servoUart.flush(); servoUart.updateBaudRate(rate);
          Serial.printf("Host bus baud=%ld; servo EEPROM baud unchanged.\n", rate); return;
        }
      }
    }
  }
  Serial.println("Invalid command/arguments; H for help.");
}

void setup() {
  // Keep the previous wheel-driver EN inactive if accidentally still connected.
  pinMode(7, OUTPUT); digitalWrite(7, LOW);
  Serial.begin(115200);
  servoUart.begin(Config::BAUD, SERIAL_8N1, Config::UART_RX, Config::UART_TX);
  servo.pSerial = &servoUart;
  servo.IOTimeOut = 8;
  // Accept reply-level 0 and 1. All critical writes are verified with READ.
  servo.Level = 0;
  Serial.println("S3 + Waveshare Bus Servo Adapter(A) + Feetech STS3032; console=native USB");
  Serial.println("Boot: no motion/EEPROM/torque-enable commands sent. First connect ONE servo.");
  help();
}

void loop() {
  static char buffer[80];
  static size_t used = 0;
  static bool overflow = false;
  for (int budget = 0; budget < 80 && Serial.available(); ++budget) {
    const char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (!overflow && used) { buffer[used] = 0; command(buffer); }
      if (overflow) Serial.println("Command too long; discarded.");
      used = 0; overflow = false;
    } else if (!overflow) {
      if (used < sizeof(buffer) - 1) buffer[used++] = c;
      else { used = 0; overflow = true; }
    }
  }
  if (offPending && millis() - lastOffRetry >= 500) disable("retry torque OFF");
  if (armed && millis() - lastCommand >= Config::TIMEOUT_MS) disable("30s without J/K");
  if (millis() - lastPoll >= 500) {
    lastPoll = millis();
    if (armed) {
      if (feedback(true)) badReads = 0;
      else if (++badReads >= 2) disable("feedback/voltage/temperature failure");
    } else Serial.printf("READY selected=%d armed=0 off_pending=%d; H help, SCAN find IDs\n", selected, offPending);
  }
  delay(1);
}
