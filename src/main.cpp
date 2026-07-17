#include <Arduino.h>
#include <Wire.h>

#include "bq25186_minimal.h"
#include "i2c_devices.h"
#include "ina228_helpers.h"
#include "pins_nautilus.h"
#include "power_pwm.h"

namespace {
constexpr const char* kFirmwareVersion = "NAUTILUS bring-up 0.1.0";
constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kI2cClockHz = 100000;
constexpr uint32_t kLogIntervalMs = 2000;
constexpr uint32_t kDualControlIntervalMs = 200;
constexpr float kDualInputMinV = 0.8f;
constexpr float kDualInputMaxV = 3.5f;
constexpr float kDualBoostTargetV = 8.0f;
constexpr float kDualBuckTargetV = 5.0f;
constexpr float kDualBoostStartDuty = 5.0f;
constexpr float kDualBoostMaxDuty = 30.0f;
constexpr float kDualBuckStartDuty = 10.0f;
constexpr float kDualBuckMaxDuty = 85.0f;
constexpr float kDualBuckOverVoltageV = 5.75f;
constexpr float kDualInputCurrentLimitA = 2.5f;
constexpr float kDualBoostRampStepPercent = 2.0f;
constexpr float kDualBuckKpPercentPerVolt = 4.0f;

BQ25186 bq(Wire, I2CDevices::ADDR_BQ25186);
INA228Device inaBoost("Boost/input", I2CDevices::ADDR_INA228_BOOST, Wire);
INA228Device inaBuck("Buck/output", I2CDevices::ADDR_INA228_BUCK, Wire);
PowerPwm powerPwm;

bool logEnabled = false;
bool sensorsStarted = false;
bool inaInitialised = false;
uint32_t lastLogMs = 0;

struct DualControlState {
  bool active = false;
  float boostDuty = 0.0f;
  float boostDutyTarget = 0.0f;
  float buckDuty = 0.0f;
  uint32_t lastTickMs = 0;
};

DualControlState dualControl;

bool i2cProbe(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void printHexAddress(uint8_t address) {
  Serial.print(F("0x"));
  if (address < 16) Serial.print('0');
  Serial.print(address, HEX);
}

uint8_t sensirionCrc(const uint8_t* data, size_t len) {
  uint8_t crc = 0xFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31) : static_cast<uint8_t>(crc << 1);
    }
  }
  return crc;
}

bool writeSensirionCommand(uint8_t address, uint16_t command) {
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(command >> 8));
  Wire.write(static_cast<uint8_t>(command & 0xFF));
  return Wire.endTransmission() == 0;
}

bool writeSensirionCommandWithWord(uint8_t address, uint16_t command, uint16_t word) {
  const uint8_t data[2] = {static_cast<uint8_t>(word >> 8), static_cast<uint8_t>(word & 0xFF)};
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(command >> 8));
  Wire.write(static_cast<uint8_t>(command & 0xFF));
  Wire.write(data[0]);
  Wire.write(data[1]);
  Wire.write(sensirionCrc(data, 2));
  return Wire.endTransmission() == 0;
}

bool readSensirionWords(uint8_t address, uint16_t* words, size_t count) {
  const int bytesExpected = static_cast<int>(count * 3);
  if (Wire.requestFrom(static_cast<int>(address), bytesExpected) != bytesExpected) {
    return false;
  }
  for (size_t i = 0; i < count; ++i) {
    const uint8_t data[2] = {static_cast<uint8_t>(Wire.read()), static_cast<uint8_t>(Wire.read())};
    const uint8_t crc = Wire.read();
    if (sensirionCrc(data, 2) != crc) {
      return false;
    }
    words[i] = static_cast<uint16_t>((data[0] << 8) | data[1]);
  }
  return true;
}

float wordsToFloat(uint16_t msw, uint16_t lsw) {
  const uint32_t raw = (static_cast<uint32_t>(msw) << 16) | lsw;
  float value = 0.0f;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

bool scd30Init() {
  if (!writeSensirionCommandWithWord(I2CDevices::ADDR_SCD30, 0x4600, 2)) {
    return false;
  }
  delay(10);
  return writeSensirionCommandWithWord(I2CDevices::ADDR_SCD30, 0x0010, 0);
}

bool scd30Read(Stream& out) {
  if (!writeSensirionCommand(I2CDevices::ADDR_SCD30, 0x0202)) {
    out.println(F("SCD30 data-ready command failed"));
    return false;
  }
  delay(4);
  uint16_t ready = 0;
  if (!readSensirionWords(I2CDevices::ADDR_SCD30, &ready, 1) || ready == 0) {
    out.println(F("SCD30 data not ready"));
    return false;
  }

  if (!writeSensirionCommand(I2CDevices::ADDR_SCD30, 0x0300)) {
    out.println(F("SCD30 read command failed"));
    return false;
  }
  delay(4);
  uint16_t words[6] = {};
  if (!readSensirionWords(I2CDevices::ADDR_SCD30, words, 6)) {
    out.println(F("SCD30 read/CRC failed"));
    return false;
  }

  out.print(F("SCD30: CO2="));
  out.print(wordsToFloat(words[0], words[1]), 1);
  out.print(F(" ppm, T="));
  out.print(wordsToFloat(words[2], words[3]), 2);
  out.print(F(" C, RH="));
  out.print(wordsToFloat(words[4], words[5]), 2);
  out.println(F(" %"));
  return true;
}

bool sps30Init() {
  return writeSensirionCommandWithWord(I2CDevices::ADDR_SPS30, 0x0010, 0x0300);
}

bool sps30Read(Stream& out) {
  if (!writeSensirionCommand(I2CDevices::ADDR_SPS30, 0x0202)) {
    out.println(F("SPS30 data-ready command failed"));
    return false;
  }
  delay(5);
  uint16_t ready = 0;
  if (!readSensirionWords(I2CDevices::ADDR_SPS30, &ready, 1) || ready == 0) {
    out.println(F("SPS30 data not ready"));
    return false;
  }

  if (!writeSensirionCommand(I2CDevices::ADDR_SPS30, 0x0300)) {
    out.println(F("SPS30 read command failed"));
    return false;
  }
  delay(10);
  uint16_t words[20] = {};
  if (!readSensirionWords(I2CDevices::ADDR_SPS30, words, 20)) {
    out.println(F("SPS30 read/CRC failed"));
    return false;
  }

  out.print(F("SPS30 mass: PM1.0="));
  out.print(wordsToFloat(words[0], words[1]), 2);
  out.print(F(" PM2.5="));
  out.print(wordsToFloat(words[2], words[3]), 2);
  out.print(F(" PM4="));
  out.print(wordsToFloat(words[4], words[5]), 2);
  out.print(F(" PM10="));
  out.print(wordsToFloat(words[6], words[7]), 2);
  out.println(F(" ug/m3"));

  out.print(F("SPS30 count: PM0.5="));
  out.print(wordsToFloat(words[8], words[9]), 2);
  out.print(F(" PM1.0="));
  out.print(wordsToFloat(words[10], words[11]), 2);
  out.print(F(" PM2.5="));
  out.print(wordsToFloat(words[12], words[13]), 2);
  out.print(F(" PM4="));
  out.print(wordsToFloat(words[14], words[15]), 2);
  out.print(F(" PM10="));
  out.print(wordsToFloat(words[16], words[17]), 2);
  out.print(F(" typical="));
  out.print(wordsToFloat(words[18], words[19]), 2);
  out.println(F(" um"));
  return true;
}

void printFlame(Stream& out) {
  out.print(F("Flame sensor: AOUT raw="));
  out.print(analogRead(Pins::PIN_WS_AOUT));
  out.print(F(", DOUT="));
  out.println(digitalRead(Pins::PIN_WS_DOUT));
}

String nextToken(String& line) {
  line.trim();
  const int split = line.indexOf(' ');
  if (split < 0) {
    const String token = line;
    line = "";
    return token;
  }
  const String token = line.substring(0, split);
  line = line.substring(split + 1);
  return token;
}

bool parseFloatToken(const String& token, float& value) {
  if (token.length() == 0) {
    return false;
  }
  char* end = nullptr;
  value = strtof(token.c_str(), &end);
  return end != token.c_str() && *end == '\0' && isfinite(value);
}

bool parseUInt16Token(const String& token, uint16_t& value) {
  if (token.length() == 0) {
    return false;
  }
  char* end = nullptr;
  const unsigned long parsed = strtoul(token.c_str(), &end, 10);
  if (end == token.c_str() || *end != '\0' || parsed > UINT16_MAX) {
    return false;
  }
  value = static_cast<uint16_t>(parsed);
  return true;
}

float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

void setPowerPin(gpio_num_t pin, bool on) {
  digitalWrite(pin, on ? HIGH : LOW);
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  help"));
  Serial.println(F("  pins"));
  Serial.println(F("  power sensor|radio|5v on|off"));
  Serial.println(F("  scan | scan all"));
  Serial.println(F("  ina init | ina read"));
  Serial.println(F("  bq regs | bq profile liion | bq current <mA> | bq charge on|off | bq status"));
  Serial.println(F("  pwm boost|buck <duty_percent>|enable|disable"));
  Serial.println(F("  pwm enable boost|buck"));
  Serial.println(F("  pwm disable boost|buck|all"));
  Serial.println(F("  dual start|stop|status"));
  Serial.println(F("  sensors init | sensors read"));
  Serial.println(F("  log on|off"));
  Serial.println(F("  faults clear"));
}

void printPins() {
  Serial.println(F("Pin map/state:"));
  Serial.printf("  SENSOR_EN GPIO%d state=%d\n", Pins::PIN_SENSOR_EN, digitalRead(Pins::PIN_SENSOR_EN));
  Serial.printf("  RADIO_EN  GPIO%d state=%d\n", Pins::PIN_RADIO_EN, digitalRead(Pins::PIN_RADIO_EN));
  Serial.printf("  5V_EN     GPIO%d state=%d\n", Pins::PIN_5V_EN, digitalRead(Pins::PIN_5V_EN));
  Serial.printf("  I2C SDA/SCL GPIO%d/GPIO%d @ %lu Hz\n", Pins::PIN_I2C_SDA, Pins::PIN_I2C_SCL, kI2cClockHz);
  Serial.printf("  BOOST LO/HI GPIO%d/GPIO%d\n", Pins::PIN_BOOST_LO_PWM, Pins::PIN_BOOST_HI_PWM);
  Serial.printf("  BUCK  LO/HI GPIO%d/GPIO%d\n", Pins::PIN_BUCK_LO_PWM, Pins::PIN_BUCK_HI_PWM);
  Serial.printf("  BQ /CE GPIO%d state=%d, /PG GPIO%d state=%d, /INT GPIO%d state=%d\n",
                Pins::PIN_BQ_CE_N, digitalRead(Pins::PIN_BQ_CE_N), Pins::PIN_BQ_PG_N,
                digitalRead(Pins::PIN_BQ_PG_N), Pins::PIN_BQ_INT, digitalRead(Pins::PIN_BQ_INT));
  Serial.printf("  WS AOUT GPIO%d, WS DOUT GPIO%d state=%d\n", Pins::PIN_WS_AOUT, Pins::PIN_WS_DOUT,
                digitalRead(Pins::PIN_WS_DOUT));
  powerPwm.printStatus(Serial);
}

void scanExpected() {
  Serial.println(F("Expected I2C devices:"));
  size_t found = 0;
  for (size_t i = 0; i < I2CDevices::EXPECTED_COUNT; ++i) {
    const auto& device = I2CDevices::EXPECTED[i];
    const bool ok = i2cProbe(device.address);
    if (ok) ++found;
    Serial.print(F("  "));
    printHexAddress(device.address);
    Serial.print(F(" "));
    Serial.print(device.name);
    Serial.print(F(" - "));
    Serial.print(device.function);
    Serial.print(F(": "));
    Serial.println(ok ? F("FOUND") : F("MISSING"));
  }
  Serial.print(F("Expected found: "));
  Serial.print(found);
  Serial.print(F("/"));
  Serial.println(I2CDevices::EXPECTED_COUNT);
}

void scanAll() {
  Serial.println(F("All responding 7-bit I2C addresses:"));
  for (uint8_t address = 1; address < 0x7F; ++address) {
    if (i2cProbe(address)) {
      Serial.print(F("  "));
      printHexAddress(address);
      Serial.println();
    }
  }
}

void runSensorInit() {
  setPowerPin(Pins::PIN_SENSOR_EN, true);
  setPowerPin(Pins::PIN_5V_EN, true);
  delay(500);
  const bool scdOk = scd30Init();
  const bool spsOk = sps30Init();
  sensorsStarted = scdOk && spsOk;
  Serial.print(F("SCD30 init: "));
  Serial.println(scdOk ? F("OK") : F("FAILED"));
  Serial.print(F("SPS30 init: "));
  Serial.println(spsOk ? F("OK") : F("FAILED"));
}

void runSensorRead() {
  if (!sensorsStarted) {
    Serial.println(F("Sensors were not initialised; run 'sensors init' first"));
  }
  scd30Read(Serial);
  sps30Read(Serial);
  printFlame(Serial);
}

void runLogTick() {
  if (!logEnabled || millis() - lastLogMs < kLogIntervalMs) {
    return;
  }
  lastLogMs = millis();
  Serial.println(F("--- periodic log ---"));
  inaBoost.printSample(Serial);
  inaBuck.printSample(Serial);
  bq.printStatus(Serial, digitalRead(Pins::PIN_BQ_PG_N), digitalRead(Pins::PIN_BQ_INT));
  runSensorRead();
}

bool initialiseInas(Stream& out) {
  const bool okBoost = inaBoost.begin(0.05f, 3.0f, out);
  const bool okBuck = inaBuck.begin(0.05f, 3.0f, out);
  inaInitialised = okBoost && okBuck;
  if (!inaInitialised) {
    dualControl.active = false;
    powerPwm.disableAll();
    out.println(F("INA init problem; PWM forced disabled"));
  }
  return inaInitialised;
}

float idealBoostDutyPercent(float inputVoltageV, float targetVoltageV) {
  if (inputVoltageV <= 0.0f || targetVoltageV <= inputVoltageV) {
    return 0.0f;
  }
  return 100.0f * (1.0f - (inputVoltageV / targetVoltageV));
}

void stopDualControl(const __FlashStringHelper* reason, Stream& out) {
  dualControl.active = false;
  powerPwm.disableAll();
  out.print(F("Dual-stage control stopped: "));
  out.println(reason);
}

void printDualStatus(Stream& out) {
  out.print(F("Dual control: "));
  out.print(dualControl.active ? F("active") : F("inactive"));
  out.print(F(", boost duty="));
  out.print(dualControl.boostDuty, 2);
  out.print(F("% target="));
  out.print(dualControl.boostDutyTarget, 2);
  out.print(F("%, buck duty="));
  out.print(dualControl.buckDuty, 2);
  out.println(F("%"));
}

void startDualControl(Stream& out) {
  if (!inaInitialised && !initialiseInas(out)) {
    return;
  }

  const Ina228Sample boostSample = inaBoost.readSample();
  if (!boostSample.ok) {
    stopDualControl(F("boost/input INA read failed"), out);
    return;
  }

  const float inputV = boostSample.busVoltageV;
  if (inputV < kDualInputMinV || inputV > kDualInputMaxV) {
    out.print(F("Refusing dual start; boost/input voltage is "));
    out.print(inputV, 3);
    out.print(F(" V, expected "));
    out.print(kDualInputMinV, 1);
    out.print(F(".."));
    out.print(kDualInputMaxV, 1);
    out.println(F(" V"));
    return;
  }

  float boostTargetDuty = idealBoostDutyPercent(inputV, kDualBoostTargetV);
  if (boostTargetDuty > kDualBoostMaxDuty) {
    out.print(F("Requested 8 V boost duty is "));
    out.print(boostTargetDuty, 1);
    out.print(F("%; clamping boost to safe test limit "));
    out.print(kDualBoostMaxDuty, 1);
    out.println(F("%"));
    boostTargetDuty = kDualBoostMaxDuty;
  }
  boostTargetDuty = clampFloat(boostTargetDuty, kDualBoostStartDuty, kDualBoostMaxDuty);

  dualControl.active = false;
  powerPwm.disableAll();
  dualControl.boostDuty = kDualBoostStartDuty;
  dualControl.boostDutyTarget = boostTargetDuty;
  dualControl.buckDuty = kDualBuckStartDuty;
  dualControl.lastTickMs = millis();

  if (!powerPwm.setBoostDutyForControl(dualControl.boostDuty, out) ||
      !powerPwm.setBuckDutyForControl(dualControl.buckDuty, out) ||
      !powerPwm.enableDualForControl(out)) {
    dualControl.active = false;
    powerPwm.disableAll();
    out.println(F("Dual-stage control failed to start"));
    return;
  }

  dualControl.active = true;
  out.print(F("Dual-stage control started. Vin="));
  out.print(inputV, 3);
  out.print(F(" V, open-loop boost target duty="));
  out.print(dualControl.boostDutyTarget, 2);
  out.println(F("%, buck target=5.00 V"));
  out.println(F("Watch boost node, buck output, input current, and temperature. 'dual stop' disables both stages."));
}

void runDualControlTick() {
  if (!dualControl.active || millis() - dualControl.lastTickMs < kDualControlIntervalMs) {
    return;
  }
  dualControl.lastTickMs = millis();

  const Ina228Sample boostSample = inaBoost.readSample();
  const Ina228Sample buckSample = inaBuck.readSample();
  if (!boostSample.ok || !buckSample.ok) {
    stopDualControl(F("INA read failed"), Serial);
    return;
  }
  if (fabsf(boostSample.currentA) > kDualInputCurrentLimitA) {
    stopDualControl(F("input current limit exceeded"), Serial);
    return;
  }
  if (buckSample.busVoltageV > kDualBuckOverVoltageV) {
    stopDualControl(F("buck output overvoltage"), Serial);
    return;
  }

  if (dualControl.boostDuty < dualControl.boostDutyTarget) {
    dualControl.boostDuty = min(dualControl.boostDuty + kDualBoostRampStepPercent,
                                dualControl.boostDutyTarget);
    if (!powerPwm.setBoostDutyForControl(dualControl.boostDuty, Serial)) {
      dualControl.active = false;
      return;
    }
  }

  const float buckErrorV = kDualBuckTargetV - buckSample.busVoltageV;
  dualControl.buckDuty += kDualBuckKpPercentPerVolt * buckErrorV;
  dualControl.buckDuty = clampFloat(dualControl.buckDuty, 0.0f, kDualBuckMaxDuty);
  if (!powerPwm.setBuckDutyForControl(dualControl.buckDuty, Serial)) {
    dualControl.active = false;
    return;
  }

  Serial.print(F("dual: Vin="));
  Serial.print(boostSample.busVoltageV, 3);
  Serial.print(F(" V Iin="));
  Serial.print(boostSample.currentA, 3);
  Serial.print(F(" A boostD="));
  Serial.print(dualControl.boostDuty, 1);
  Serial.print(F("% Vout="));
  Serial.print(buckSample.busVoltageV, 3);
  Serial.print(F(" V buckD="));
  Serial.print(dualControl.buckDuty, 1);
  Serial.println(F("%"));
}

void handlePower(String args) {
  const String rail = nextToken(args);
  const String state = nextToken(args);
  if ((state != F("on")) && (state != F("off"))) {
    Serial.println(F("Usage: power sensor|radio|5v on|off"));
    return;
  }
  const bool on = state == F("on");
  if (rail == F("sensor")) {
    setPowerPin(Pins::PIN_SENSOR_EN, on);
  } else if (rail == F("radio")) {
    setPowerPin(Pins::PIN_RADIO_EN, on);
  } else if (rail == F("5v")) {
    setPowerPin(Pins::PIN_5V_EN, on);
  } else {
    Serial.println(F("Unknown power rail"));
    return;
  }
  Serial.print(F("Power "));
  Serial.print(rail);
  Serial.print(F(" "));
  Serial.println(on ? F("on") : F("off"));
}

void handleIna(String args) {
  const String sub = nextToken(args);
  if (sub == F("init")) {
    initialiseInas(Serial);
  } else if (sub == F("read")) {
    inaBoost.printSample(Serial);
    inaBuck.printSample(Serial);
  } else {
    Serial.println(F("Usage: ina init|read"));
  }
}

void handleBq(String args) {
  const String sub = nextToken(args);
  if (sub == F("regs")) {
    if (!bq.dumpRegisters(Serial)) {
      dualControl.active = false;
      powerPwm.disableAll();
      Serial.println(F("BQ register read failed; PWM forced disabled"));
    }
  } else if (sub == F("profile")) {
    if (nextToken(args) != F("liion")) {
      Serial.println(F("Usage: bq profile liion"));
      return;
    }
    digitalWrite(Pins::PIN_BQ_CE_N, HIGH);
    Serial.println(bq.setLiIon420Profile() ? F("BQ Li-ion 4.20 V profile written; charge still disabled")
                                           : F("BQ profile write failed"));
  } else if (sub == F("current")) {
    uint16_t ma = 0;
    if (!parseUInt16Token(nextToken(args), ma) || ma == 0) {
      Serial.println(F("Usage: bq current <mA>"));
      return;
    }
    const uint16_t clamped = min<uint16_t>(ma, BQ25186::SAFE_MAX_CHARGE_MA);
    if (bq.setChargeCurrentMilliAmps(clamped)) {
      Serial.print(F("BQ charge current set/requested to "));
      Serial.print(clamped);
      Serial.println(F(" mA"));
    } else {
      Serial.println(F("BQ current write failed"));
    }
  } else if (sub == F("charge")) {
    const String state = nextToken(args);
    if ((state != F("on")) && (state != F("off"))) {
      Serial.println(F("Usage: bq charge on|off"));
      return;
    }
    const bool enable = state == F("on");
    if (!enable) {
      digitalWrite(Pins::PIN_BQ_CE_N, HIGH);
    }
    const bool i2cOk = bq.setChargeEnabled(enable);
    if (enable && i2cOk) {
      digitalWrite(Pins::PIN_BQ_CE_N, LOW);
    }
    if (i2cOk) {
      Serial.println(enable ? F("BQ charging enabled") : F("BQ charging disabled"));
    } else {
      Serial.println(F("BQ charge enable register write failed"));
    }
  } else if (sub == F("status")) {
    bq.printStatus(Serial, digitalRead(Pins::PIN_BQ_PG_N), digitalRead(Pins::PIN_BQ_INT));
  } else {
    Serial.println(F("Usage: bq regs|profile liion|current <mA>|charge on|off|status"));
  }
}

void handlePwm(String args) {
  if (dualControl.active) {
    stopDualControl(F("manual PWM command received"), Serial);
  }
  const String sub = nextToken(args);
  if (sub == F("boost") || sub == F("buck")) {
    const String dutyToken = nextToken(args);
    if (dutyToken == F("enable")) {
      powerPwm.enable(sub, Serial);
      return;
    }
    if (dutyToken == F("disable")) {
      powerPwm.disable(sub, Serial);
      return;
    }
    float duty = 0.0f;
    if (!parseFloatToken(dutyToken, duty)) {
      powerPwm.disable(F("all"), Serial);
      dualControl.active = false;
      Serial.println(F("Usage: pwm boost|buck <duty_percent>|enable|disable"));
      return;
    }
    powerPwm.setDuty(sub, duty, Serial);
  } else if (sub == F("enable")) {
    powerPwm.enable(nextToken(args), Serial);
  } else if (sub == F("disable")) {
    powerPwm.disable(nextToken(args), Serial);
  } else {
    Serial.println(F("Usage: pwm boost|buck <duty>|enable|disable | pwm enable boost|buck | pwm disable boost|buck|all"));
  }
}

void handleDual(String args) {
  const String sub = nextToken(args);
  if (sub == F("start")) {
    startDualControl(Serial);
  } else if (sub == F("stop")) {
    stopDualControl(F("user command"), Serial);
  } else if (sub == F("status")) {
    printDualStatus(Serial);
  } else {
    Serial.println(F("Usage: dual start|stop|status"));
  }
}

void handleSensors(String args) {
  const String sub = nextToken(args);
  if (sub == F("init")) {
    runSensorInit();
  } else if (sub == F("read")) {
    runSensorRead();
  } else {
    Serial.println(F("Usage: sensors init|read"));
  }
}

void handleLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }
  const String cmd = nextToken(line);
  if (cmd == F("help")) {
    printHelp();
  } else if (cmd == F("pins")) {
    printPins();
  } else if (cmd == F("power")) {
    handlePower(line);
  } else if (cmd == F("scan")) {
    nextToken(line) == F("all") ? scanAll() : scanExpected();
  } else if (cmd == F("ina")) {
    handleIna(line);
  } else if (cmd == F("bq")) {
    handleBq(line);
  } else if (cmd == F("pwm")) {
    handlePwm(line);
  } else if (cmd == F("dual")) {
    handleDual(line);
  } else if (cmd == F("sensors")) {
    handleSensors(line);
  } else if (cmd == F("log")) {
    const String state = nextToken(line);
    if (state == F("on")) {
      logEnabled = true;
      lastLogMs = 0;
      Serial.println(F("Periodic log enabled"));
    } else if (state == F("off")) {
      logEnabled = false;
      Serial.println(F("Periodic log disabled"));
    } else {
      Serial.println(F("Usage: log on|off"));
    }
  } else if (cmd == F("faults")) {
    if (nextToken(line) == F("clear")) {
      powerPwm.clearFault();
      Serial.println(F("Software faults cleared"));
    } else {
      Serial.println(F("Usage: faults clear"));
    }
  } else {
    Serial.println(F("Unknown command; run 'help'"));
    dualControl.active = false;
    powerPwm.disableAll();
  }
}

void processSerial() {
  static String line;
  while (Serial.available() > 0) {
    const char ch = static_cast<char>(Serial.read());
    if (ch == '\r') {
      continue;
    }
    if (ch == '\n') {
      handleLine(line);
      line = "";
    } else if (line.length() < 160) {
      line += ch;
    } else {
      line = "";
      dualControl.active = false;
      powerPwm.disableAll();
      Serial.println(F("Input line too long; PWM forced disabled"));
    }
  }
}

void printBanner() {
  Serial.println();
  Serial.println(kFirmwareVersion);
  Serial.println(F("Safe boot: sensor/radio/5V rails off, charger disabled, PWM forced low."));
  Serial.println(F("Converter PWM stays disabled until explicit pwm enable command."));
  scanExpected();
  printHelp();
}
}  // namespace

void setup() {
  pinMode(Pins::PIN_SENSOR_EN, OUTPUT);
  pinMode(Pins::PIN_RADIO_EN, OUTPUT);
  pinMode(Pins::PIN_5V_EN, OUTPUT);
  pinMode(Pins::PIN_BQ_CE_N, OUTPUT);
  pinMode(Pins::PIN_BQ_INT, INPUT_PULLUP);
  pinMode(Pins::PIN_BQ_PG_N, INPUT_PULLUP);
  pinMode(Pins::PIN_WS_DOUT, INPUT);
  pinMode(Pins::PIN_LED, OUTPUT);
  pinMode(Pins::PIN_BOOT, INPUT_PULLUP);

  setPowerPin(Pins::PIN_SENSOR_EN, false);
  setPowerPin(Pins::PIN_RADIO_EN, false);
  setPowerPin(Pins::PIN_5V_EN, false);
  digitalWrite(Pins::PIN_BQ_CE_N, HIGH);
  digitalWrite(Pins::PIN_LED, LOW);

  const bool pwmReady = powerPwm.begin();
  Wire.begin(Pins::PIN_I2C_SDA, Pins::PIN_I2C_SCL, kI2cClockHz);

  Serial.begin(kSerialBaud);
  delay(1000);
  if (!pwmReady) {
    Serial.println(F("WARNING: MCPWM initialisation failed; PWM commands will be refused."));
  }
  printBanner();
}

void loop() {
  processSerial();
  runDualControlTick();
  runLogTick();
}
