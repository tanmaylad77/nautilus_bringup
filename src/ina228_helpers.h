#pragma once

#include <Arduino.h>
#include <Wire.h>

struct Ina228Sample {
  bool ok = false;
  float busVoltageV = 0.0f;
  float shuntVoltageMv = 0.0f;
  float currentA = 0.0f;
  float powerW = 0.0f;
  float temperatureC = 0.0f;
};

class INA228Device {
 public:
  INA228Device(const char* label, uint8_t address, TwoWire& wire = Wire);

  bool begin(float shuntOhms, float maxExpectedCurrentA, Stream& out);
  Ina228Sample readSample() const;
  void printSample(Stream& out) const;

 private:
  bool writeRegister16(uint8_t reg, uint16_t value) const;
  bool readRegister16(uint8_t reg, uint16_t& value) const;
  bool readRegister24(uint8_t reg, uint32_t& value) const;

  const char* label_;
  uint8_t address_;
  TwoWire* wire_;
  float currentLsbA_ = 0.0f;
};
