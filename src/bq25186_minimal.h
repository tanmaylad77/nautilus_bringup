#pragma once

#include <Arduino.h>
#include <Wire.h>

class BQ25186 {
 public:
  explicit BQ25186(TwoWire& wire = Wire, uint8_t address = 0x6A);

  bool readRegister(uint8_t reg, uint8_t& value) const;
  bool writeRegister(uint8_t reg, uint8_t value) const;
  bool updateRegister(uint8_t reg, uint8_t mask, uint8_t value) const;

  bool setLiIon420Profile() const;
  bool setChargeCurrentMilliAmps(uint16_t milliamps) const;
  bool setChargeEnabled(bool enabled) const;
  bool dumpRegisters(Stream& out) const;
  bool printStatus(Stream& out, int pgLevel, int intLevel) const;

  static constexpr uint8_t REG_STAT0 = 0x00;
  static constexpr uint8_t REG_STAT1 = 0x01;
  static constexpr uint8_t REG_FLAG0 = 0x02;
  static constexpr uint8_t REG_VBAT_CTRL = 0x03;
  static constexpr uint8_t REG_ICHG_CTRL = 0x04;
  static constexpr uint8_t REG_CHARGECTRL0 = 0x05;
  static constexpr uint8_t REG_CHARGECTRL1 = 0x06;
  static constexpr uint8_t REG_IC_CTRL = 0x07;
  static constexpr uint8_t REG_TMR_ILIM = 0x08;
  static constexpr uint8_t REG_SHIP_RST = 0x09;

  static constexpr uint8_t ICHG_CHG_DIS = 0x80;
  static constexpr uint16_t SAFE_MAX_CHARGE_MA = 100;

 private:
  TwoWire* wire_;
  uint8_t address_;
};
