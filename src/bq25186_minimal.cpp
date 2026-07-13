#include "bq25186_minimal.h"

namespace {
constexpr uint8_t kVBatReg420Code = 70;  // 3.5 V + 70 * 10 mV = 4.20 V.
constexpr uint8_t kIchgCodeMask = 0x7F;

uint8_t chargeCurrentToCode(uint16_t milliamps) {
  if (milliamps <= 35) {
    return static_cast<uint8_t>(milliamps > 5 ? milliamps - 5 : 0);
  }
  if (milliamps <= BQ25186::SAFE_MAX_CHARGE_MA) {
    return static_cast<uint8_t>(31 + ((milliamps - 40 + 9) / 10));
  }
  return chargeCurrentToCode(BQ25186::SAFE_MAX_CHARGE_MA);
}
}  // namespace

BQ25186::BQ25186(TwoWire& wire, uint8_t address) : wire_(&wire), address_(address) {}

bool BQ25186::readRegister(uint8_t reg, uint8_t& value) const {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  if (wire_->requestFrom(static_cast<int>(address_), 1) != 1) {
    return false;
  }
  value = wire_->read();
  return true;
}

bool BQ25186::writeRegister(uint8_t reg, uint8_t value) const {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(value);
  return wire_->endTransmission() == 0;
}

bool BQ25186::updateRegister(uint8_t reg, uint8_t mask, uint8_t value) const {
  uint8_t current = 0;
  if (!readRegister(reg, current)) {
    return false;
  }
  current = static_cast<uint8_t>((current & ~mask) | (value & mask));
  return writeRegister(reg, current);
}

bool BQ25186::setLiIon420Profile() const {
  if (!writeRegister(REG_VBAT_CTRL, kVBatReg420Code)) {
    return false;
  }
  if (!setChargeCurrentMilliAmps(10)) {
    return false;
  }
  return setChargeEnabled(false);
}

bool BQ25186::setChargeCurrentMilliAmps(uint16_t milliamps) const {
  const uint8_t code = chargeCurrentToCode(min<uint16_t>(milliamps, SAFE_MAX_CHARGE_MA));
  return updateRegister(REG_ICHG_CTRL, kIchgCodeMask, code);
}

bool BQ25186::setChargeEnabled(bool enabled) const {
  return updateRegister(REG_ICHG_CTRL, ICHG_CHG_DIS, enabled ? 0 : ICHG_CHG_DIS);
}

bool BQ25186::dumpRegisters(Stream& out) const {
  bool ok = true;
  out.println(F("BQ25186 registers 0x00..0x09:"));
  for (uint8_t reg = 0; reg <= REG_SHIP_RST; ++reg) {
    uint8_t value = 0;
    const bool readOk = readRegister(reg, value);
    ok = ok && readOk;
    out.print(F("  0x"));
    if (reg < 16) out.print('0');
    out.print(reg, HEX);
    out.print(F(": "));
    if (readOk) {
      out.print(F("0x"));
      if (value < 16) out.print('0');
      out.println(value, HEX);
    } else {
      out.println(F("<read failed>"));
    }
  }
  return ok;
}

bool BQ25186::printStatus(Stream& out, int pgLevel, int intLevel) const {
  uint8_t stat0 = 0;
  uint8_t stat1 = 0;
  uint8_t flag0 = 0;
  const bool ok = readRegister(REG_STAT0, stat0) && readRegister(REG_STAT1, stat1) &&
                  readRegister(REG_FLAG0, flag0);

  out.println(F("BQ25186 status:"));
  out.print(F("  /PG pin: "));
  out.println(pgLevel == LOW ? F("LOW (power good asserted)") : F("HIGH"));
  out.print(F("  /INT pin: "));
  out.println(intLevel == LOW ? F("LOW (interrupt asserted)") : F("HIGH"));
  if (!ok) {
    out.println(F("  I2C status read failed"));
    return false;
  }
  out.print(F("  STAT0=0x"));
  if (stat0 < 16) out.print('0');
  out.print(stat0, HEX);
  out.print(F(" STAT1=0x"));
  if (stat1 < 16) out.print('0');
  out.print(stat1, HEX);
  out.print(F(" FLAG0=0x"));
  if (flag0 < 16) out.print('0');
  out.println(flag0, HEX);
  return true;
}
