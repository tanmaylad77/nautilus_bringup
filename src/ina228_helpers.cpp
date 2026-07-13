#include "ina228_helpers.h"

namespace {
constexpr uint8_t REG_CONFIG = 0x00;
constexpr uint8_t REG_ADC_CONFIG = 0x01;
constexpr uint8_t REG_SHUNT_CAL = 0x02;
constexpr uint8_t REG_VSHUNT = 0x04;
constexpr uint8_t REG_VBUS = 0x05;
constexpr uint8_t REG_DIETEMP = 0x06;
constexpr uint8_t REG_CURRENT = 0x07;
constexpr uint8_t REG_POWER = 0x08;
constexpr uint8_t REG_MANUFACTURER_ID = 0x3E;
constexpr uint8_t REG_DEVICE_ID = 0x3F;

int32_t signExtend20(uint32_t raw) {
  raw &= 0xFFFFFUL;
  if (raw & 0x80000UL) {
    raw |= 0xFFF00000UL;
  }
  return static_cast<int32_t>(raw);
}

int16_t asSigned16(uint16_t raw) {
  return static_cast<int16_t>(raw);
}
}  // namespace

INA228Device::INA228Device(const char* label, uint8_t address, TwoWire& wire)
    : label_(label), address_(address), wire_(&wire) {}

bool INA228Device::begin(float shuntOhms, float maxExpectedCurrentA, Stream& out) {
  uint16_t manufacturer = 0;
  uint16_t device = 0;
  const bool idsOk = readRegister16(REG_MANUFACTURER_ID, manufacturer) &&
                     readRegister16(REG_DEVICE_ID, device);

  currentLsbA_ = maxExpectedCurrentA / 524288.0f;
  const float shuntCalFloat = 13107.2e6f * currentLsbA_ * shuntOhms;
  const uint16_t shuntCal = static_cast<uint16_t>(min<float>(shuntCalFloat + 0.5f, 65535.0f));

  bool ok = writeRegister16(REG_CONFIG, 0x0000);
  ok = ok && writeRegister16(REG_ADC_CONFIG, 0xFB68);
  ok = ok && writeRegister16(REG_SHUNT_CAL, shuntCal);

  out.print(label_);
  out.print(F(" INA228 init: "));
  out.print(ok ? F("OK") : F("FAILED"));
  if (idsOk) {
    out.print(F(" manufacturer=0x"));
    out.print(manufacturer, HEX);
    out.print(F(" device=0x"));
    out.print(device, HEX);
  } else {
    out.print(F(" ID read failed"));
  }
  out.print(F(" current_lsb_A="));
  out.print(currentLsbA_, 9);
  out.print(F(" shunt_cal="));
  out.println(shuntCal);
  return ok && idsOk;
}

Ina228Sample INA228Device::readSample() const {
  Ina228Sample sample;
  if (currentLsbA_ <= 0.0f) {
    return sample;
  }

  uint32_t vshuntRaw = 0;
  uint32_t vbusRaw = 0;
  uint32_t currentRaw = 0;
  uint32_t power = 0;
  uint16_t tempRaw = 0;
  sample.ok = readRegister24(REG_VSHUNT, vshuntRaw) && readRegister24(REG_VBUS, vbusRaw) &&
              readRegister16(REG_DIETEMP, tempRaw) && readRegister24(REG_CURRENT, currentRaw) &&
              readRegister24(REG_POWER, power);
  if (!sample.ok) {
    return sample;
  }

  const int32_t vshunt = signExtend20(vshuntRaw >> 4);
  const int32_t vbus = signExtend20(vbusRaw >> 4);
  const int32_t current = signExtend20(currentRaw >> 4);
  sample.shuntVoltageMv = static_cast<float>(vshunt) * 0.0003125f;
  sample.busVoltageV = static_cast<float>(vbus) * 0.0001953125f;
  sample.temperatureC = static_cast<float>(asSigned16(tempRaw)) * 0.0078125f;
  sample.currentA = static_cast<float>(current) * currentLsbA_;
  sample.powerW = static_cast<float>(power) * 3.2f * currentLsbA_;
  return sample;
}

void INA228Device::printSample(Stream& out) const {
  const Ina228Sample sample = readSample();
  out.print(label_);
  out.println(F(" INA228:"));
  if (!sample.ok) {
    out.println(F("  read failed or device not initialised"));
    return;
  }
  out.print(F("  Vbus="));
  out.print(sample.busVoltageV, 4);
  out.print(F(" V, Vshunt="));
  out.print(sample.shuntVoltageMv, 4);
  out.print(F(" mV, I="));
  out.print(sample.currentA, 6);
  out.print(F(" A, P="));
  out.print(sample.powerW, 6);
  out.print(F(" W, Tdie="));
  out.print(sample.temperatureC, 2);
  out.println(F(" C"));
}

bool INA228Device::writeRegister16(uint8_t reg, uint16_t value) const {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  wire_->write(static_cast<uint8_t>(value >> 8));
  wire_->write(static_cast<uint8_t>(value & 0xFF));
  return wire_->endTransmission() == 0;
}

bool INA228Device::readRegister16(uint8_t reg, uint16_t& value) const {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  if (wire_->requestFrom(static_cast<int>(address_), 2) != 2) {
    return false;
  }
  value = static_cast<uint16_t>(wire_->read() << 8);
  value |= static_cast<uint16_t>(wire_->read());
  return true;
}

bool INA228Device::readRegister24(uint8_t reg, uint32_t& value) const {
  wire_->beginTransmission(address_);
  wire_->write(reg);
  if (wire_->endTransmission(false) != 0) {
    return false;
  }
  if (wire_->requestFrom(static_cast<int>(address_), 3) != 3) {
    return false;
  }
  const uint32_t raw = (static_cast<uint32_t>(wire_->read()) << 16) |
                       (static_cast<uint32_t>(wire_->read()) << 8) |
                       static_cast<uint32_t>(wire_->read());
  value = raw;
  return true;
}
