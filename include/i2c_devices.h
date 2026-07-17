#pragma once

#include <Arduino.h>

namespace I2CDevices {

// Boost/input INA228 has A1=VS, A0=GND on the as-built board.
constexpr uint8_t ADDR_INA228_BOOST = 0x44;
// Buck/output INA228 has A1=GND, A0=GND on the as-built board.
constexpr uint8_t ADDR_INA228_BUCK = 0x40;
constexpr uint8_t ADDR_SCD30 = 0x61;
constexpr uint8_t ADDR_SPS30 = 0x69;
constexpr uint8_t ADDR_BQ25186 = 0x6A;

struct ExpectedDevice {
  uint8_t address;
  const char* name;
  const char* function;
};

constexpr ExpectedDevice EXPECTED[] = {
    {ADDR_INA228_BOOST, "INA228 #1", "boost/input current and voltage monitor"},
    {ADDR_INA228_BUCK, "INA228 #2", "buck/output current and voltage monitor"},
    {ADDR_SCD30, "SCD30", "CO2, temperature, humidity sensor"},
    {ADDR_SPS30, "SPS30", "particulate matter sensor"},
    {ADDR_BQ25186, "BQ25186", "battery charger"},
};

constexpr size_t EXPECTED_COUNT = sizeof(EXPECTED) / sizeof(EXPECTED[0]);

}  // namespace I2CDevices
