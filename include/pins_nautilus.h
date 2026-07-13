#pragma once

#include <Arduino.h>

namespace Pins {

// Power enables
constexpr gpio_num_t PIN_SENSOR_EN = GPIO_NUM_5;
constexpr gpio_num_t PIN_RADIO_EN = GPIO_NUM_6;
constexpr gpio_num_t PIN_5V_EN = GPIO_NUM_7;

// I2C
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_8;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_9;

// Converter PWM
constexpr gpio_num_t PIN_BOOST_LO_PWM = GPIO_NUM_15;
constexpr gpio_num_t PIN_BOOST_HI_PWM = GPIO_NUM_16;
constexpr gpio_num_t PIN_BUCK_LO_PWM = GPIO_NUM_17;
constexpr gpio_num_t PIN_BUCK_HI_PWM = GPIO_NUM_18;

// Charger GPIOs
constexpr gpio_num_t PIN_BQ_INT = GPIO_NUM_14;
constexpr gpio_num_t PIN_BQ_CE_N = GPIO_NUM_21;
constexpr gpio_num_t PIN_BQ_PG_N = GPIO_NUM_48;

// Flame/Waveshare sensor
constexpr gpio_num_t PIN_WS_AOUT = GPIO_NUM_2;
constexpr gpio_num_t PIN_WS_DOUT = GPIO_NUM_40;

// LoRa definitions only. LoRa functional validation is intentionally out of scope.
constexpr gpio_num_t PIN_LORA_NSS = GPIO_NUM_10;
constexpr gpio_num_t PIN_LORA_MOSI = GPIO_NUM_11;
constexpr gpio_num_t PIN_LORA_SCK = GPIO_NUM_12;
constexpr gpio_num_t PIN_LORA_MISO = GPIO_NUM_13;
constexpr gpio_num_t PIN_LORA_BUSY = GPIO_NUM_38;
constexpr gpio_num_t PIN_LORA_NRST = GPIO_NUM_39;
constexpr gpio_num_t PIN_RF_SW = GPIO_NUM_4;

// User I/O
constexpr gpio_num_t PIN_LED = GPIO_NUM_1;
constexpr gpio_num_t PIN_BOOT = GPIO_NUM_0;

}  // namespace Pins
