#pragma once

#include <Arduino.h>

class PowerPwm {
 public:
  bool begin();
  bool setDuty(const String& stage, float dutyPercent, Stream& out);
  bool enable(const String& stage, Stream& out);
  bool disable(const String& stage, Stream& out);
  void disableAll();
  void printStatus(Stream& out) const;
  bool ready() const { return initialized_; }
  bool hasFault() const { return faultLatched_; }
  void clearFault() { faultLatched_ = false; }
  bool boostEnabled() const { return boostEnabled_; }
  bool buckEnabled() const { return buckEnabled_; }

 private:
  enum class Stage { Boost, Buck };

  bool setDuty(Stage stage, float dutyPercent);
  bool enableStage(Stage stage, Stream& out);
  void disableStage(Stage stage);
  void latchFault(const __FlashStringHelper* message, Stream& out);

  float boostDuty_ = 0.0f;
  float buckDuty_ = 0.0f;
  bool boostEnabled_ = false;
  bool buckEnabled_ = false;
  bool initialized_ = false;
  bool faultLatched_ = false;
};
