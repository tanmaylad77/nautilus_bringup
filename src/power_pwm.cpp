#include "power_pwm.h"

#include <driver/mcpwm.h>
#include "pins_nautilus.h"

namespace {
constexpr uint32_t kPwmFrequencyHz = 50000;
constexpr uint32_t kDeadtimeTicks = 40;  // 500 ns at the 80 MHz MCPWM clock.
constexpr float kMaxDutyPercent = 30.0f;
constexpr float kControlMaxBoostDutyPercent = 30.0f;
constexpr float kControlMaxBuckDutyPercent = 85.0f;
constexpr float kMinEnabledDutyPercent = 0.001f;

constexpr mcpwm_unit_t kBoostUnit = MCPWM_UNIT_0;
constexpr mcpwm_timer_t kBoostTimer = MCPWM_TIMER_0;
constexpr mcpwm_unit_t kBuckUnit = MCPWM_UNIT_0;
constexpr mcpwm_timer_t kBuckTimer = MCPWM_TIMER_1;

void configureLowPins() {
  pinMode(Pins::PIN_BOOST_LO_PWM, OUTPUT);
  pinMode(Pins::PIN_BOOST_HI_PWM, OUTPUT);
  pinMode(Pins::PIN_BUCK_LO_PWM, OUTPUT);
  pinMode(Pins::PIN_BUCK_HI_PWM, OUTPUT);
  digitalWrite(Pins::PIN_BOOST_LO_PWM, LOW);
  digitalWrite(Pins::PIN_BOOST_HI_PWM, LOW);
  digitalWrite(Pins::PIN_BUCK_LO_PWM, LOW);
  digitalWrite(Pins::PIN_BUCK_HI_PWM, LOW);
}

void setOperatorDuty(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, float dutyPercent) {
  mcpwm_set_duty(unit, timer, op, dutyPercent);
  mcpwm_set_duty_type(unit, timer, op, MCPWM_DUTY_MODE_0);
}
}  // namespace

bool PowerPwm::begin() {
  initialized_ = false;
  configureLowPins();

  mcpwm_gpio_init(kBoostUnit, MCPWM0A, Pins::PIN_BOOST_HI_PWM);
  mcpwm_gpio_init(kBoostUnit, MCPWM0B, Pins::PIN_BOOST_LO_PWM);
  mcpwm_gpio_init(kBuckUnit, MCPWM1A, Pins::PIN_BUCK_HI_PWM);
  mcpwm_gpio_init(kBuckUnit, MCPWM1B, Pins::PIN_BUCK_LO_PWM);

  mcpwm_config_t config = {};
  config.frequency = kPwmFrequencyHz;
  config.cmpr_a = 0.0f;
  config.cmpr_b = 0.0f;
  config.counter_mode = MCPWM_UP_COUNTER;
  config.duty_mode = MCPWM_DUTY_MODE_0;

  if (mcpwm_init(kBoostUnit, kBoostTimer, &config) != ESP_OK) {
    return false;
  }
  if (mcpwm_init(kBuckUnit, kBuckTimer, &config) != ESP_OK) {
    return false;
  }
#ifdef ENABLE_SYNC_COMPLEMENTARY_PWM
  if (mcpwm_deadtime_enable(kBoostUnit, kBoostTimer, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                            kDeadtimeTicks, kDeadtimeTicks) != ESP_OK) {
    return false;
  }
  if (mcpwm_deadtime_enable(kBuckUnit, kBuckTimer, MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE,
                            kDeadtimeTicks, kDeadtimeTicks) != ESP_OK) {
    return false;
  }
#else
  (void)kDeadtimeTicks;
#endif

  disableAll();
  initialized_ = true;
  return true;
}

bool PowerPwm::setDuty(const String& stage, float dutyPercent, Stream& out) {
  if (!initialized_) {
    latchFault(F("PWM hardware was not initialised; command rejected"), out);
    return false;
  }
  if (dutyPercent < 0.0f || dutyPercent > kMaxDutyPercent) {
    latchFault(F("PWM duty rejected; allowed range is 0..30%"), out);
    disableAll();
    return false;
  }
  if (stage == F("boost")) {
    boostDuty_ = dutyPercent;
    if (boostEnabled_) {
      if (dutyPercent <= kMinEnabledDutyPercent) {
        disableStage(Stage::Boost);
        out.println(F("Boost duty set to 0%; boost PWM disabled"));
      } else {
        setDuty(Stage::Boost, dutyPercent);
        out.println(F("Boost duty updated"));
      }
    } else {
      out.println(F("Boost duty staged"));
    }
    return true;
  }
  if (stage == F("buck")) {
    buckDuty_ = dutyPercent;
    if (buckEnabled_) {
      if (dutyPercent <= kMinEnabledDutyPercent) {
        disableStage(Stage::Buck);
        out.println(F("Buck duty set to 0%; buck PWM disabled"));
      } else {
        setDuty(Stage::Buck, dutyPercent);
        out.println(F("Buck duty updated"));
      }
    } else {
      out.println(F("Buck duty staged"));
    }
    return true;
  }
  latchFault(F("Unknown PWM stage"), out);
  disableAll();
  return false;
}

bool PowerPwm::setBoostDutyForControl(float dutyPercent, Stream& out) {
  return setDutyForControl(Stage::Boost, dutyPercent, kControlMaxBoostDutyPercent, out);
}

bool PowerPwm::setBuckDutyForControl(float dutyPercent, Stream& out) {
  return setDutyForControl(Stage::Buck, dutyPercent, kControlMaxBuckDutyPercent, out);
}

bool PowerPwm::enable(const String& stage, Stream& out) {
  if (!initialized_) {
    latchFault(F("PWM hardware was not initialised; enable rejected"), out);
    return false;
  }
  if (faultLatched_) {
    out.println(F("PWM fault is latched; run 'pwm disable all' then 'faults clear'"));
    return false;
  }
  if (stage == F("boost")) {
#ifndef ALLOW_DUAL_STAGE_PWM
    if (buckEnabled_) {
      latchFault(F("Refusing to enable boost while buck is enabled"), out);
      disableAll();
      return false;
    }
#endif
    boostEnabled_ = enableStage(Stage::Boost, out);
    out.println(boostEnabled_ ? F("Boost PWM enabled") : F("Boost PWM enable failed"));
    return boostEnabled_;
  }
  if (stage == F("buck")) {
#ifndef ALLOW_DUAL_STAGE_PWM
    if (boostEnabled_) {
      latchFault(F("Refusing to enable buck while boost is enabled"), out);
      disableAll();
      return false;
    }
#endif
    buckEnabled_ = enableStage(Stage::Buck, out);
    out.println(buckEnabled_ ? F("Buck PWM enabled") : F("Buck PWM enable failed"));
    return buckEnabled_;
  }
  latchFault(F("Unknown PWM stage"), out);
  disableAll();
  return false;
}

bool PowerPwm::enableDualForControl(Stream& out) {
  if (!initialized_) {
    latchFault(F("PWM hardware was not initialised; dual enable rejected"), out);
    return false;
  }
  if (faultLatched_) {
    out.println(F("PWM fault is latched; run 'pwm disable all' then 'faults clear'"));
    return false;
  }
  const bool boostOk = enableStage(Stage::Boost, out);
  const bool buckOk = enableStage(Stage::Buck, out);
  boostEnabled_ = boostOk;
  buckEnabled_ = buckOk;
  if (!(boostOk && buckOk)) {
    disableAll();
  }
  out.println((boostOk && buckOk) ? F("Boost and buck PWM enabled for controlled test")
                                  : F("Dual PWM enable failed"));
  return boostOk && buckOk;
}

bool PowerPwm::disable(const String& stage, Stream& out) {
  if (stage == F("boost")) {
    disableStage(Stage::Boost);
    out.println(F("Boost PWM disabled"));
    return true;
  }
  if (stage == F("buck")) {
    disableStage(Stage::Buck);
    out.println(F("Buck PWM disabled"));
    return true;
  }
  if (stage == F("all")) {
    disableAll();
    out.println(F("All PWM disabled"));
    return true;
  }
  out.println(F("Unknown PWM stage"));
  disableAll();
  faultLatched_ = true;
  return false;
}

void PowerPwm::disableAll() {
  disableStage(Stage::Boost);
  disableStage(Stage::Buck);
}

void PowerPwm::printStatus(Stream& out) const {
  out.print(F("PWM: hw="));
  out.print(initialized_ ? F("ready") : F("not-ready"));
  out.print(F(", boost="));
  out.print(boostEnabled_ ? F("enabled") : F("disabled"));
  out.print(F(" duty="));
  out.print(boostDuty_, 2);
  out.print(F("%, buck="));
  out.print(buckEnabled_ ? F("enabled") : F("disabled"));
  out.print(F(" duty="));
  out.print(buckDuty_, 2);
  out.print(F("%, fault="));
  out.println(faultLatched_ ? F("latched") : F("clear"));
}

bool PowerPwm::setDuty(Stage stage, float dutyPercent) {
  const mcpwm_timer_t timer = stage == Stage::Boost ? kBoostTimer : kBuckTimer;
  if (stage == Stage::Boost) {
#ifdef ENABLE_SYNC_COMPLEMENTARY_PWM
    mcpwm_set_duty(kBoostUnit, timer, MCPWM_OPR_A, 100.0f - dutyPercent);
    mcpwm_set_duty_type(kBoostUnit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
    mcpwm_set_duty_type(kBoostUnit, timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
#else
    setOperatorDuty(kBoostUnit, timer, MCPWM_OPR_A, 0.0f);
    setOperatorDuty(kBoostUnit, timer, MCPWM_OPR_B, dutyPercent);
#endif
    return true;
  }
#ifdef ENABLE_SYNC_COMPLEMENTARY_PWM
  mcpwm_set_duty(kBuckUnit, timer, MCPWM_OPR_A, dutyPercent);
  mcpwm_set_duty_type(kBuckUnit, timer, MCPWM_OPR_A, MCPWM_DUTY_MODE_0);
  mcpwm_set_duty_type(kBuckUnit, timer, MCPWM_OPR_B, MCPWM_DUTY_MODE_0);
#else
  setOperatorDuty(kBuckUnit, timer, MCPWM_OPR_B, 0.0f);
  setOperatorDuty(kBuckUnit, timer, MCPWM_OPR_A, dutyPercent);
#endif
  return true;
}

bool PowerPwm::setDutyForControl(Stage stage, float dutyPercent, float maxDutyPercent, Stream& out) {
  if (!initialized_) {
    latchFault(F("PWM hardware was not initialised; control duty rejected"), out);
    return false;
  }
  if (dutyPercent < 0.0f || dutyPercent > maxDutyPercent) {
    latchFault(F("Control PWM duty rejected; requested duty outside control limit"), out);
    disableAll();
    return false;
  }
  if (stage == Stage::Boost) {
    boostDuty_ = dutyPercent;
    if (boostEnabled_) {
      setDuty(Stage::Boost, dutyPercent);
    }
  } else {
    buckDuty_ = dutyPercent;
    if (buckEnabled_) {
      setDuty(Stage::Buck, dutyPercent);
    }
  }
  return true;
}

bool PowerPwm::enableStage(Stage stage, Stream& out) {
  const float duty = stage == Stage::Boost ? boostDuty_ : buckDuty_;
  if (duty <= kMinEnabledDutyPercent) {
    out.println(F("Refusing to enable PWM with 0% duty; stage a positive duty first"));
    disableStage(stage);
    return false;
  }
  setDuty(stage, duty);
  return true;
}

void PowerPwm::disableStage(Stage stage) {
  const mcpwm_timer_t timer = stage == Stage::Boost ? kBoostTimer : kBuckTimer;
  const mcpwm_unit_t unit = stage == Stage::Boost ? kBoostUnit : kBuckUnit;
  setOperatorDuty(unit, timer, MCPWM_OPR_A, 0.0f);
  setOperatorDuty(unit, timer, MCPWM_OPR_B, 0.0f);
  if (stage == Stage::Boost) {
    boostEnabled_ = false;
  } else {
    buckEnabled_ = false;
  }
}

void PowerPwm::latchFault(const __FlashStringHelper* message, Stream& out) {
  faultLatched_ = true;
  out.print(F("FAULT: "));
  out.println(message);
}
