#include "drivers/motor/motor_pwm.h"

#include <Arduino.h>
#include <math.h>

#include "config/flight_config.h"

namespace flight {
namespace {

constexpr uint32_t kMaximumDuty =
    (1UL << config::kMotorPwmResolutionBits) - 1UL;

float clampNormalized(const float output) {
  if (!isfinite(output)) {
    return 0.0F;
  }
  if (output < 0.0F) {
    return 0.0F;
  }
  if (output > 1.0F) {
    return 1.0F;
  }
  return output;
}

}  // namespace

bool MotorPwm::begin() {
  initialized_ = false;

  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    const uint8_t pin = hardware::kMotorPins[channel];
    if (ledcSetup(channel, config::kMotorPwmFrequencyHz,
                  config::kMotorPwmResolutionBits) == 0U) {
      stopAll();
      return false;
    }

    ledcAttachPin(pin, channel);
    ledcWrite(channel, 0U);
  }

  initialized_ = true;
  return true;
}

bool MotorPwm::setNormalized(const hardware::MotorId motor,
                             const float output) {
  if (!initialized_) {
    return false;
  }

  const uint8_t channel = static_cast<uint8_t>(motor);
  if (channel >= kChannelCount) {
    return false;
  }

  const uint32_t duty = static_cast<uint32_t>(
      clampNormalized(output) * static_cast<float>(kMaximumDuty));
  ledcWrite(channel, duty);
  return true;
}

void MotorPwm::stopAll() {
  for (uint8_t channel = 0; channel < kChannelCount; ++channel) {
    ledcWrite(channel, 0U);
  }
}

}  // namespace flight

