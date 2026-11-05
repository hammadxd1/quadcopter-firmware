#pragma once

#include <stdint.h>

namespace flight::config {

constexpr uint32_t kControlLoopHz = 250;
constexpr uint32_t kControlPeriodUs = 1000000UL / kControlLoopHz;
constexpr uint32_t kMotorPwmFrequencyHz = 22000;
constexpr uint8_t kMotorPwmResolutionBits = 10;

// Body-frame convention. Sensor-axis signs are bench-verified before arming.
constexpr bool kBodyXAxisPointsForward = true;

}  // namespace flight::config

