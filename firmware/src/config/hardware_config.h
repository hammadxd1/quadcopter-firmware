#pragma once

#include <stdint.h>

namespace flight::hardware {

constexpr uint8_t kImu1I2cBus = 0;
constexpr uint8_t kImu1SdaPin = 21;
constexpr uint8_t kImu1SclPin = 22;
constexpr uint8_t kImu2I2cBus = 1;
constexpr uint8_t kImu2SdaPin = 25;
constexpr uint8_t kImu2SclPin = 26;
constexpr uint8_t kImu1Address = 0x68;
constexpr uint8_t kImu2Address = 0x69;
constexpr uint32_t kI2cClockHz = 400000;

constexpr uint8_t kMotorFrontLeftPwmPin = 13;   // CW.
constexpr uint8_t kMotorFrontRightPwmPin = 14;  // CCW.
constexpr uint8_t kMotorRearLeftPwmPin = 27;    // CCW.
constexpr uint8_t kMotorRearRightPwmPin = 33;   // CW.

constexpr uint8_t kMotorPins[] = {
    kMotorFrontLeftPwmPin,
    kMotorFrontRightPwmPin,
    kMotorRearLeftPwmPin,
    kMotorRearRightPwmPin,
};

enum class MotorId : uint8_t {
  kFrontLeft = 0,
  kFrontRight = 1,
  kRearLeft = 2,
  kRearRight = 3,
  kCount = 4,
};

constexpr uint8_t kMotorCount = static_cast<uint8_t>(MotorId::kCount);

static_assert(kMotorFrontLeftPwmPin != kMotorFrontRightPwmPin &&
                  kMotorFrontLeftPwmPin != kMotorRearLeftPwmPin &&
                  kMotorFrontLeftPwmPin != kMotorRearRightPwmPin &&
                  kMotorFrontRightPwmPin != kMotorRearLeftPwmPin &&
                  kMotorFrontRightPwmPin != kMotorRearRightPwmPin &&
                  kMotorRearLeftPwmPin != kMotorRearRightPwmPin,
              "Motor PWM pins must be unique.");

}  // namespace flight::hardware

