#pragma once

#include <stdint.h>

namespace flight::hardware {

constexpr uint8_t kI2cSdaPin = 21;
constexpr uint8_t kI2cSclPin = 22;
constexpr uint8_t kImu1Address = 0x68;
constexpr uint8_t kImu2Address = 0x69;
constexpr uint32_t kI2cClockHz = 400000;

constexpr uint8_t kMotor1PwmPin = 25;  // Front-right, CCW.
constexpr uint8_t kMotor2PwmPin = 26;  // Front-left, CW.
constexpr uint8_t kMotor3PwmPin = 27;  // Rear-right, CW.
constexpr uint8_t kMotor4PwmPin = 14;  // Rear-left, CCW.

constexpr uint8_t kMotorPins[] = {
    kMotor1PwmPin,
    kMotor2PwmPin,
    kMotor3PwmPin,
    kMotor4PwmPin,
};

enum class MotorId : uint8_t {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearRight = 2,
  kRearLeft = 3,
  kCount = 4,
};

constexpr uint8_t kMotorCount = static_cast<uint8_t>(MotorId::kCount);

static_assert(kMotor1PwmPin != kMotor2PwmPin &&
                  kMotor1PwmPin != kMotor3PwmPin &&
                  kMotor1PwmPin != kMotor4PwmPin &&
                  kMotor2PwmPin != kMotor3PwmPin &&
                  kMotor2PwmPin != kMotor4PwmPin &&
                  kMotor3PwmPin != kMotor4PwmPin,
              "Motor PWM pins must be unique.");

}  // namespace flight::hardware

