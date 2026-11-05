#pragma once

#include <stdint.h>

namespace flight::imu_config {

constexpr uint8_t kWhoAmIRegister = 0x75;
constexpr uint8_t kExpectedWhoAmI = 0x68;

constexpr uint8_t kAccelConfigRegister = 0x1C;
constexpr uint8_t kGyroConfigRegister = 0x1B;
constexpr uint8_t kConfigRegister = 0x1A;
constexpr uint8_t kSampleRateDividerRegister = 0x19;
constexpr uint8_t kPowerManagement1Register = 0x6B;
constexpr uint8_t kAccelDataRegister = 0x3B;

// ±8 g and ±1000 deg/s are conservative initial ranges. They are kept
// centralized for later vibration and rate-data validation.
constexpr uint8_t kAccelRangeBits = 0x10;
constexpr uint8_t kGyroRangeBits = 0x10;
constexpr float kAccelScaleMps2PerLsb = 9.80665F / 4096.0F;
constexpr float kGyroScaleRadPerSecPerLsb =
    (3.14159265358979323846F / 180.0F) / 32.8F;

constexpr uint8_t kDlpFilterBits = 0x03;  // 42 Hz gyro/accel DLPF.
constexpr uint8_t kSampleRateDivider = 3; // 1 kHz / (1 + 3) = 250 Hz.
constexpr uint16_t kCalibrationSampleCount = 500;
constexpr uint16_t kCalibrationSampleDelayMs = 4;
constexpr uint32_t kI2cTimeoutMs = 10;

constexpr float kGravityMps2 = 9.80665F;
constexpr float kStationaryAccelToleranceMps2 = 1.5F;

}  // namespace flight::imu_config

