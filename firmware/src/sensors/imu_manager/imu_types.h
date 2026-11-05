#pragma once

#include <stdint.h>

namespace flight {

struct Vector3f {
  constexpr Vector3f() = default;
  constexpr Vector3f(const float xValue, const float yValue,
                     const float zValue)
      : x(xValue), y(yValue), z(zValue) {}

  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct ImuSample {
  Vector3f accelerationMps2{};
  Vector3f angularRateRadPerSec{};
  float temperatureC = 0.0F;
  uint64_t timestampUs = 0U;
  bool valid = false;
};

struct ImuHealth {
  bool initialized = false;
  bool calibrated = false;
  bool healthy = false;
  uint32_t successfulReads = 0U;
  uint32_t failedReads = 0U;
  uint32_t consecutiveFailures = 0U;
  uint64_t lastSampleTimestampUs = 0U;
};

struct DualImuSample {
  ImuSample imu1{};
  ImuSample imu2{};
};

}  // namespace flight

