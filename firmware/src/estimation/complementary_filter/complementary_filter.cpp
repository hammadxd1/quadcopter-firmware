#include "estimation/complementary_filter/complementary_filter.h"

#include <math.h>

namespace flight {
namespace {
float magnitude(const Vector3f& v) { return sqrtf(v.x * v.x + v.y * v.y + v.z * v.z); }
}  // namespace

ComplementaryFilter::ComplementaryFilter(ComplementaryFilterConfiguration configuration)
    : configuration_(configuration) {}

bool ComplementaryFilter::update(const ImuSample& input, float dtSec, AttitudeEstimate& estimate) {
  estimate = state_;
  if (!input.valid || dtSec <= 0.0F || dtSec > configuration_.maximumDtSec) return false;
  const Vector3f rate(input.angularRateRadPerSec.x * configuration_.rollAxisSign,
                      input.angularRateRadPerSec.y * configuration_.pitchAxisSign,
                      input.angularRateRadPerSec.z * configuration_.yawAxisSign);
  const Vector3f accel(input.accelerationMps2.x * configuration_.pitchAxisSign,
                       input.accelerationMps2.y * configuration_.rollAxisSign,
                       input.accelerationMps2.z);
  const float accelMagnitude = magnitude(accel);
  const bool useAccel = accelMagnitude >= configuration_.accelCorrectionMinimumMps2 &&
                        accelMagnitude <= configuration_.accelCorrectionMaximumMps2;
  const float accelRoll = atan2f(accel.y, accel.z);
  const float accelPitch = atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z));
  if (!initialized_) {
    if (!useAccel) return false;
    state_.rollRad = accelRoll;
    state_.pitchRad = accelPitch;
    initialized_ = true;
  } else {
    const float gyroRoll = state_.rollRad + rate.x * dtSec;
    const float gyroPitch = state_.pitchRad + rate.y * dtSec;
    if (useAccel) {
      const float alpha = configuration_.timeConstantSec / (configuration_.timeConstantSec + dtSec);
      state_.rollRad = alpha * gyroRoll + (1.0F - alpha) * accelRoll;
      state_.pitchRad = alpha * gyroPitch + (1.0F - alpha) * accelPitch;
    } else {
      state_.rollRad = gyroRoll;
      state_.pitchRad = gyroPitch;
    }
  }
  state_.angularRateRadPerSec = rate;
  state_.timestampUs = input.timestampUs;
  state_.valid = true;
  estimate = state_;
  return true;
}

void ComplementaryFilter::reset() { initialized_ = false; state_ = {}; }
}  // namespace flight
