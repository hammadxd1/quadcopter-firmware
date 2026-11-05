#include "controllers/pid/pid_controller.h"
#include <math.h>

namespace flight {
namespace {
float clamp(float value, float limit) {
  if (limit <= 0.0F) return 0.0F;
  if (value > limit) return limit;
  if (value < -limit) return -limit;
  return value;
}
bool sameDirection(float a, float b) { return (a >= 0.0F && b >= 0.0F) || (a <= 0.0F && b <= 0.0F); }
}  // namespace

PidController::PidController(PidGains gains) : gains_(gains) {}
float PidController::update(float setpoint, float measurement, float dtSec) {
  if (!isfinite(setpoint) || !isfinite(measurement) || dtSec <= 0.0F) { reset(); return 0.0F; }
  const float error = setpoint - measurement;
  float measurementDerivative = 0.0F;
  if (hasPreviousMeasurement_) measurementDerivative = (measurement - previousMeasurement_) / dtSec;
  previousMeasurement_ = measurement;
  hasPreviousMeasurement_ = true;
  if (gains_.derivativeFilterTimeConstantSec > 0.0F) {
    const float alpha = dtSec / (gains_.derivativeFilterTimeConstantSec + dtSec);
    filteredMeasurementDerivative_ += alpha * (measurementDerivative - filteredMeasurementDerivative_);
  } else {
    filteredMeasurementDerivative_ = measurementDerivative;
  }
  const float candidateIntegral = clamp(integralState_ + gains_.integral * error * dtSec, gains_.integralLimit);
  const float unsaturated = gains_.proportional * error + candidateIntegral -
                            gains_.derivative * filteredMeasurementDerivative_ + gains_.feedForward * setpoint;
  const float output = clamp(unsaturated, gains_.outputLimit);
  if (output == unsaturated || !sameDirection(error, unsaturated)) integralState_ = candidateIntegral;
  return output;
}
void PidController::reset() { integralState_ = 0.0F; filteredMeasurementDerivative_ = 0.0F; previousMeasurement_ = 0.0F; hasPreviousMeasurement_ = false; }
}  // namespace flight

