#pragma once

namespace flight {
struct PidGains {
  float proportional = 0.0F;
  float integral = 0.0F;
  float derivative = 0.0F;
  float feedForward = 0.0F;
  float integralLimit = 0.0F;
  float outputLimit = 0.0F;
  float derivativeFilterTimeConstantSec = 0.0F;
};
class PidController {
 public:
  explicit PidController(PidGains gains);
  float update(float setpoint, float measurement, float dtSec);
  void reset();
 private:
  PidGains gains_;
  float integralState_ = 0.0F;
  float filteredMeasurementDerivative_ = 0.0F;
  float previousMeasurement_ = 0.0F;
  bool hasPreviousMeasurement_ = false;
};
}  // namespace flight

