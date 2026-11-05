#pragma once

#include "sensors/imu_manager/imu_types.h"

namespace flight {
struct AttitudeEstimate {
  float rollRad = 0.0F;
  float pitchRad = 0.0F;
  Vector3f angularRateRadPerSec{};
  uint64_t timestampUs = 0U;
  bool valid = false;
};
struct ComplementaryFilterConfiguration {
  constexpr ComplementaryFilterConfiguration() = default;
  constexpr ComplementaryFilterConfiguration(float timeConstant,
                                             float correctionMinimum,
                                             float correctionMaximum,
                                             float maximumDt,
                                             float rollSign, float pitchSign,
                                             float yawSign)
      : timeConstantSec(timeConstant),
        accelCorrectionMinimumMps2(correctionMinimum),
        accelCorrectionMaximumMps2(correctionMaximum),
        maximumDtSec(maximumDt),
        rollAxisSign(rollSign),
        pitchAxisSign(pitchSign),
        yawAxisSign(yawSign) {}

  float timeConstantSec = 0.0F;
  float accelCorrectionMinimumMps2 = 0.0F;
  float accelCorrectionMaximumMps2 = 0.0F;
  float maximumDtSec = 0.0F;
  float rollAxisSign = 1.0F;
  float pitchAxisSign = 1.0F;
  float yawAxisSign = 1.0F;
};
class ComplementaryFilter {
 public:
  explicit ComplementaryFilter(ComplementaryFilterConfiguration configuration);
  bool update(const ImuSample& fusedMeasurement, float dtSec, AttitudeEstimate& estimate);
  void reset();
 private:
  ComplementaryFilterConfiguration configuration_;
  bool initialized_ = false;
  AttitudeEstimate state_{};
};
}  // namespace flight

