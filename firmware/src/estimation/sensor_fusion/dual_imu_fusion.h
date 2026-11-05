#pragma once

#include "sensors/imu_manager/imu_types.h"

namespace flight {
struct FusionConfiguration {
  constexpr FusionConfiguration() = default;
  constexpr FusionConfiguration(float firstWeight, float secondWeight,
                                uint64_t maximumSkewUs,
                                float maximumAccelDisagreement,
                                float maximumGyroDisagreement)
      : imu1Weight(firstWeight),
        imu2Weight(secondWeight),
        maximumSampleSkewUs(maximumSkewUs),
        maximumAccelDisagreementMps2(maximumAccelDisagreement),
        maximumGyroDisagreementRadPerSec(maximumGyroDisagreement) {}

  float imu1Weight = 0.5F;
  float imu2Weight = 0.5F;
  uint64_t maximumSampleSkewUs = 0U;
  float maximumAccelDisagreementMps2 = 0.0F;
  float maximumGyroDisagreementRadPerSec = 0.0F;
};

struct FusionDiagnostics {
  bool valid = false;
  bool samplesSynchronous = false;
  float accelDisagreementMps2 = 0.0F;
  float gyroDisagreementRadPerSec = 0.0F;
  uint64_t sampleSkewUs = 0U;
};

class DualImuFusion {
 public:
  explicit DualImuFusion(FusionConfiguration configuration);
  bool fuse(const DualImuSample& input, ImuSample& output,
            FusionDiagnostics& diagnostics) const;

 private:
  FusionConfiguration configuration_;
};
}  // namespace flight

