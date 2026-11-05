#pragma once

#include <stdint.h>

namespace flight::fusion_config {
constexpr float kImu1Weight = 0.5F;
constexpr float kImu2Weight = 0.5F;
constexpr uint64_t kMaximumSampleSkewUs = 2000U;
constexpr float kMaximumAccelDisagreementMps2 = 2.0F;
constexpr float kMaximumGyroDisagreementRadPerSec = 0.5F;
constexpr float kComplementaryTimeConstantSec = 0.5F;
constexpr float kAccelCorrectionMinimumMps2 = 8.0F;
constexpr float kAccelCorrectionMaximumMps2 = 11.6F;
constexpr float kMaximumEstimatorDtSec = 0.02F;
constexpr float kRollAxisSign = 1.0F;
constexpr float kPitchAxisSign = 1.0F;
constexpr float kYawAxisSign = 1.0F;
}  // namespace flight::fusion_config

