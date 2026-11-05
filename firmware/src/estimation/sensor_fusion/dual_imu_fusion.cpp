#include "estimation/sensor_fusion/dual_imu_fusion.h"

#include <math.h>

namespace flight {
namespace {
float magnitude(const Vector3f& value) {
  return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}
Vector3f subtract(const Vector3f& a, const Vector3f& b) {
  return Vector3f(a.x - b.x, a.y - b.y, a.z - b.z);
}
Vector3f weightedAverage(const Vector3f& a, const Vector3f& b, float wa,
                          float wb) {
  return Vector3f(a.x * wa + b.x * wb, a.y * wa + b.y * wb,
                  a.z * wa + b.z * wb);
}
uint64_t absoluteDifference(uint64_t a, uint64_t b) { return a >= b ? a - b : b - a; }
}  // namespace

DualImuFusion::DualImuFusion(FusionConfiguration configuration)
    : configuration_(configuration) {
  const float total = configuration_.imu1Weight + configuration_.imu2Weight;
  if (total > 0.0F) {
    configuration_.imu1Weight /= total;
    configuration_.imu2Weight /= total;
  }
}

bool DualImuFusion::fuse(const DualImuSample& input, ImuSample& output,
                          FusionDiagnostics& diagnostics) const {
  output = {};
  diagnostics = {};
  if (!input.imu1.valid || !input.imu2.valid || configuration_.imu1Weight <= 0.0F ||
      configuration_.imu2Weight <= 0.0F) return false;

  diagnostics.sampleSkewUs = absoluteDifference(input.imu1.timestampUs, input.imu2.timestampUs);
  diagnostics.samplesSynchronous = diagnostics.sampleSkewUs <= configuration_.maximumSampleSkewUs;
  diagnostics.accelDisagreementMps2 = magnitude(subtract(input.imu1.accelerationMps2, input.imu2.accelerationMps2));
  diagnostics.gyroDisagreementRadPerSec = magnitude(subtract(input.imu1.angularRateRadPerSec, input.imu2.angularRateRadPerSec));
  if (!diagnostics.samplesSynchronous ||
      diagnostics.accelDisagreementMps2 > configuration_.maximumAccelDisagreementMps2 ||
      diagnostics.gyroDisagreementRadPerSec > configuration_.maximumGyroDisagreementRadPerSec) return false;

  output.accelerationMps2 = weightedAverage(input.imu1.accelerationMps2, input.imu2.accelerationMps2, configuration_.imu1Weight, configuration_.imu2Weight);
  output.angularRateRadPerSec = weightedAverage(input.imu1.angularRateRadPerSec, input.imu2.angularRateRadPerSec, configuration_.imu1Weight, configuration_.imu2Weight);
  output.temperatureC = input.imu1.temperatureC * configuration_.imu1Weight + input.imu2.temperatureC * configuration_.imu2Weight;
  output.timestampUs = input.imu1.timestampUs > input.imu2.timestampUs ? input.imu1.timestampUs : input.imu2.timestampUs;
  output.valid = true;
  diagnostics.valid = true;
  return true;
}
}  // namespace flight

