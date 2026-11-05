#include "sensors/imu_manager/imu_manager.h"

#include "config/hardware_config.h"
#include "config/imu_config.h"

namespace flight {

ImuManager::ImuManager(TwoWire& wire)
    : wire_(wire),
      imu1_(wire, hardware::kImu1Address),
      imu2_(wire, hardware::kImu2Address) {}

bool ImuManager::begin() {
  wire_.begin(hardware::kI2cSdaPin, hardware::kI2cSclPin,
              hardware::kI2cClockHz);
  wire_.setTimeOut(imu_config::kI2cTimeoutMs);
  // Do not short-circuit: both sources must be independently attempted and
  // retain their own diagnostic state even when the first device fails.
  const bool imu1Initialized = imu1_.begin();
  const bool imu2Initialized = imu2_.begin();
  return imu1Initialized && imu2Initialized;
}

bool ImuManager::calibrateBothStationary() {
  // Calibration is intentionally independent so each bias and failure state is
  // preserved. Motors remain disabled for the whole Phase 2 startup sequence.
  const bool imu1Calibrated = imu1_.calibrateStationary();
  const bool imu2Calibrated = imu2_.calibrateStationary();
  return imu1Calibrated && imu2Calibrated;
}

bool ImuManager::readBoth(DualImuSample& samples) {
  const bool imu1Read = imu1_.readSample(samples.imu1);
  const bool imu2Read = imu2_.readSample(samples.imu2);
  return imu1Read && imu2Read;
}

bool ImuManager::allHealthy() const {
  const ImuHealth& first = imu1_.health();
  const ImuHealth& second = imu2_.health();
  return first.initialized && first.calibrated && first.healthy &&
         second.initialized && second.calibrated && second.healthy;
}

}  // namespace flight

