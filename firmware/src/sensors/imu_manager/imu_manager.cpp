#include "sensors/imu_manager/imu_manager.h"

#include "config/hardware_config.h"
#include "config/imu_config.h"

namespace flight {

ImuManager::ImuManager(TwoWire& imu1Bus, TwoWire& imu2Bus)
    : imu1Bus_(imu1Bus),
      imu2Bus_(imu2Bus),
      imu1_(imu1Bus, hardware::kImu1Address),
      imu2_(imu2Bus, hardware::kImu2Address) {}

bool ImuManager::begin() {
  imu1Bus_.begin(hardware::kImu1SdaPin, hardware::kImu1SclPin,
                 hardware::kI2cClockHz);
  imu2Bus_.begin(hardware::kImu2SdaPin, hardware::kImu2SclPin,
                 hardware::kI2cClockHz);
  imu1Bus_.setTimeOut(imu_config::kI2cTimeoutMs);
  imu2Bus_.setTimeOut(imu_config::kI2cTimeoutMs);
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

