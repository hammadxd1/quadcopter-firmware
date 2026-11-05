#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "sensors/imu_manager/imu_types.h"

namespace flight {

class Mpu6050 {
 public:
  Mpu6050(TwoWire& wire, uint8_t address);

  bool begin();
  bool calibrateStationary();
  bool readSample(ImuSample& sample);

  uint8_t address() const { return address_; }
  const ImuHealth& health() const { return health_; }

 private:
  bool writeRegister(uint8_t reg, uint8_t value);
  bool readRegisters(uint8_t startRegister, uint8_t* destination,
                     size_t length);
  bool readUncalibrated(ImuSample& sample);
  void noteReadFailure();
  void noteReadSuccess(uint64_t timestampUs);

  TwoWire& wire_;
  uint8_t address_;
  Vector3f gyroBiasRadPerSec_{};
  Vector3f accelBiasMps2_{};
  ImuHealth health_{};
};

}  // namespace flight

