#pragma once

#include <Arduino.h>
#include <Wire.h>

#include "drivers/mpu6050/mpu6050.h"
#include "sensors/imu_manager/imu_types.h"

namespace flight {

class ImuManager {
 public:
  explicit ImuManager(TwoWire& wire);

  bool begin();
  bool calibrateBothStationary();
  bool readBoth(DualImuSample& samples);

  const ImuHealth& imu1Health() const { return imu1_.health(); }
  const ImuHealth& imu2Health() const { return imu2_.health(); }
  bool allHealthy() const;

 private:
  TwoWire& wire_;
  Mpu6050 imu1_;
  Mpu6050 imu2_;
};

}  // namespace flight

