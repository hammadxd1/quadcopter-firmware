#include "drivers/mpu6050/mpu6050.h"

#include <esp_timer.h>
#include <math.h>

#include "config/imu_config.h"

namespace flight {
namespace {

int16_t makeInt16(const uint8_t msb, const uint8_t lsb) {
  return static_cast<int16_t>((static_cast<uint16_t>(msb) << 8U) | lsb);
}

float vectorMagnitude(const Vector3f& value) {
  return sqrtf(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vector3f add(const Vector3f& left, const Vector3f& right) {
  return {left.x + right.x, left.y + right.y, left.z + right.z};
}

Vector3f divide(const Vector3f& value, const float divisor) {
  return {value.x / divisor, value.y / divisor, value.z / divisor};
}

}  // namespace

Mpu6050::Mpu6050(TwoWire& wire, const uint8_t address)
    : wire_(wire), address_(address) {}

bool Mpu6050::begin() {
  uint8_t whoAmI = 0U;
  if (!readRegisters(imu_config::kWhoAmIRegister, &whoAmI, 1U) ||
      whoAmI != imu_config::kExpectedWhoAmI) {
    health_.initialized = false;
    health_.healthy = false;
    return false;
  }

  const bool configured =
      writeRegister(imu_config::kPowerManagement1Register, 0x00U) &&
      writeRegister(imu_config::kConfigRegister, imu_config::kDlpFilterBits) &&
      writeRegister(imu_config::kSampleRateDividerRegister,
                    imu_config::kSampleRateDivider) &&
      writeRegister(imu_config::kGyroConfigRegister,
                    imu_config::kGyroRangeBits) &&
      writeRegister(imu_config::kAccelConfigRegister,
                    imu_config::kAccelRangeBits);

  health_.initialized = configured;
  health_.healthy = configured;
  health_.calibrated = false;
  return configured;
}

bool Mpu6050::calibrateStationary() {
  if (!health_.initialized) {
    return false;
  }

  Vector3f gyroSum{};
  Vector3f accelSum{};
  uint16_t acceptedSamples = 0U;

  for (uint16_t index = 0U; index < imu_config::kCalibrationSampleCount;
       ++index) {
    ImuSample rawSample{};
    if (!readUncalibrated(rawSample)) {
      continue;
    }

    const float accelerationMagnitude = vectorMagnitude(rawSample.accelerationMps2);
    if (fabsf(accelerationMagnitude - imu_config::kGravityMps2) >
        imu_config::kStationaryAccelToleranceMps2) {
      continue;
    }

    gyroSum = add(gyroSum, rawSample.angularRateRadPerSec);
    accelSum = add(accelSum, rawSample.accelerationMps2);
    ++acceptedSamples;
    delay(imu_config::kCalibrationSampleDelayMs);
  }

  if (acceptedSamples < imu_config::kCalibrationSampleCount) {
    health_.calibrated = false;
    health_.healthy = false;
    return false;
  }

  gyroBiasRadPerSec_ = divide(gyroSum, static_cast<float>(acceptedSamples));
  const Vector3f meanAcceleration =
      divide(accelSum, static_cast<float>(acceptedSamples));
  // The +Z stationary gravity sign is explicitly bench-verified before the
  // fused attitude estimate is permitted to arm.
  accelBiasMps2_ = {meanAcceleration.x, meanAcceleration.y,
                    meanAcceleration.z - imu_config::kGravityMps2};

  health_.calibrated = true;
  health_.healthy = true;
  return true;
}

bool Mpu6050::readSample(ImuSample& sample) {
  sample = {};
  if (!health_.initialized || !health_.calibrated ||
      !readUncalibrated(sample)) {
    return false;
  }

  sample.accelerationMps2.x -= accelBiasMps2_.x;
  sample.accelerationMps2.y -= accelBiasMps2_.y;
  sample.accelerationMps2.z -= accelBiasMps2_.z;
  sample.angularRateRadPerSec.x -= gyroBiasRadPerSec_.x;
  sample.angularRateRadPerSec.y -= gyroBiasRadPerSec_.y;
  sample.angularRateRadPerSec.z -= gyroBiasRadPerSec_.z;
  sample.valid = true;
  return true;
}

bool Mpu6050::writeRegister(const uint8_t reg, const uint8_t value) {
  wire_.beginTransmission(address_);
  wire_.write(reg);
  wire_.write(value);
  if (wire_.endTransmission(true) != 0U) {
    noteReadFailure();
    return false;
  }
  return true;
}

bool Mpu6050::readRegisters(const uint8_t startRegister, uint8_t* destination,
                             const size_t length) {
  wire_.beginTransmission(address_);
  wire_.write(startRegister);
  if (wire_.endTransmission(false) != 0U) {
    noteReadFailure();
    return false;
  }

  const size_t received = wire_.requestFrom(address_, length, true);
  if (received != length) {
    while (wire_.available() > 0) {
      wire_.read();
    }
    noteReadFailure();
    return false;
  }

  for (size_t index = 0U; index < length; ++index) {
    destination[index] = static_cast<uint8_t>(wire_.read());
  }
  return true;
}

bool Mpu6050::readUncalibrated(ImuSample& sample) {
  uint8_t bytes[14]{};
  if (!readRegisters(imu_config::kAccelDataRegister, bytes, sizeof(bytes))) {
    return false;
  }

  const int16_t accelX = makeInt16(bytes[0], bytes[1]);
  const int16_t accelY = makeInt16(bytes[2], bytes[3]);
  const int16_t accelZ = makeInt16(bytes[4], bytes[5]);
  const int16_t temperature = makeInt16(bytes[6], bytes[7]);
  const int16_t gyroX = makeInt16(bytes[8], bytes[9]);
  const int16_t gyroY = makeInt16(bytes[10], bytes[11]);
  const int16_t gyroZ = makeInt16(bytes[12], bytes[13]);

  sample.accelerationMps2 = {
      accelX * imu_config::kAccelScaleMps2PerLsb,
      accelY * imu_config::kAccelScaleMps2PerLsb,
      accelZ * imu_config::kAccelScaleMps2PerLsb,
  };
  sample.angularRateRadPerSec = {
      gyroX * imu_config::kGyroScaleRadPerSecPerLsb,
      gyroY * imu_config::kGyroScaleRadPerSecPerLsb,
      gyroZ * imu_config::kGyroScaleRadPerSecPerLsb,
  };
  sample.temperatureC = temperature / 340.0F + 36.53F;
  sample.timestampUs = static_cast<uint64_t>(esp_timer_get_time());
  sample.valid = true;
  noteReadSuccess(sample.timestampUs);
  return true;
}

void Mpu6050::noteReadFailure() {
  ++health_.failedReads;
  ++health_.consecutiveFailures;
  health_.healthy = false;
}

void Mpu6050::noteReadSuccess(const uint64_t timestampUs) {
  ++health_.successfulReads;
  health_.consecutiveFailures = 0U;
  health_.lastSampleTimestampUs = timestampUs;
  health_.healthy = true;
}

}  // namespace flight

