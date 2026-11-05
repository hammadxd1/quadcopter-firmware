#include <Arduino.h>

#include "config/hardware_config.h"
#include "config/fusion_config.h"
#include "drivers/motor/motor_pwm.h"
#include "estimation/complementary_filter/complementary_filter.h"
#include "estimation/sensor_fusion/dual_imu_fusion.h"
#include "sensors/imu_manager/imu_manager.h"

namespace {

flight::MotorPwm motorPwm;
flight::ImuManager imuManager(Wire);
flight::DualImuFusion fusion({flight::fusion_config::kImu1Weight,
                               flight::fusion_config::kImu2Weight,
                               flight::fusion_config::kMaximumSampleSkewUs,
                               flight::fusion_config::kMaximumAccelDisagreementMps2,
                               flight::fusion_config::kMaximumGyroDisagreementRadPerSec});
flight::ComplementaryFilter complementaryFilter(
    {flight::fusion_config::kComplementaryTimeConstantSec,
     flight::fusion_config::kAccelCorrectionMinimumMps2,
     flight::fusion_config::kAccelCorrectionMaximumMps2,
     flight::fusion_config::kMaximumEstimatorDtSec,
     flight::fusion_config::kRollAxisSign,
     flight::fusion_config::kPitchAxisSign,
     flight::fusion_config::kYawAxisSign});
uint32_t lastReportMs = 0U;
uint64_t previousFusionTimestampUs = 0U;

void initializeSafeMotorOutputs() {
  // Hardware gate pulldowns are also required: GPIOs can be high impedance
  // before firmware reaches setup().
  for (const uint8_t pin : flight::hardware::kMotorPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
}

}  // namespace

void setup() {
  initializeSafeMotorOutputs();

  Serial.begin(115200);
  Serial.println("ESP32 flight controller: Phase 1/2 safe startup");

  if (!motorPwm.begin()) {
    Serial.println("ERROR: PWM initialization failed; motors remain disabled.");
    return;
  }
  motorPwm.stopAll();

  if (!imuManager.begin()) {
    Serial.println("ERROR: dual-IMU initialization failed; motors remain disabled.");
    return;
  }

  Serial.println("Keep the aircraft stationary: calibrating both IMUs.");
  if (!imuManager.calibrateBothStationary()) {
    Serial.println("ERROR: dual-IMU calibration failed; motors remain disabled.");
    return;
  }

  Serial.println("Phase 2 IMU diagnostics ready; motors remain disabled.");
}

void loop() {
  motorPwm.stopAll();

  flight::DualImuSample samples{};
  flight::ImuSample fused{};
  flight::FusionDiagnostics fusionDiagnostics{};
  flight::AttitudeEstimate attitude{};
  if (imuManager.readBoth(samples) && fusion.fuse(samples, fused, fusionDiagnostics)) {
    if (previousFusionTimestampUs != 0U && fused.timestampUs > previousFusionTimestampUs) {
      const float dtSec = static_cast<float>(fused.timestampUs - previousFusionTimestampUs) / 1000000.0F;
      complementaryFilter.update(fused, dtSec, attitude);
    }
    previousFusionTimestampUs = fused.timestampUs;
  }

  const uint32_t nowMs = millis();
  if (nowMs - lastReportMs >= 1000U) {
    lastReportMs = nowMs;
    Serial.printf("IMU1=%s IMU2=%s reads=%lu/%lu failures=%lu/%lu ",
                  imuManager.imu1Health().healthy ? "healthy" : "fault",
                  imuManager.imu2Health().healthy ? "healthy" : "fault",
                  static_cast<unsigned long>(imuManager.imu1Health().successfulReads),
                  static_cast<unsigned long>(imuManager.imu2Health().successfulReads),
                  static_cast<unsigned long>(imuManager.imu1Health().failedReads),
                  static_cast<unsigned long>(imuManager.imu2Health().failedReads));
    Serial.printf("fusion=%s skew=%llu accelDiff=%.3f gyroDiff=%.3f ",
                  fusionDiagnostics.valid ? "valid" : "rejected",
                  static_cast<unsigned long long>(fusionDiagnostics.sampleSkewUs),
                  fusionDiagnostics.accelDisagreementMps2,
                  fusionDiagnostics.gyroDisagreementRadPerSec);
    Serial.printf("roll=%.3f pitch=%.3f\n", attitude.rollRad, attitude.pitchRad);
  }

  // This is a diagnostics loop, not the Phase 5 real-time control loop.
  delay(20);
}

