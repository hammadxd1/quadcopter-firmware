#pragma once

#include <stdint.h>

#include "config/hardware_config.h"

namespace flight {

class MotorPwm {
 public:
  bool begin();
  bool setNormalized(hardware::MotorId motor, float output);
  void stopAll();
  bool isInitialized() const { return initialized_; }

 private:
  static constexpr uint8_t kChannelCount = hardware::kMotorCount;

  bool initialized_ = false;
};

}  // namespace flight

