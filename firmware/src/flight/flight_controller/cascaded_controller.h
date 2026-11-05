#pragma once

#include "controllers/pid/pid_controller.h"
#include "estimation/complementary_filter/complementary_filter.h"

namespace flight {
struct PilotAttitudeCommand { float throttle = 0.0F; float rollAngleRad = 0.0F; float pitchAngleRad = 0.0F; float yawRateRadPerSec = 0.0F; };
struct CascadedControllerConfiguration { PidGains rollAttitude{}; PidGains pitchAttitude{}; PidGains rollRate{}; PidGains pitchRate{}; PidGains yawRate{}; };
struct ControlDemand { float throttle = 0.0F; float roll = 0.0F; float pitch = 0.0F; float yaw = 0.0F; };
class CascadedController {
 public:
  explicit CascadedController(CascadedControllerConfiguration configuration);
  ControlDemand update(const PilotAttitudeCommand& command, const AttitudeEstimate& attitude, float dtSec);
  void reset();
 private:
  PidController rollAttitude_; PidController pitchAttitude_; PidController rollRate_; PidController pitchRate_; PidController yawRate_;
};
}  // namespace flight

