#include "flight/flight_controller/cascaded_controller.h"
namespace flight {
CascadedController::CascadedController(CascadedControllerConfiguration c)
    : rollAttitude_(c.rollAttitude), pitchAttitude_(c.pitchAttitude), rollRate_(c.rollRate), pitchRate_(c.pitchRate), yawRate_(c.yawRate) {}
ControlDemand CascadedController::update(const PilotAttitudeCommand& command, const AttitudeEstimate& attitude, float dtSec) {
  if (!attitude.valid) { reset(); return {}; }
  const float wantedRollRate = rollAttitude_.update(command.rollAngleRad, attitude.rollRad, dtSec);
  const float wantedPitchRate = pitchAttitude_.update(command.pitchAngleRad, attitude.pitchRad, dtSec);
  ControlDemand output{};
  output.throttle = command.throttle;
  output.roll = rollRate_.update(wantedRollRate, attitude.angularRateRadPerSec.x, dtSec);
  output.pitch = pitchRate_.update(wantedPitchRate, attitude.angularRateRadPerSec.y, dtSec);
  output.yaw = yawRate_.update(command.yawRateRadPerSec, attitude.angularRateRadPerSec.z, dtSec);
  return output;
}
void CascadedController::reset() { rollAttitude_.reset(); pitchAttitude_.reset(); rollRate_.reset(); pitchRate_.reset(); yawRate_.reset(); }
}  // namespace flight

