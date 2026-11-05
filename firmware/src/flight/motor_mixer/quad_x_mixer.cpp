#include "flight/motor_mixer/quad_x_mixer.h"
#include <math.h>
namespace flight { namespace {
float clampUnit(float value) { if (!isfinite(value) || value <= 0.0F) return 0.0F; return value >= 1.0F ? 1.0F : value; }
}  // namespace
MotorOutputs QuadXMixer::mix(const ControlDemand& demand) const {
  if (!isfinite(demand.throttle) || demand.throttle <= 0.0F) return {};
  // Front-right CCW, front-left CW, rear-right CW, rear-left CCW.
  MotorOutputs output{};
  output.frontRight = clampUnit(demand.throttle - demand.roll - demand.pitch + demand.yaw);
  output.frontLeft = clampUnit(demand.throttle + demand.roll - demand.pitch - demand.yaw);
  output.rearRight = clampUnit(demand.throttle - demand.roll + demand.pitch - demand.yaw);
  output.rearLeft = clampUnit(demand.throttle + demand.roll + demand.pitch + demand.yaw);
  return output;
}
}  // namespace flight
