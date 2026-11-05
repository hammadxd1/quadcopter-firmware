#pragma once
#include "flight/flight_controller/cascaded_controller.h"
namespace flight {
struct MotorOutputs { float frontRight = 0.0F; float frontLeft = 0.0F; float rearRight = 0.0F; float rearLeft = 0.0F; };
class QuadXMixer { public: MotorOutputs mix(const ControlDemand& demand) const; };
}  // namespace flight

