# Cascaded PID architecture

The outer attitude PID converts requested roll/pitch angles into requested
roll/pitch rates. Inner rate PIDs compare those rates with gyroscope rates and
produce roll/pitch/yaw control demands. Yaw is rate-controlled directly.

Every PID supports output limits, integral limits, conditional anti-windup,
derivative-on-measurement, optional derivative filtering, and reset.

All default gains are zero. Phase 4 implements the controller structure; it
does not claim flight-ready gains.

## Quad-X mixer

| Output | Position | Rotation |
|---|---|---|
| M1 | Front-right | CCW |
| M2 | Front-left | CW |
| M3 | Rear-right | CW |
| M4 | Rear-left | CCW |

Positive roll increases left motors; positive pitch increases rear motors.
Each mixed output is limited to the normalized 0–1 range.

