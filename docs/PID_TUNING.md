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
| FL | Front-left, GPIO 13 | CW |
| FR | Front-right, GPIO 14 | CCW |
| RL | Rear-left, GPIO 27 | CCW |
| RR | Rear-right, GPIO 33 | CW |

Positive roll increases left motors; positive pitch increases rear motors.
Each mixed output is limited to the normalized 0–1 range.

