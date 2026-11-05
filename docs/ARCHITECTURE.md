# Architecture

The firmware is organized around a 250 Hz flight-control task that owns sensor
acquisition, dual-IMU fusion, attitude estimation, PID control, motor mixing,
and PWM updates. Wi-Fi and the web interface run independently and publish only
validated pilot-command snapshots to the flight task.

Phase 1 provides an LEDC PWM driver whose initialization always writes zero
duty to all channels. Phase 2 provides two independent MPU6050 driver objects,
one per I2C address, with independent calibration and health state. Phase 3
adds validity-gated weighted fusion followed by a complementary roll/pitch
filter. Phase 4 adds cascaded attitude/rate PID and Quad-X mixing as isolated
calculation modules. No controller output reaches PWM until a later safety
phase explicitly authorizes it.

## Planned ownership

- `drivers`: hardware-facing dual-I2C, MPU6050, and PWM code.
- `sensors`: independent IMU calibration and health monitoring.
- `estimation`: calibrated measurement fusion and complementary attitude filter.
- `controllers` and `flight`: cascaded PID and Quad-X mixing.
- `communication`: access point, WebSocket, and command validation.
- `safety`: arming, timeout, sensor, and timing safety gates.

