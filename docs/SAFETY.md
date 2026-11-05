# Safety baseline

Phase 0 starts with all four configured motor GPIOs driven LOW. Firmware alone
cannot guarantee an off state during ESP32 reset or boot, therefore each MOSFET
gate must retain its physical pull-down resistor.

The Phase 1 PWM driver configures all four LEDC channels at zero duty. During
Phases 1 and 2, `loop()` calls the PWM stop operation continuously and exposes
no normal code path for a non-zero PWM command.

Later phases will add an explicit arming state machine. Motors remain disabled
unless both IMUs are healthy and calibrated, pilot input is fresh and valid,
throttle is zero at arm request, and no failsafe condition is latched.

