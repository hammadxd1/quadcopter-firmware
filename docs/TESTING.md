# Testing record

## Phase 0 acceptance checklist

- [x] PlatformIO build completes using the pinned platform.
- [ ] On reset and after firmware startup, all motor GPIOs are LOW.
- [ ] No motor or PWM test is performed with propellers installed.

## Phase 0 build record

- Date: 2026-11-06
- Environment: `esp32dev`, PlatformIO Espressif32 `6.12.0`, Arduino framework.
- Result: build completed and generated `firmware.bin` (269,472 bytes) and
  `firmware.elf`.
- Static image size: 198,669 bytes text, 70,696 bytes data, and 4,937 bytes BSS.

Hardware and unit-test procedures are added with their corresponding subsystem.

## Phase 1–2 build record

- Date: 2026-11-06
- Environment: `esp32dev`, PlatformIO Espressif32 `6.12.0`, Arduino framework.
- Result: build completed successfully.
- Image size: 295,569 bytes flash (22.6% of the configured application
  partition) and 22,044 bytes RAM (6.7% of available RAM).

## Phase 3–4 test procedure

1. With propellers removed, keep the craft level after calibration and verify
   `fusion=valid` in the serial report.
2. Rotate slowly around roll and pitch. Reported angle signs must match the
   physical rotation. Correct only the explicit axis-sign configuration.
3. Disconnect either IMU with power removed and repeat startup. No fused output
   may be accepted.
4. PID and mixer calculations remain offline from PWM. Validate mixer signs
   with simulated inputs before they are connected to safety-gated output.

## Phase 3–4 build record

- Date: 2026-11-06
- Result: build completed successfully with the pinned ESP32 toolchain.
- Image size: 298,021 bytes flash (22.7%) and 22,148 bytes RAM (6.8%).

## Phase 1 propeller-off procedure

1. Remove all propellers and secure the frame.
2. Flash the firmware and verify each motor output is electrically low at boot.
3. Verify all four PWM outputs are 20 kHz with zero duty using an oscilloscope
   or logic analyzer. No motor-spin command is implemented in this phase.
4. Record the result; individual motor spin/mapping testing is deferred until
   the explicit safety and bench-test interface exists.

## Phase 2 IMU procedure

1. Keep the airframe stationary and level for startup calibration.
2. Confirm serial output reports both IMUs as healthy with increasing read
   counts and zero failures.
3. Disconnect one module with power removed, then repeat startup: the affected
   IMU must fail initialization and motors must remain disabled.
4. Record the observed I2C addresses, calibration result, and read-failure
   counters. Fusion and attitude tests begin in Phase 3.

