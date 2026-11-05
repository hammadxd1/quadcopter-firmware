# ESP32 Minimalist Quadcopter Flight Controller

Safety-first firmware for an ESP32 DevKit V1 brushed micro quadcopter. The
target is reliable self-level flight using two independently processed MPU6050
IMUs, dual-IMU measurement fusion, cascaded PID stabilization, and a minimal
local Wi-Fi controller.

## Current status

**Phases 3–4 — estimation and control mathematics.** Firmware additionally
fuses synchronized dual-IMU measurements and reports complementary-filter roll
and pitch diagnostics. Cascaded PID and Quad-X mixer modules are implemented
but remain disconnected from PWM. It contains no arming, Wi-Fi, or path that
commands non-zero motor output.

## Confirmed hardware

- ESP32 DevKit V1; Arduino framework; PlatformIO.
- MPU6050 #1 at `0x68` and MPU6050 #2 at `0x69`, sharing GPIO 21 (SDA) and
  GPIO 22 (SCL). Both use the same rigid orientation with X forward.
- 8520 brushed motors, 3.7 V nominal, 1.0 mm shaft, 55 mm matched CW/CCW
  propellers.
- Quad-X mapping: front-right CCW GPIO 25; front-left CW GPIO 26; rear-right
  CW GPIO 27; rear-left CCW GPIO 14.
- Motors are supplied from the 1S battery rail. The DevKit VIN/5V input is
  supplied by a regulated 5 V boost converter. Motor flyback protection uses
  1N5819-class Schottky diodes.

## Build

Install PlatformIO, then run:

```powershell
pio run -d firmware
```

The project pins PlatformIO Espressif32 platform `6.12.0`.

## Safety

Do not connect propellers for Phase 1/2 validation. The source-level LOW output
and zero-duty PWM are secondary safeguards; each MOSFET gate must also have its
hardware pull-down resistor fitted.

