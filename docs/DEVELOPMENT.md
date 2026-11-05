# Development workflow

Work proceeds only through approved phases. Each phase includes source review,
build/test evidence, documentation updates, and project-owner approval before
the next major subsystem.

The initial toolchain target is PlatformIO Espressif32 `6.12.0`, Arduino
framework, and board ID `esp32dev`. External firmware dependencies are not used
in Phases 0–2.

## Current hardware configuration source of truth

The canonical pin and bus assignments are defined in
`firmware/src/config/hardware_config.h`. Current assignments are two separate
400 kHz I2C controllers (IMU1: GPIO 21/22, IMU2: GPIO 25/26) and four 22 kHz
motor PWM outputs (FL 13, FR 14, RL 27, RR 33). Future firmware changes must
update that header first, then the dependent drivers and this documentation.

