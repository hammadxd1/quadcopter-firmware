# Hardware record

## Power

The aircraft uses a 1S LiPo motor rail (3.7–4.2 V) and a regulated 5 V boost
rail connected to the ESP32 DevKit V1 VIN/5V input. ESP32 logic and both IMUs
operate from the board's regulated 3.3 V domain. All grounds are common.

## Motors and driver assumptions

Four 3.7 V 8520 brushed coreless motors are switched independently by low-side
SI2300DS N-channel MOSFET channels. Each channel requires a gate pull-down and
motor flyback path. The installed flyback diode type is 1N5819-class Schottky.

Measured motor stall current, diode temperature, MOSFET temperature, and boost
converter margin remain formal hardware-test records; they are not inferred by
this document.

## IMUs

The two MPU6050 boards use independent ESP32 I2C controllers. MPU6050 #1 is
at `0x68` on controller 0 (SDA GPIO 21, SCL GPIO 22); MPU6050 #2 is at `0x69`
on controller 1 (SDA GPIO 25, SCL GPIO 26). Both buses run at 400 kHz. Both
boards are mounted flat and identically, with printed X forward.

The initial driver configures both sensors for ±8 g acceleration, ±1000 deg/s
gyro range, 42 Hz digital filtering, and a nominal 250 Hz sensor output rate.
These are configuration values pending logged vibration and rotation tests, not
flight-tuned values.

