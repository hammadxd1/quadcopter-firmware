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

Two MPU6050 boards share one I2C bus. Their AD0 wiring selects addresses `0x68`
and `0x69`. Both are mounted flat and identically, with printed X forward.

The initial driver configures both sensors for ±8 g acceleration, ±1000 deg/s
gyro range, 42 Hz digital filtering, and a nominal 250 Hz sensor output rate.
These are configuration values pending logged vibration and rotation tests, not
flight-tuned values.

