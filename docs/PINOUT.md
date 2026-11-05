# Pinout

| Function | GPIO | Notes |
|---|---:|---|
| I2C SDA | 21 | MPU6050 #1 and #2 shared bus |
| I2C SCL | 22 | MPU6050 #1 and #2 shared bus |
| M1 PWM | 25 | Front-right, CCW |
| M2 PWM | 26 | Front-left, CW |
| M3 PWM | 27 | Rear-right, CW |
| M4 PWM | 14 | Rear-left, CCW |

No IMU interrupt line is assigned in V1. Sampling will be scheduled by the
flight-control task.

