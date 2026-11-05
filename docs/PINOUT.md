# Pinout

| Function | GPIO | Notes |
|---|---:|---|
| MPU6050 #1 SDA | 21 | I2C controller 0, address `0x68` |
| MPU6050 #1 SCL | 22 | I2C controller 0, 400 kHz |
| MPU6050 #2 SDA | 25 | I2C controller 1, address `0x69` |
| MPU6050 #2 SCL | 26 | I2C controller 1, 400 kHz |
| Motor FL PWM | 13 | Front-left, CW, 22 kHz |
| Motor FR PWM | 14 | Front-right, CCW, 22 kHz |
| Motor RL PWM | 27 | Rear-left, CCW, 22 kHz |
| Motor RR PWM | 33 | Rear-right, CW, 22 kHz |

No IMU interrupt line is assigned in V1. Sampling will be scheduled by the
flight-control task.

