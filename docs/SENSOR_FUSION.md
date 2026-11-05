# Dual-IMU fusion and attitude estimation

Each MPU6050 is acquired, scaled, timestamped, calibrated, and health-tracked
independently. Fusion never averages raw register values.

## Acceptance gate

A fused sample is produced only when both calibrated samples are valid, their
timestamps are within the configured skew limit, and their accelerometer and
gyro vector disagreement are within configured limits. There is no single-IMU
fallback in V1.

The initial weights are 0.5/0.5. Stationary-noise and controlled-rotation test
data must justify any later weighting change.

## Complementary filter

Gyroscope roll/pitch integration provides the short-term estimate. The
accelerometer tilt estimate corrects it only when measured acceleration is
close to 1 g. This avoids treating translational acceleration as tilt.

The X-forward mounting and roll/pitch signs must be verified by manual bench
rotation before controllers can be enabled.

