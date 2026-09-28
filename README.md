# Gyroscope — Stationary Bias Calibration

Standalone ICM-20948 gyroscope example for STM32F303K8.

## What it does

- Verifies ICM-20948 with `WHO_AM_I = 0xEA`
- Uses the ±250 °/s gyroscope range
- Averages stationary raw samples
- Computes X/Y/Z zero-rate bias
- Outputs corrected angular rate

For calibration, keep the board completely still on a stable surface.
