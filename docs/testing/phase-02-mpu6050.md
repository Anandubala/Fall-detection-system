# Phase 2: MPU6050 verification
Objective: resolve WHO_AM_I = 0x70 and confirm valid, fresh data.
Hardware: ESP32 DevKit, GY-521-style module, I2C GPIO21/22, address 0x68.
Procedure: firmware/tests/mpu_identity_test, board still, USB power.
Measured results (2026-10-09):
- WHO_AM_I over 50 reads: min=max=0x70
- PWR_MGMT_1=0x00, GYRO_CFG=0x08, ACCEL_CFG=0x10 (all read back as written)
- Reg 0x1D: wrote 0x05, read back 0x05
- Temp raw 3430 (MPU-6500 formula 31.3 C, MPU-6050 formula 46.6 C)
- 200 samples at 50 Hz in 3982 ms: ok=200 fail=0 identical=0
Calibration check (Phase 3 run): |a| 0.997 to 1.003 g, |gyro| 0.11 to 0.27 dps over 8 samples.
Result: PASS. Module behaves like an MPU-6500-family chip. Register code and calibration unchanged.
Open: accel bandwidth vs 50 Hz sampling to be decided with recorded data.
Evidence: Serial Monitor output saved with this file.