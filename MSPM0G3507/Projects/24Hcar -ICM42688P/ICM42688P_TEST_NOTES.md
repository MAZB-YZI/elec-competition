# ICM42688P test notes

## Snapshot: first I2C yaw test

- Wiring: VCC=5V, GND=GND, SDA=PA0, SCL=PA1, AD0=GND, CS pulled high.
- I2C address: `0x68` (`A104` on OLED).
- WHO_AM_I: `0x47` (`W071` on OLED).
- I2C error count: stable at `0`.
- Static Z rate: about `0.0 dps`.

## Observed behavior

- Clockwise 90 degree hand turn: about `Y=-935`, close to `-93.5 deg`.
- Returning to original heading can leave about `+10 deg` error when turning fast.
- Very small rotations may be insensitive, or show tiny movement in the opposite direction.

## Suspected causes

- Current test code applies yaw low-pass filtering and a gyro deadband before integration.
- The filter/deadband may improve stillness but can hurt fast turns and tiny-angle response.
- Next test should try raw yaw integration: no low-pass, no deadband, keep short calibration.
