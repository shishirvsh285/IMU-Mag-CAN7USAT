# adaptive_mahony_uno

The adaptive Mahony filter running on the Arduino Uno itself, reading the BMI270 and IIS2MDC.

## Files

| File | What it is |
|---|---|
| `adaptive_mahony_uno.ino` | Reads both chips at 50 Hz, applies the calibration, runs the filter, prints angles |
| `AdaptiveMahony.h` | Copy of the filter (`../AdaptiveMahony/AdaptiveMahony.h`) |
| `imu_cal.h` | Calibration constants. **The one here is a placeholder with no calibration.** |
| `bmi270_config.h` | Bosch config blob, needed to start the BMI270 |

## Setup

1. Run `python full_calibration.py --port COM5` (with `motioncal_stream` on the Uno).
2. Copy the `imu_cal.h` it writes into this folder, over the placeholder.
3. Upload this sketch. Keep the board still for about 2 s after reset.

`imu_cal.h` holds `GYRO_BIAS_DPS`, `ACC_OFFSET_G`, `ACC_GAIN`, `MAG_HARD_MG`, `MAG_SOFT`, `MAG_MAP`,
`USE_MAG`, `MAG_DIP_DEG` and `MAG_FIELD_MG`. With the placeholder, `USE_MAG` is 0, so the filter runs
6-DOF and yaw drifts slowly.

## Output (115200 baud)

```
roll,pitch,yaw,kp,wa,us
-54.70,29.76,-16.88,0.632,0.918,2804
```

| Column | Meaning |
|---|---|
| `roll,pitch,yaw` | degrees, ZYX Euler, body to world, world Z up |
| `kp` | current accelerometer gain (1/s) |
| `wa` | accelerometer trust, 0 (useless) to 1 (clean gravity) |
| `us` | time the filter update took, in microseconds (about 2800 on an Uno) |

Uses 25 kB of flash and 629 B of RAM.
