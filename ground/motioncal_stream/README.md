# motioncal_stream

The main streaming sketch. Reads both sensors and prints raw values about 50 times a second, in the
format the PJRC MotionCal tool expects. Most of the Python tools in this repo read this stream.

## Output (115200 baud)

```
Raw:ax,ay,az,gx,gy,gz,mx,my,mz
Raw:-2176,5614,5414,1,2,-1,-59,101,-452
```

| Field | Unit | Note |
|---|---|---|
| `ax ay az` | 8192 counts per g | BMI270 at ±8 g, rescaled from 4096 counts/g. Not clamped, so values go past ±32767 above 4 g |
| `gx gy gz` | 16 counts per °/s | BMI270 at ±2000 °/s, rescaled from 16.384. X has the datasheet cross-axis correction applied |
| `mx my mz` | 1 count = 1 mG = 0.1 µT | IIS2MDC, 1.5 mG/LSB rescaled |

Each chip is in its **own** axes, unchanged. The IIS2MDC frame is left-handed, so its axes do not match
the BMI270. `full_calibration.py` finds the mapping.

Lines end in CR LF, because MotionCal only ends a line on CR. MotionCal stores values as 16-bit, so above 4 g
its accel display wraps. That does not matter for magnetometer calibration, which is done gently.

## Sensor settings

- BMI270: config blob uploaded, performance mode, accel and gyro both at **50 Hz** (`ACC_CONF = 0xA7`,
  `GYR_CONF = 0xE7`), so the sensor rate matches the 50 Hz read rate and nothing aliases.
- Gyro cross-axis post-processing (BMI270 datasheet 4.6.10): the chip's `GYR_CAS.factor_zx` is read at start-up
  and applied as $\omega_x \mathrel{-}= \text{factor}_{zx}\,\omega_z / 2^9$. The sketch prints it once as
  `# GYR_CAS factor_zx = N` (ours is 2, that is 0.39 % of Z leaking into X).
- IIS2MDC: 50 Hz continuous, high resolution, temperature compensation, offset cancellation, low-pass
  filter, block data update. The loop is paced on its data-ready flag.
- I2C at 400 kHz (fast mode, allowed by both chips, needed for a full 50 Hz) with a 3 ms timeout, so a loose wire resets the bus instead of hanging.
- If a chip does not start, the sketch prints `INIT FAILED  BMI270=x  IIS2MDC=y (try n, retrying)` and keeps retrying instead of stopping. If no sample arrives for 1 s while running, it prints `# no data for 1 s, re-initialising` and restarts both chips.

If it prints `INIT FAILED`, check the wiring with `../../i2c_probe/`. If it goes silent after rough
handling, re-upload it or replug the USB.

Used by: `full_calibration.py`, `mahony_live.py`, `adaptive_compare.py`, `physical_test.py`,
`imu_calibrate.py`, and MotionCal.
