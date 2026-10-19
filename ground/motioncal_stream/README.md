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
| `ax ay az` | 8192 counts per g | BMI270 at ±8 g, rescaled from 4096 counts/g |
| `gx gy gz` | 16 counts per °/s | BMI270 at ±2000 °/s, rescaled from 16.4 |
| `mx my mz` | 1 count = 1 mG = 0.1 µT | IIS2MDC, 1.5 mG/LSB rescaled |

Each chip is in its **own** axes, unchanged. The IIS2MDC frame is left-handed, so its axes do not match
the BMI270. `full_calibration.py` finds the mapping.

Lines end in CR LF, because MotionCal only ends a line on CR.

## Sensor settings

- BMI270: config blob uploaded, accel 100 Hz, gyro 200 Hz, performance mode.
- IIS2MDC: 50 Hz continuous, high resolution, temperature compensation, offset cancellation, low-pass
  filter, block data update. The loop is paced on its data-ready flag.
- I2C at 100 kHz with a 3 ms timeout, so a loose wire resets the bus instead of hanging.

If it prints `INIT FAILED`, check the wiring with `../../i2c_probe/`. If it goes silent after rough
handling, re-upload it or replug the USB.

Used by: `full_calibration.py`, `mahony_live.py`, `adaptive_compare.py`, `physical_test.py`,
`imu_calibrate.py`, and MotionCal.
