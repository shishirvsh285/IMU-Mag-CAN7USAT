# iis2mdc_read

Magnetometer only (IIS2MDC at 0x1E). Used to study its noise and calibration on its own.

Settings: 50 Hz continuous, high resolution, temperature compensation, offset cancellation, low-pass
filter, block data update. 1.5 mG per LSB.

## Output (115200 baud)

```
mag mG: x y z
```

Read by `log_mag.py`, `plot_mag.py`, `mag_heading.py`, `live_heading.py`, `live_projection.py`,
`live_ellipsoid.py` and `mag_ellipsoid.py`.
