# mag_calibrated

Same as `motioncal_stream`, except the magnetometer is corrected on the Uno with a MotionCal result:

$$\mathbf m_{\text{cal}} = W^{-1}\,(\mathbf m_{\text{raw}} - \mathbf V)$$

The constants in the sketch come from `../mag_cal_motioncal.json`: hard iron
$\mathbf V$ = (82.4, 37.1, 58.8) mG, soft iron $W^{-1}$, field 30.66 µT.

## Output (115200 baud)

```
mag mG: x y z          (calibrated)
Raw:ax,ay,az,gx,gy,gz,mx,my,mz
```

The `Raw:` line carries the calibrated magnetometer values, so MotionCal can be used to check that the
calibration holds (the point cloud should be a sphere centred on zero).

If the board is rebuilt or moved relative to the Uno and battery, the hard iron changes. Recalibrate
with `full_calibration.py` or MotionCal.
