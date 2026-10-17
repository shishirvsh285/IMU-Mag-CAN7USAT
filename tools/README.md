# tools

`MotionCal.exe`: PJRC Motion Sensor Calibration Tool for Windows, by Paul Stoffregen.
Source and other builds: https://github.com/PaulStoffregen/MotionCal

How we use it:

1. Upload `../motioncal_stream/` to the Uno.
2. Open MotionCal, pick the Uno's port (COM5 here).
3. Tumble the board slowly in every direction until the point cloud forms a full sphere.
4. Read off the hard iron offset and soft iron matrix (we saved ours in `../mag_cal_motioncal.json`).

`full_calibration.py` does the same fit in Python and also finds the magnetometer-to-IMU axis map,
which MotionCal does not.
