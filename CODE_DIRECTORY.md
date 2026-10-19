# Code Directory

What every file in these folders is, and how they fit together.
Last updated: 16 Oct 2026 (23:00).

- `Downloads\bmi270_init\` : BMI270 + IIS2MDC sensor board (IMU + magnetometer)
- `Downloads\gps_test\` : NEO-6M GPS module (section at the end)

---

## 1. Hardware

**Sensor board -> Arduino Uno**

| Board pin | Uno pin |
|---|---|
| VDD | 3.3V (not 5V) |
| SDA | A4 |
| SCL | A5 |
| BMI270_SDO | GND (BMI270 at 0x68) |
| GND | GND |
| INT pins | not connected |

I2C addresses: BMI270 = 0x68, IIS2MDC = 0x1E. Serial port on this laptop: COM5.

---

## 2. Arduino sketches (`bmi270_init`)

Only one sketch can be on the Uno at a time. Each sketch folder must keep its own copy of `bmi270_config.h`.

| Sketch | What it does | Serial output | Used by |
|---|---|---|---|
| `bmi270_init.ino` + `bmi270_config.h` (folder root) | BMI270 init (8 kB Bosch config upload), accel 100 Hz +-8 g, gyro 200 Hz +-2000 deg/s | `acc g: x y z   gyr dps: x y z` at ~5 Hz, 115200 baud | `log_serial.py`, `plot_errors.py`, `integrate_angles.py` |
| `iis2mdc_read\iis2mdc_read.ino` | IIS2MDC only: 50 Hz, high-res, temp comp, offset cancel, LPF, BDU | `mag mG: x y z` at 50 Hz | `log_mag.py`, `plot_mag.py`, `mag_heading.py`, `live_*.py` |
| `motioncal_stream\motioncal_stream.ino` | Both chips, raw, MotionCal format, I2C timeout | `Raw:ax,ay,az,gx,gy,gz,mx,my,mz` (8192/g, 16/(deg/s), 1 mG), CR LF, ~50 Hz | MotionCal, `imu_calibrate.py`, `mahony_live.py` |
| `adaptive_mahony_uno\adaptive_mahony_uno.ino` | Adaptive Mahony running ON the Uno (needs `AdaptiveMahony.h`, `imu_cal.h`, `bmi270_config.h` in the folder). Copy the `imu_cal.h` made by `full_calibration.py` over the placeholder | `roll,pitch,yaw,kp,wa,us` at 50 Hz. Measured: 2.8 ms per update, 25 kB flash, 629 B RAM | Flight code base |
| `mag_calibrated\mag_calibrated.ino` | Same as above, but mag corrected on the Uno with the MotionCal result: `cal = invW * (raw - V)` | `mag mG: x y z` (calibrated) + `Raw:` line | MotionCal check, `mahony_live.py`, mag `live_*.py` |

Config file `bmi270_config.h`: Bosch BMI270_SensorAPI (BSD-3), unchanged.

**Currently on the Uno:** `motioncal_stream` (as of 15 Oct 2026).

Upload from the Arduino IDE, or with
`arduino-cli upload -b arduino:avr:uno -p COM5 <sketch_folder>`.
Close any Serial Monitor or Python window first, only one program can hold COM5.

---

## 3. Python scripts (`bmi270_init`)

All need `imu_common.py` in the same folder. Install once: `pip install pyserial matplotlib numpy`.

### Shared
| File | What it is |
|---|---|
| `imu_common.py` | Line parsers (`acc g:`, `mag mG:`), plot style, heading function `atan2(-Y, X)` |
| `mahony.py` | Mahony filter class (quaternion, accel + gyro, optional mag) |

### BMI270 (needs `bmi270_init.ino`)
| File | Run | Output |
|---|---|---|
| `log_serial.py` | `python log_serial.py --port COM5` | Live window (\|a\| - 1 g, gyro), CSV |
| `plot_errors.py` | `python plot_errors.py run2.csv [--skip s --end s --tag name]` | `_errors.png`, `_histograms.png`, stats table |
| `integrate_angles.py` | `python integrate_angles.py run2.csv [--bias-window 10]` | `_angles.png`: gyro integrated angle, raw and bias-removed |

### Magnetometer (needs `iis2mdc_read.ino`, or `mag_calibrated.ino` for calibrated data)
| File | Run | Output |
|---|---|---|
| `log_mag.py` | `python log_mag.py --port COM5` | Live window (noise, \|B\|, heading), CSV |
| `plot_mag.py` | `python plot_mag.py mag1.csv` | `_mag_errors.png`, `_mag_histograms.png` |
| `mag_heading.py` | `python mag_heading.py mag1.csv [--make-cal] [--declination d]` | Heading CSV and plot, disturbance flag, 2D calibration `mag_cal.json` |
| `live_heading.py` | `python live_heading.py --port COM5` | Live: field angle in XY, YZ, ZX planes |
| `live_projection.py` | `python live_projection.py --port COM5` | Live: field vector projected on the 3 planes |
| `live_ellipsoid.py` | `python live_ellipsoid.py --port COM5` | Live 3D raw cloud with fitted ellipsoid |
| `mag_ellipsoid.py` | `python mag_ellipsoid.py proj1.csv` | Full ellipsoid fit (hard + soft iron), `mag_cal3d.json`, `_ellipsoid.png` |
| `apply_motioncal.py` | `python apply_motioncal.py ell2.csv` | Applies `mag_cal_motioncal.json` to a log, `_mcal.csv/png` |

### IMU calibration and attitude (needs `motioncal_stream.ino` or `mag_calibrated.ino`)
| File | Run | Output |
|---|---|---|
| `full_calibration.py` | `python full_calibration.py --port COM5` | **All-in-one**: gyro bias (10 s still), accel 6 poses, mag tumble (ellipsoid), mag-to-IMU axis map. Saves `imu_full_cal.json`, `imu_full_cal_report.txt`, `imu_cal.h`. Needs `motioncal_stream` (raw mag). |
| `imu_calibrate.py` | `python imu_calibrate.py --port COM5` | Interactive 6-pose accel cal + gyro bias only, `imu_cal.json`. Rejects wrong poses. |
| `mahony_live.py` | `python mahony_live.py --port COM5 [--kp 1 --ki 0.05] [--mag --mag-map x,-y,z]` | Live 3D board + roll/pitch/yaw + probability density of each angle over the last 10 s (`--pdf-window`), `mahony_<time>.csv` |
| `physical_test.py` | `python physical_test.py --port COM5 [--only shake] [--set kp_min=0.05]` / `--replay phystest_x.csv` | **Guided hands-on test** of standard vs adaptive: shake (no rotation), pendulum, spin + swing, tumble, magnet. Prompts and countdowns in the terminal. Truth = still board at the end + gyro run backwards. Saves `phystest_<time>.csv`, `_results.txt`, `.png`. `--replay` re-scores a recording (other settings, no board) |
| `physical_test_fake.py` | used by `physical_test.py --selftest` | Fake board that "does" every test, only for checking the script |
| `adaptive_compare.py` | `python adaptive_compare.py --port COM5` (live) / `--replay adapt_x.csv [--set kp_min=0.05]` | Standard vs adaptive side by side, logs raw to `adapt_<time>.csv`. Replay scores both on shake / pendulum tests (still board = truth) |

### Adaptive Mahony (descent attitude filter)
| File | Run | Output |
|---|---|---|
| `adaptive_mahony.py` | imported | `AdaptiveMahony` class (Kalman-like accel gain, gyro-carried averaged gravity, mag heading-only, bias freeze) + `GpsTurnAccel` (turn accel from GPS velocity) |
| `AdaptiveMahony\AdaptiveMahony.h` | `#include` | Same filter in float C++ for Arduino. Matches the Python within 0.35 deg |
| `descent_sim.py` | imported / `python descent_sim.py` | Simulated descents with true attitude: pendulum under canopy + spin + gusts, parafoil turns, deployment tumble, mag disturbance |
| `test_adaptive.py` | `python test_adaptive.py [--seeds 5]` | Standard vs adaptive vs adaptive+GPS error table, `adaptive_test_<scenario>.png` |
| `tune_sweep.py` | `python tune_sweep.py 3` | Grid search of settings on sims (tuning seeds 100+, test seeds 0-4 kept separate) |
| `cpp_check\run_filter.cpp`, `compare_cpp.py` | `g++ run_filter.cpp`, `python compare_cpp.py` | Checks the C++ header gives the same attitude as the Python |

---

## 4. Calibration files

| File | What | Status |
|---|---|---|
| `mag_cal_motioncal.json` | MotionCal result: offset V (uT), invW matrix, field 30.66 uT | In use on the Uno (`mag_calibrated`). Not yet verified in a clean session. |
| `mag_cal3d.json` | Our ellipsoid fit from `proj1.csv` | Reference only. Runs disagree with each other. |
| `imu_cal_BAD_all_poses_flat.json` | Failed accel cal (all 6 poses recorded flat) | Do not use. `imu_cal.json` not made yet. |

---

## 5. Data logs and plots

| Files | Sketch | What |
|---|---|---|
| `run1.csv` + `run1_*.png` | bmi270_init | 60 s, board bumped at 28 to 32 s and 50 s. Still part: 1 to 27 s (`run1_still_*`) |
| `run2.csv` + `run2_*.png` | bmi270_init | 37 s still. Main accel/gyro error data |
| `mag1.csv` + `mag1_*` | iis2mdc_read | 40 s still. Main mag noise data |
| `mag2.csv` + `mag2_heading.*` | iis2mdc_read | 152 s, magnet nearby for long stretches |
| `head1.csv` | iis2mdc_read | From `live_heading.py` |
| `proj1.csv` + `proj1_*` | iis2mdc_read | 33k samples tumble, used for `mag_ellipsoid.py` |
| `ell1.csv`, `ell2.csv` + `_mcal.*` | iis2mdc_read | From `live_ellipsoid.py`, plus MotionCal cal applied |
| `mahony_20261015_224930.csv` | motioncal_stream | First Mahony run |

---

## 6. Reports and tools

| File | What |
|---|---|
| `IMU_datasheet_vs_measured.xlsx` | Datasheet error values, register settings, measured vs spec |
| `IMU_work_update.docx` | Full work update with graphs |
| `IMU_measured_vs_datasheet.docx` | Data-only comparison with datasheet |
| `tools\MotionCal.exe` | PJRC Motion Sensor Calibration Tool (signed) |

Not in this folder (sent in chat only): `id_check.ino` (chip ID check), `bmi270_acc_cal.ino` (on-Uno 6-pose accel cal).

---

## 7. GPS (`Downloads\gps_test\`)

**Wiring:** GPS VCC -> 5V, GND -> GND, GPS TX -> D3, GPS RX -> D4 (as used in `gps_sd_logger`). Keep D0/D1 free.

| Sketch | What | Serial |
|---|---|---|
| `gps_raw_nmea\` | Raw NMEA passthrough (reads D4), warns if silent | 115200 |
| `gps_probe\` | Finds the GPS on D3 or D4, prints fix + satellites locked/in view. `SHOW_RAW = true` also prints NMEA | 115200 |
| `gps_device_example\` | TinyGPSPlus DeviceExample (lat, lon, date, time), reads D4 | 115200 |
| `gps_full_example\` | TinyGPSPlus FullExample table, reads D4 | 115200 |
| `gps_sd_logger\` | GPS + DS1307 RTC + SD card log every 10 s to `gpsLOG.txt`. TinyGPS and RTClib files inside the sketch folder | 9600 |

Library files in the folder root (`TinyGPS.*`, `RTClib.*`, `RTC_*.cpp`) are the originals. `gps_sd_logger` uses its own copies.
Installed libraries: TinyGPSPlus 1.0.3, Adafruit BusIO 1.17.4, SD 1.3.0.

---

## Change log
- 16 Oct 2026: file created.
- 16 Oct 2026: added `full_calibration.py` (all calibration steps in one script).
- 17 Oct 2026: added the adaptive Mahony set (`adaptive_mahony.py`, `AdaptiveMahony.h`, `adaptive_mahony_uno`, `descent_sim.py`, `test_adaptive.py`, `tune_sweep.py`, `adaptive_compare.py`, `cpp_check`). `full_calibration.py` now also writes `USE_MAG`, `MAG_DIP_DEG`, `MAG_FIELD_MG` into `imu_cal.h`. Uno left running `motioncal_stream`.
- 17 Oct 2026: added `physical_test.py` (guided physical test with scoring) and `physical_test_fake.py`.
- 17 Oct 2026: `mahony_live.py` now shows a live probability density (histogram + kernel density, mean, sd) of roll, pitch, yaw over the last 10 s.
- 17 Oct 2026: folder is now a git repo, pushed to https://github.com/shishirvsh285/IMU-Mag-CAN7USAT (branch main). `__pycache__` is ignored. `Downloads\gps_test` is not in it.
- 17 Oct 2026: added README.md files (root + each sketch folder, AdaptiveMahony, cpp_check, tools) and requirements.txt for the git repo.
- 19 Oct 2026: added `ground/` (ground station only): `motioncal_stream`, `full_calibration.py`, `heading.py` (calibration + Mahony -> roll, pitch, compass heading, live or `--replay`), copies of `mahony.py` and `adaptive_mahony.py`, README. Copies: change both if a filter changes.
- 19 Oct 2026: datasheet check. In `motioncal_stream` (and ground copy), `mag_calibrated`, `adaptive_mahony_uno`: gyro X cross-axis correction from GYR_CAS.factor_zx (BMI270 4.6.10, ours = 2, 0.39 % of Z), accel and gyro ODR 50 Hz (0xA7 / 0xE7) to match the 50 Hz read rate, gyro scale 16.384 LSB/dps, accel in the Raw: stream no longer clamped at 4 g. Measured still noise after: accel 0.59 mg, gyro 0.035 deg/s (was 0.06-0.12). Old calibrations made before this are invalid: re-run full_calibration.py. `bmi270_init.ino` (5 Hz demo) left as it was.
