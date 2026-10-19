# IMU-Mag-CAN7USAT

Attitude estimation for the **CAN-7USAT CanSat** (Team Samarthya, IN-SPACe CanSat competition).

We read a **BMI270** (accelerometer + gyroscope) and an **IIS2MDC** (magnetometer) on an Arduino Uno,
calibrate them, and work out roll, pitch and yaw with a Mahony filter. The main piece of work is an
**adaptive Mahony filter** that keeps working while the CanSat swings and spins under its parachute.
A normal filter gets fooled there, because the accelerometer stops pointing at gravity.

Everything is here: the Arduino sketches, the Python tools for logging, calibration and live viewing,
a descent simulator with ground truth, and the filter in both Python and Arduino C++.

---

## Contents

- [Hardware](#hardware)
- [Quick start](#quick-start)
- [How the pieces fit together](#how-the-pieces-fit-together)
- [Repository layout](#repository-layout)
- [The adaptive Mahony filter (short version)](#the-adaptive-mahony-filter-short-version)
- [Results so far](#results-so-far)
- [Known issues](#known-issues)
- [Credits](#credits)

---

## Hardware

| Part | Role | I2C address |
|---|---|---|
| Bosch **BMI270** | 3-axis accel (±8 g) + 3-axis gyro (±2000 °/s) | `0x68` (SDO tied to GND) |
| ST **IIS2MDC** | 3-axis magnetometer, 1.5 mG/LSB | `0x1E` |
| Arduino **Uno** | Reads both chips, streams data over USB at 115200 baud | |

**Wiring (sensor board to Uno)**

| Sensor board | Uno |
|---|---|
| VDD | **3.3V** (not 5V) |
| GND | GND |
| SDA | A4 |
| SCL | A5 |
| BMI270 SDO | GND |
| INT pins | not used |

The bus runs at 100 kHz with a 3 ms I2C timeout, so a loose jumper wire resets the bus instead of
freezing the Uno forever.

---

## Quick start

### 1. Install

- Arduino IDE 2 (or `arduino-cli`) with the **Arduino AVR Boards** core.
- Python 3.10 or newer, then:

```bash
pip install -r requirements.txt
```

### 2. Check the wiring

Upload `i2c_probe/` and open the Serial Monitor at 115200 baud. You should see:

```
found: 0x1E 0x68   BMI270 id=0x24 (want 0x24)   IIS2MDC id=0x40 (want 0x40)
```

### 3. Stream both sensors

Upload `motioncal_stream/`. It prints about 50 lines per second:

```
Raw:ax,ay,az,gx,gy,gz,mx,my,mz
```

Units: accel 8192 counts/g, gyro 16 counts/(°/s), mag 1 count = 1 mG. These are the units the
PJRC MotionCal tool expects, so the same stream feeds MotionCal and all our Python tools.

### 4. Calibrate (about 3 minutes)

```bash
python full_calibration.py --port COM5
```

The script talks you through three steps:

1. **Gyro bias:** board flat and still for 10 s.
2. **Accelerometer:** 6 still poses (each face up), 3 s each. Wrong or wobbly poses are rejected.
3. **Magnetometer:** tumble the board slowly through every direction for about 60 s, away from metal.

It writes `imu_full_cal.json`, a readable report, and `imu_cal.h` (the same numbers as C constants
for the Uno). It also finds the axis map between the magnetometer and the IMU, which is not obvious:
the IIS2MDC frame is left-handed.

### 5. Watch the attitude live

For calibrated heading on the ground station use `ground/heading.py` (see [`ground/README.md`](ground/README.md)).
The tools below are the development versions.

```bash
python mahony_live.py --port COM5          # 6-DOF: accel + gyro
python mahony_live.py --port COM5 --mag    # 9-DOF: also the magnetometer
```

You get a 3D board, roll/pitch/yaw against time, and the probability density of each angle over the
last 10 s. Keep the board still for the first 3 s while it measures the gyro bias.

### 6. Compare standard and adaptive Mahony on the real board

```bash
python physical_test.py --port COM5
```

A guided test: it tells you when to shake the board, swing it on a string, spin it and tumble it, then
scores both filters (see [How we get ground truth](#how-we-get-ground-truth-on-a-real-board)).

> On Linux or macOS the port looks like `/dev/ttyACM0` or `/dev/cu.usbmodem*` instead of `COM5`.
> Only one program can hold the port at a time, so close the Serial Monitor first.

---

## How the pieces fit together

```
 Uno sketch                          Python on the laptop
 ───────────                         ─────────────────────
 i2c_probe            ──────────►    (Serial Monitor: wiring check)

 bmi270_init.ino      ──────────►    log_serial.py → plot_errors.py, integrate_angles.py
 iis2mdc_read         ──────────►    log_mag.py → plot_mag.py, mag_heading.py,
                                     live_heading.py, live_projection.py, live_ellipsoid.py,
                                     mag_ellipsoid.py

 motioncal_stream     ──────────►    full_calibration.py  → imu_full_cal.json, imu_cal.h
   (Raw: lines)                      mahony_live.py, adaptive_compare.py, physical_test.py
                                     tools/MotionCal.exe

 adaptive_mahony_uno  ◄──────────    imu_cal.h  (filter runs on the Uno itself)
```

Typical order: probe the wiring, look at raw noise, calibrate, then run a filter (live on the laptop
or on the Uno).

---

## Repository layout

### Ground station: [`ground/`](ground)

A self-contained folder with only what the ground station needs: the stream sketch, `full_calibration.py`,
and `heading.py`, which applies the calibration and runs the Mahony filter to give roll, pitch and compass
heading (live from USB, or offline from a saved log). Start there if you only want to use the board.

### Arduino sketches

| Folder / file | What it does |
|---|---|
| [`i2c_probe/`](i2c_probe) | Scans the bus and prints both chip IDs once a second. First thing to run when something goes quiet. |
| `bmi270_init.ino` + `bmi270_config.h` (root) | BMI270 only. Uploads Bosch's 8 kB config blob, then prints accel (g) and gyro (°/s) at about 5 Hz. |
| [`iis2mdc_read/`](iis2mdc_read) | IIS2MDC only, 50 Hz, temperature compensation, offset cancellation, low-pass filter. Prints `mag mG: x y z`. |
| [`motioncal_stream/`](motioncal_stream) | **Main streaming sketch.** Both chips, raw, in MotionCal's `Raw:` format. |
| [`mag_calibrated/`](mag_calibrated) | Same stream, but the magnetometer is corrected on the Uno with a MotionCal result. |
| [`adaptive_mahony_uno/`](adaptive_mahony_uno) | The adaptive filter running on the Uno. Prints roll, pitch, yaw at 50 Hz. |

Each sketch folder keeps its own copy of `bmi270_config.h`, because the Arduino IDE only compiles
files inside the sketch folder.

### Python: sensor checks

| File | What it does |
|---|---|
| `imu_common.py` | Shared line parsers and plot style. Imported by most scripts. |
| `log_serial.py` | Live window + CSV log for `bmi270_init.ino`. |
| `plot_errors.py` | Noise, bias and drift plots and histograms from a still log. |
| `integrate_angles.py` | Integrates the gyro to angles, raw and bias-removed, to show drift. |
| `log_mag.py`, `plot_mag.py` | Same pair for the magnetometer. |
| `mag_heading.py` | Heading from the magnetometer, with a simple 2D calibration. |
| `live_heading.py`, `live_projection.py`, `live_ellipsoid.py` | Live views of the magnetic field vector: angles in each plane, projections, and the raw 3D cloud with a fitted ellipsoid. |
| `mag_ellipsoid.py` | Full ellipsoid fit (hard + soft iron) from a tumble log. |
| `apply_motioncal.py` | Applies a MotionCal calibration to an old log. |

### Python: calibration

| File | What it does |
|---|---|
| `full_calibration.py` | **All calibration in one go** (gyro, accel, mag, axis map). Outputs `imu_full_cal.json` and `imu_cal.h`. |
| `imu_calibrate.py` | Older: accel 6 poses + gyro bias only. |

### Python: attitude filters and testing

| File | What it does |
|---|---|
| `mahony.py` | Standard Mahony filter (quaternion, accel + gyro, optional mag). |
| `mahony_live.py` | Live 3D view of the standard filter, with angle density plots. |
| `adaptive_mahony.py` | **Adaptive Mahony filter** + `GpsTurnAccel` helper. |
| `descent_sim.py` | Simulated CanSat descents with true attitude: parachute swing and spin, parafoil turns, deployment tumble, magnetic disturbances. |
| `test_adaptive.py` | Runs standard vs adaptive on the simulator and prints an error table. |
| `tune_sweep.py` | Grid search of filter settings on the simulator. |
| `adaptive_compare.py` | Both filters side by side on the real board, live or replayed from a log. |
| `physical_test.py` | Guided hands-on test with scoring. `physical_test_fake.py` is a fake board, used only to self-test the script. |
| [`cpp_check/`](cpp_check) | Checks that the Arduino C++ filter gives the same answer as the Python one. |

### Other

| Path | What it is |
|---|---|
| [`AdaptiveMahony/`](AdaptiveMahony) | The filter as a single Arduino/C++ header, with a full explanation in its README. |
| `mag_cal_motioncal.json` | Magnetometer calibration from MotionCal (hard iron V, soft iron matrix). |
| `mag_cal3d.json`, `imu_cal_BAD_all_poses_flat.json` | Old calibration outputs, kept for reference. The `BAD` one is a failed run: do not use it. |
| [`tools/`](tools) | PJRC MotionCal for Windows. |
| `CODE_DIRECTORY.md` | Our running lab notebook: every file, every change, with dates. |

Logs (`*.csv`), plots (`*.png`) and reports are kept out of git on purpose. Running the scripts makes
fresh ones.

---

## The adaptive Mahony filter (short version)

A Mahony filter integrates the gyro and nudges the result towards the accelerometer, which it
assumes points straight down:

$$\boldsymbol\omega_{\text{used}} = \boldsymbol\omega_{\text{gyro}} + K_p\,(\hat{\mathbf a}\times\hat{\mathbf v}) + K_i\!\int(\hat{\mathbf a}\times\hat{\mathbf v})\,dt$$

Under a parachute that assumption breaks. The payload hangs on its lines like a pendulum, so the
accelerometer reads "straight up the lines" at almost exactly 1 g, even when the payload is tilted
30°. Checking `|a| ≈ 1 g` does not catch this.

The adaptive version changes how much it trusts each sensor, every sample:

1. **Kalman-style gain.** It tracks how unsure it is about its own tilt ($P$) and how unsure it is about
   the accelerometer right now ($R$), and sets $K_p = P / (R\,\tau_c)$. It is high after power-on or a
   tumble, and very low while swinging.
2. **A better trust test.** It keeps a second gravity estimate: the accelerometer averaged over 3 s
   and carried along by the gyro. Real acceleration averages away (speed cannot grow forever) and
   gravity does not. If the accelerometer disagrees with this average, something is pushing the board.
3. **Gyro drift is still removed** through that average, even while the instant reading is ignored.
4. **The magnetometer only corrects heading**, so a magnetic disturbance can never tilt the estimate.
5. **The bias integral is frozen** while the data looks bad, so shaking cannot teach it a fake bias.
6. **Gyro saturation** (deployment shock) resets the filter to "attitude unknown".
7. **Optional GPS turn correction** for parafoil spirals.

The full explanation, settings and API are in [`AdaptiveMahony/README.md`](AdaptiveMahony/README.md).

---

## Results so far

**Measured on our board (still, after warm-up)**

| Quantity | Measured | Datasheet |
|---|---|---|
| Gyro bias | about +0.11, +0.06, −0.06 °/s | ±0.5 °/s |
| Gyro noise | 0.06 to 0.12 °/s rms | |
| Accel magnitude error | about 1 mg | offset ±20 mg |
| Magnetometer noise | 3.1 to 3.6 mG rms | 3 typ, 4.6 max mG |
| Local field / dip | about 306 mG, dip about 26° (Mumbai) | |

**Simulated descents (tilt error after deployment, rms, mean of 5 runs)**

| Scenario | Standard Mahony | Adaptive Mahony |
|---|---|---|
| Bench, slow hand tilts | 0.32° | 0.31° |
| Parachute: swing + 60 °/s spin + gusts | 9.4° | **4.5°** |
| Parafoil: steady turns | 12.9° | **3.6°** (2.2° with GPS) |

The settings were tuned on different simulator runs from the ones in this table.
On the Uno the filter takes **2.8 ms per update** (50 Hz leaves 20 ms), using 25 kB of flash and
629 bytes of RAM.

### How we get ground truth on a real board

There is no motion rig, so `physical_test.py` uses two tricks:

- **A still board is its own truth.** When it is still, the accelerometer reads only gravity. Every
  test ends with the board held still.
- **Run the gyro backwards.** From that known still attitude, the gyro is integrated backwards through
  the motion. Over a few seconds a calibrated gyro is good to about 0.1 °/s, and it never uses the
  accelerometer, so it is fair to both filters. The script prints how good this truth is and skips
  scoring when it gets worse than 5°.

---

## Known issues

- **`physical_test.py` time step.** The first real run measured the sample rate wrongly (serial data
  piled up while it waited for Enter), so both filters used 0.009 s instead of 0.02 s. Fix: clear the
  buffer after Enter and take the rate from timestamps. Until then, real-board scores are not valid.
- **Standard Mahony learns a fake bias** while the board is shaken, then yaw drifts (we saw
  0.28 °/s on a still board in 6-DOF). This is a weakness of the standard filter, and one reason the
  adaptive one exists.
- **`mahony_live.py --mag` uses the raw magnetometer** and an unverified axis map (`x,-y,z`). Heading is
  offset and wobbles when the board rotates. Run `full_calibration.py` first for real 9-DOF.
- **I2C hang.** After hard shaking, `motioncal_stream` once went silent. Both chips were fine, and
  re-uploading (or replugging USB) fixed it.
- **Gusts under a canopy** that push for a second or more fool any accelerometer-based filter,
  including ours. That is most of the remaining 4.5° in simulation.

---

## Credits

- `bmi270_config.h`: Bosch Sensortec [BMI270_SensorAPI](https://github.com/boschsensortec/BMI270_SensorAPI) (BSD-3-Clause), unchanged.
- `tools/MotionCal.exe`: [MotionCal](https://github.com/PaulStoffregen/MotionCal) by Paul Stoffregen / PJRC.
- Mahony filter: R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the Special Orthogonal Group", IEEE TAC, 2008.

Team Samarthya, IIT Bombay.
