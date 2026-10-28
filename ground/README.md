# ground

Everything the ground station needs, and nothing else: **calibrate** the sensor board once, then turn
its raw data into **calibrated roll, pitch and compass heading**.

This folder is self-contained. Copy it to any laptop with Python and it works. (The files here are
copies of the ones in the repo root, so if you change a filter, change both.)

| File | What it does |
|---|---|
| `motioncal_stream/` | Uno sketch. Streams both sensors raw: `Raw:ax,ay,az,gx,gy,gz,mx,my,mz` at 50 Hz |
| `full_calibration.py` | Step 1. Measures every sensor error and writes `imu_full_cal.json` |
| `heading.py` | Step 2. Applies the calibration, runs the Mahony filter, gives roll, pitch, heading |
| `adaptive_mahony.py` | The adaptive Mahony filter (default) |
| `mahony.py` | The standard Mahony filter (`--filter standard`) |
| `mag_calibrate.py`, `mag_cal_check/` | Magnetometer-only calibration and its MotionCal check (see below) |
| `esp32/` | ESP32 DevKit V1 versions of `motioncal_stream` and `mag_cal_check` (wiring: SCL D22, SDA D21, 3V3, GND). See `esp32/README.md` |
| `requirements.txt` | `pip install -r requirements.txt` |

## 1. Set up

1. Wire the board: VDD to **3.3V**, GND, SDA to A4, SCL to A5, BMI270 SDO to GND.
2. Upload `motioncal_stream/` to the Uno (Arduino IDE, board "Arduino Uno").
3. `pip install -r requirements.txt`
4. Close the Serial Monitor. Only one program can use the port.

## 2. Calibrate (once, about 3 minutes)

```bash
python full_calibration.py --port COM5
```

Follow the prompts:

1. **Gyro:** board flat and still, 10 s.
2. **Accelerometer:** 6 still poses, each face pointing up in turn, 3 s each. Prop it against a book.
3. **Magnetometer:** turn the board slowly through every direction for 60 s. Keep phones, magnets,
   laptops and steel away. Watch the "direction coverage" number climb above 80 %.

You get `imu_full_cal.json` (used by `heading.py`), `imu_full_cal_report.txt` (each value checked against
the datasheet) and `imu_cal.h` (the same numbers for code on the Uno).

Check two lines in the report:

- `map is CLEAR`. If it says UNCERTAIN, redo the tumble with more variety.
- `|B| spread ... calibrated` under about 2 %.

Redo the calibration whenever the board, battery or anything metal near the sensor moves. Those change
the magnetometer offsets.

## 3. Get the heading

**Live**, from the board on USB:

```bash
python heading.py --port COM5
```

Keep the board still for the first 3 s. A window opens with a compass needle, the heading number, and
heading, roll and pitch over time. Everything is also logged to `heading_<time>.csv`.

**Offline**, from saved data (for example raw telemetry received during a flight):

```bash
python heading.py --replay flight_raw.txt --rate 50
```

The log can be a text file of `Raw:` lines or a CSV with `ax ay az gx gy gz mx my mz` columns. Output:
`flight_raw_heading.csv` and a plot.

Useful options:

| Option | What it does |
|---|---|
| `--no-plot` | Terminal only (also works for `--replay`) |
| `--filter standard` | Plain Mahony instead of adaptive |
| `--declination D` | Add the magnetic declination (deg, east positive) to get **true** north. Mumbai is close to 0. |
| `--cal file.json` | Use a different calibration file |

## Magnetometer only: calibrate and check in MotionCal

| File | What it does |
|---|---|
| `mag_calibrate.py` | Magnetometer-only calibration: 60 s tumble, ellipsoid fit, hard iron $\mathbf b$ and soft iron $W$ |
| `mag_cal_check/` | Uno sketch that applies that calibration on board and streams in MotionCal's format |

1. Close MotionCal and the Serial Monitor (only one program can hold the port), with `motioncal_stream` on the Uno.
2. `python mag_calibrate.py --port COM5` and turn the board through every direction until coverage passes 80 %.
   It prints the result, shows raw vs calibrated clouds, and writes `mag_cal.json`, `mag_cal_report.txt`
   and **`mag_cal_check/mag_cal.h`** (so the sketch picks it up with no copying).
3. Upload `mag_cal_check/` to the Uno. The first line after reset says which calibration is loaded.
4. Open `tools/MotionCal.exe`, pick the port, tumble again. A good calibration shows a sphere centred on
   zero, MotionCal's own "Magnetic Offset" near 0 and a small fit error.

$$\mathbf m_{\text{cal}} = W\,(\mathbf m_{\text{raw}} - \mathbf b)$$

The result stays in the IIS2MDC's own axes, which is what MotionCal expects. For heading, `full_calibration.py`
also finds the magnetometer-to-IMU axis map. `python mag_calibrate.py --refit mag_cal_<time>.csv` redoes the fit
from a saved tumble without the board.

## What the numbers mean

- **Heading:** the compass direction the board's **+X axis** points, 0 to 360°, clockwise from north
  (90° = east), like a normal compass. It is measured in the horizontal plane, so it stays right when the
  board is tilted.
- **Roll, pitch:** ZYX Euler angles about the board's X and Y axes, in degrees.
- **accel trust, mag trust:** how much the adaptive filter is believing each sensor right now, 0 to 1.
- **Warnings:** `MAG DISTURBED` when |B| is more than 15 % away from the calibrated field (a magnet,
  motor or steel nearby). `ACCELERATING` when |a| is more than 0.15 g away from 1 g.

## How it works

For every sample:

1. **Calibrate**
   - gyro: $\boldsymbol\omega = \text{raw}/16 - \mathbf b_g$ (°/s, then rad/s)
   - accel: $\mathbf a = (\text{raw}/8192 - \mathbf o) \odot \mathbf k$ (g)
   - mag: $\mathbf m = S\,W\,(\text{raw} - \mathbf b_m)$ (mG, turned into the BMI270 axes)

   $\mathbf b_m$ is the hard iron offset, $W$ the soft iron matrix and $S$ the axis map, all from
   `imu_full_cal.json`. The gyro bias $\mathbf b_g$ is re-measured during the first 3 s, because it moves
   with temperature.
2. **Filter.** The Mahony filter integrates the gyro and corrects it with the accelerometer (tilt) and
   the magnetometer (heading). The adaptive version lowers its trust in the accelerometer while the
   board is being accelerated, and lets the magnetometer correct only the heading. See
   `../AdaptiveMahony/README.md` for the full story.
3. **Heading.** World frame: X = magnetic north, Y = west, Z = up. The board's X axis in that frame is
   $(x_N, x_W)$, so

$$\text{heading} = \operatorname{atan2}(-x_W,\; x_N) + \text{declination}$$

## Tested

On simulated raw data with a known attitude (hard iron, soft iron and a left-handed axis map applied
on purpose, board tilted 20° and −10°, turned from 60° to 150°):

| Filter | Heading error after the first 10 s |
|---|---|
| adaptive | 0.6° rms, 0.8° max |
| standard | 1.8° rms, 3.2° max |

The filter needs about 5 s from power-on to swing the heading round to north. Not yet checked
against a real compass on the board.
