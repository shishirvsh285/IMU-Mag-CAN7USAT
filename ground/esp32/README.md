# ESP32 versions

The same two sketches as for the Uno, ported to an **ESP32 DevKit V1** (30-pin, ESP32-WROOM). Everything on the
laptop side (`mag_calibrate.py`, `mag_map_check.py`, `mahony_live.py`, `heading.py`, `full_calibration.py`,
MotionCal) works unchanged. Only the COM port is different.

| Sketch | What it does |
|---|---|
| `motioncal_stream_esp32/` | Both sensors raw, `Raw:ax,ay,az,gx,gy,gz,mx,my,mz` at 50 Hz. Use it for calibrating. |
| `mag_cal_check_esp32/` | Same stream, magnetometer corrected on the ESP32 with `mag_cal.h`. Use it to check in MotionCal. |

## Wiring

| Sensor header pin | Signal | ESP32 DevKit V1 pin |
|---|---|---|
| 1 | I2C_SCK (clock) | **D22** |
| 2 | GND | **GND** |
| 3 | I2C_SDA (data) | **D21** |
| 4 | 3.3V supply | **3V3** (never VIN, that is 5 V) |

Both sides are 3.3 V, so no level shifting is needed. If the sensor board has no pull-up resistors and the
wires are long, 4.7 kOhm from SDA and from SCL to 3.3 V makes the bus solid at 400 kHz.

## Arduino IDE

Board: **ESP32 Dev Module** (package "esp32" by Espressif). Upload speed 921600 is fine. If the upload stalls
at "Connecting....", hold the **BOOT** button until it starts.

## Magnetometer calibration with the ESP32

1. Upload `motioncal_stream_esp32`. Close the Serial Monitor.
2. `python mag_calibrate.py --port COMx` (from the `ground` folder) and tumble the board for 60 s.
   It writes `mag_cal.h` into both `mag_cal_check/` and `esp32/mag_cal_check_esp32/`.
3. Upload `mag_cal_check_esp32` and open `tools/MotionCal.exe` on the same port: the sphere should sit on zero.
4. Optional: `python mag_map_check.py --port COMx` to measure the mag-to-IMU axis map, then
   `python mahony_live.py --port COMx --mag` (main folder) for the 3D box.

## Differences from the Uno versions

- `Wire.begin(21, 22)` names the pins. `Wire.setTimeOut(20)` (ms) replaces the Uno-only `setWireTimeout`.
- An ESP32 does not always reset when a program opens the port, so `mag_cal_check_esp32` repeats its
  `# MAG CAL ...` line every 2 s. The Python tools read that line to know the data is already corrected.
- On reset the ESP32 prints its boot messages at 74880 baud, which look like junk at 115200. The tools skip it.
