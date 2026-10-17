"""
Interactive IMU calibration for the BMI270 (accelerometer 6-position + gyro bias).

Needs the Uno running motioncal_stream (or mag_calibrated), which prints at ~50 Hz:
  Raw:ax,ay,az,gx,gy,gz,mx,my,mz   (accel 8192 counts/g, gyro 16 counts/(deg/s))

  python imu_calibrate.py --port COM5

You place the board still on each of its 6 faces and press Enter. For each pose the
script averages 3 s of data. Then it fits, per accel axis, an offset and a gain so
that |a| = 1 g in every pose, and takes the gyro bias as the mean gyro reading
over all still poses. Results go to imu_cal.json.
"""
import argparse
import json
import math
import sys
import time

import numpy as np
import serial

A_LSB = 8192.0   # counts per g in the MotionCal stream
G_LSB = 16.0     # counts per deg/s

# (label, instruction, expected axis index, expected sign)
POSES = [
    ("+Z up", "board flat, components facing UP", 2, +1),
    ("-Z up", "board flat, components facing DOWN", 2, -1),
    ("+X up", "stand the board on its edge so +X points up", 0, +1),
    ("-X up", "flip it, -X points up", 0, -1),
    ("+Y up", "stand it on the other edge so +Y points up", 1, +1),
    ("-Y up", "flip it, -Y points up", 1, -1),
]


def parse(line):
    if not line.startswith("Raw:"):
        return None
    try:
        v = [int(x) for x in line[4:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 9 else None


def read_block(ser, seconds):
    ser.reset_input_buffer()
    rows, t0 = [], time.time()
    while time.time() - t0 < seconds:
        raw = ser.readline().decode("ascii", errors="ignore").strip()
        v = parse(raw)
        if v:
            rows.append(v)
    return np.array(rows, dtype=float)


def fit_accel(P):
    """Offset + gain per axis so that |(m - o) * k| = 1 for every pose (Gauss-Newton)."""
    o = np.zeros(3); k = np.ones(3)
    for i in range(3):
        mx, mn = P[:, i].max(), P[:, i].min()
        o[i] = (mx + mn) / 2; k[i] = 2 / (mx - mn)
    for _ in range(20):
        c = (P - o) * k
        n = np.linalg.norm(c, axis=1)
        r = n - 1
        J = np.hstack([-(c * k) / n[:, None], (c * (P - o)) / n[:, None]])
        dx, *_ = np.linalg.lstsq(J, -r, rcond=None)
        o += dx[:3]; k += dx[3:]
    c = (P - o) * k
    return o, k, np.abs(np.linalg.norm(c, axis=1) - 1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--out", default="imu_cal.json")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    print("Opening port, the Uno restarts, waiting 3 s ...")
    time.sleep(3)
    test = read_block(ser, 1.0)
    if len(test) == 0:
        print("No 'Raw:' lines on the port. Upload motioncal_stream and try again.")
        input("Press Enter to close."); return
    print(f"Receiving {len(test)} samples per second. Good.\n")

    acc_means, gyro_all = [], []
    i = 0
    while i < len(POSES):
        name, how, want_axis, want_sign = POSES[i]
        input(f"[{i + 1}/6] {name}: {how}. Hold still and press Enter ...")
        print("   measuring, do not touch ...")
        d = read_block(ser, args.seconds)
        if len(d) < 20:
            print("   too few samples, try again"); continue
        a = d[:, 0:3] / A_LSB
        g = d[:, 3:6] / G_LSB
        sd = a.std(axis=0).max() * 1000
        if sd > 8:
            print(f"   board moved (accel sd {sd:.1f} mg), try this pose again"); continue
        m = a.mean(axis=0)
        k = int(np.argmax(np.abs(m)))
        print(f"   accel mean  x {m[0]:+.4f}  y {m[1]:+.4f}  z {m[2]:+.4f} g"
              f"   (strongest axis {'+' if m[k] > 0 else '-'}{'XYZ'[k]}),  noise {sd:.2f} mg")
        if k != want_axis or np.sign(m[k]) != want_sign or abs(m[k]) < 0.8:
            print(f"   WRONG POSE: expected {name}, the board reads {'+' if m[k] > 0 else '-'}{'XYZ'[k]} up. "
                  f"Turn it and press Enter again.")
            continue
        acc_means.append(m); gyro_all.append(g)
        i += 1

    P = np.array(acc_means)
    off, gain, err = fit_accel(P)
    if np.any(gain < 0.9) or np.any(gain > 1.1) or np.any(np.abs(off) > 0.2):
        print("Fit looks wrong (gain far from 1 or offset > 200 mg). Not saving. Run again.")
        input("Press Enter to close."); return
    G = np.vstack(gyro_all)
    gbias, gsd = G.mean(axis=0), G.std(axis=0)

    print("\n================ RESULT ================")
    print("Accelerometer  (a_cal = (a_raw - offset) * gain, a in g)")
    for j, ax in enumerate("XYZ"):
        print(f"  {ax}: offset {off[j] * 1000:+7.2f} mg   gain {gain[j]:.5f}   (scale error {(1 / gain[j] - 1) * 100:+.3f} %)")
    print(f"  |a| error before cal, worst pose: {np.abs(np.linalg.norm(P, axis=1) - 1).max() * 1000:.2f} mg"
          f"   (after: {err.max() * 1000:.2f} mg, exact with 6 poses)")
    print("Gyroscope  (w_cal = w_raw - bias, deg/s)")
    for j, ax in enumerate("XYZ"):
        print(f"  {ax}: bias {gbias[j]:+.4f} deg/s   noise sd {gsd[j]:.3f}")
    print("Datasheet: accel offset +-20 mg, sensitivity +-0.4 %, gyro offset +-0.5 deg/s")

    cal = {"accel_offset_g": off.round(6).tolist(), "accel_gain": gain.round(6).tolist(),
           "gyro_bias_dps": gbias.round(5).tolist(),
           "pose_means_g": P.round(5).tolist(),
           "units": "stream counts: accel 8192/g, gyro 16/(deg/s)",
           "usage": "a = (raw/8192 - offset) * gain ; w = raw/16 - bias"}
    with open(args.out, "w") as f:
        json.dump(cal, f, indent=2)
    print(f"\nSaved {args.out}")

    print("\nCheck: turn the board to any angle and hold still. |a| should read 1.000 g.")
    print("Ctrl+C to stop.\n")
    try:
        while True:
            d = read_block(ser, 0.5)
            if len(d) == 0: continue
            a = ((d[:, 0:3] / A_LSB - off) * gain).mean(axis=0)
            w = (d[:, 3:6] / G_LSB - gbias).mean(axis=0)
            sys.stdout.write(f"\r|a| = {np.linalg.norm(a):.4f} g    gyro {w[0]:+.3f} {w[1]:+.3f} {w[2]:+.3f} deg/s   ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    ser.close()


if __name__ == "__main__":
    main()
