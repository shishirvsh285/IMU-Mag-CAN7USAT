"""
mag_map_check.py : find how the IIS2MDC axes line up with the BMI270 axes, from the gyro.

Uno runs motioncal_stream (raw) or mag_cal_check. Uses mag_cal.json (from mag_calibrate.py) to correct
the magnetometer first, unless the Uno already does it.

  python mag_map_check.py --port COM5          rotate the board about all axes for 20 s
  python mag_map_check.py --port COM5 --time 30

Physics: a fixed field seen from a rotating board turns the opposite way to the board,
  dm/dt = -w x m      (all in the BMI270 frame)
Only the right axis map S (m_imu = S m_mag) makes the measured dm/dt match -w x m. All 48
signed permutations are tried. That test cannot tell S from -S, so the sign is fixed with gravity:
in the northern hemisphere the field points down, so m . up must be negative. The best one is saved into mag_cal.json as "imu_axis_map",
and mahony_live.py --mag uses it from then on.
"""
import argparse
import itertools
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
D2R = np.pi / 180


def parse(line):
    if not line.startswith("Raw:"):
        return None
    try:
        v = [int(x) for x in line[4:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 9 else None


def all_maps():
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            S = np.zeros((3, 3))
            for i in range(3):
                S[i, perm[i]] = signs[i]
            yield S


def map_name(S):
    out = []
    for i in range(3):
        j = int(np.argmax(np.abs(S[i])))
        out.append(("-" if S[i, j] < 0 else "") + "xyz"[j])
    return ",".join(out)


def score_maps(t, w, m, a):
    """w: gyro rad/s (N,3) in IMU frame, m: calibrated mag (N,3) in mag frame, a: accel (N,3) in IMU frame.
    dm/dt = -w x m cannot tell S from -S (both fit). The field points DOWN in the northern hemisphere
    (dip > 0), so the right one has m . up < 0, with up = accel direction."""
    # smooth a little, then central differences
    k = np.ones(5) / 5
    ms = np.column_stack([np.convolve(m[:, i], k, "same") for i in range(3)])
    ws = np.column_stack([np.convolve(w[:, i], k, "same") for i in range(3)])
    dm = np.gradient(ms, t, axis=0)
    sel = slice(5, -5)
    res = []
    for S in all_maps():
        mi = ms @ S.T
        pred = -np.cross(ws, mi)
        meas = dm @ S.T
        err = np.sum((meas[sel] - pred[sel]) ** 2)
        ref = np.sum(meas[sel] ** 2) + 1e-9
        res.append((err / ref, S))
    res.sort(key=lambda r: r[0])
    up = a / np.linalg.norm(a, axis=1)[:, None]
    out, seen = [], set()
    for e, S in res:
        key = tuple(np.abs(S).ravel())
        if (key, tuple(np.sign(S.sum(1)))) in seen:
            continue
        if np.mean(np.sum(up * (m @ S.T), 1)) > 0:     # field pointing up: wrong sign
            S = -S
        seen.add((key, tuple(np.sign(S.sum(1))))); seen.add((key, tuple(-np.sign(S.sum(1)))))
        out.append((e, S))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--time", type=float, default=20.0)
    ap.add_argument("--mag-cal", default=os.path.join(HERE, "mag_cal.json"))
    args = ap.parse_args()

    hard, soft = np.zeros(3), np.eye(3)
    have_cal = os.path.exists(args.mag_cal)
    if have_cal:
        c = json.load(open(args.mag_cal))
        hard, soft = np.array(c["hard_iron_mG"]), np.array(c["soft_iron_W"])

    import serial
    ser = serial.Serial(args.port, 115200, timeout=0.2)
    print("Port open, Uno restarting ...")
    time.sleep(0.3); ser.reset_input_buffer()
    uno_cal = False
    t0 = time.time()
    while time.time() - t0 < 2.2:
        line = ser.readline().decode("ascii", "ignore").strip()
        if line.startswith("# MAG CAL") and "NONE" not in line:
            uno_cal = True
    if uno_cal:
        hard, soft = np.zeros(3), np.eye(3)
    print("Magnetometer correction:", "on the Uno" if uno_cal else (args.mag_cal if have_cal else "none (raw)"))

    print("\nKeep the board STILL for 2 s (gyro bias) ...")
    ser.reset_input_buffer(); rows = []; t0 = time.time()
    while time.time() - t0 < 2.0:
        v = parse(ser.readline().decode("ascii", "ignore").strip())
        if v:
            rows.append(v)
    if len(rows) < 30:
        sys.exit("No 'Raw:' data. Upload motioncal_stream or mag_cal_check first.")
    gbias = np.array(rows, float)[:, 3:6].mean(0) / 16.0

    input(f"\nPress Enter, then ROTATE the board about all its axes for {args.time:.0f} s "
          "(yaw it round flat, roll it, pitch it, not too fast) ...")
    ser.reset_input_buffer(); T, rows = [], []; t0 = time.time()
    while time.time() - t0 < args.time:
        v = parse(ser.readline().decode("ascii", "ignore").strip())
        if v:
            T.append(time.time() - t0); rows.append(v)
            sys.stdout.write(f"\r  {time.time() - t0:4.1f} s  samples {len(rows)}  "); sys.stdout.flush()
    ser.close(); print()
    d = np.array(rows, float); t = np.array(T)
    # even time base: the Uno is paced at 50 Hz, host timestamps jitter
    t = np.linspace(t[0], t[-1], len(t))
    w = (d[:, 3:6] / 16.0 - gbias) * D2R
    m = (d[:, 6:9] - hard) @ soft.T
    rot = np.degrees(np.abs(w).sum(0) * np.median(np.diff(t)))
    print(f"rotation seen by the gyro about X, Y, Z: {rot.round(0)} deg")
    if (rot < 90).sum() >= 2:
        print("WARNING: rotate about more axes, the map is only partly determined")

    a = d[:, 0:3] / 8192.0
    res = score_maps(t, w, m, a)
    best, S = res[0]
    print("\nbest maps (lower = better fit; a clear winner is far below the rest)")
    for e, Sm in res[:5]:
        print(f"  {map_name(Sm):>10}   misfit {e:.3f}   det {np.linalg.det(Sm):+.0f}")
    clear = res[1][0] > 2 * best and best < 0.5
    print(f"\nRESULT: --mag-map {map_name(S)}   ({'CLEAR' if clear else 'NOT CLEAR: rotate more, about every axis'})")
    if np.linalg.det(S) > 0:
        print("note: det +1. The IIS2MDC frame is left-handed, so det -1 is expected. Check the result.")

    if have_cal and clear:
        c = json.load(open(args.mag_cal))
        c["imu_axis_map"] = map_name(S)
        c["imu_axis_map_matrix"] = S.tolist()
        json.dump(c, open(args.mag_cal, "w"), indent=2)
        print(f"Saved into {os.path.basename(args.mag_cal)}: mahony_live.py --mag will use it.")


if __name__ == "__main__":
    main()
