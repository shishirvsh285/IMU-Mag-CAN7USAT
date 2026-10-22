"""
Live Mahony filter on the BMI270 (+ optional IIS2MDC) stream, with a 3D board view.

Uno must run motioncal_stream (or mag_calibrated). It sends ~50 Hz lines:
  Raw:ax,ay,az,gx,gy,gz,mx,my,mz   (accel 8192/g, gyro 16/(deg/s), mag 1 mG/count)

  python mahony_live.py --port COM5                 accel + gyro (roll, pitch good; yaw drifts slowly)
  python mahony_live.py --port COM5 --kp 2 --ki 0.1
  python mahony_live.py --port COM5 --mag           also use the magnetometer for yaw (see notes)

Calibration files are found automatically (in this folder or in ground/):
  imu_full_cal.json (full_calibration.py): accel offset/gain, mag hard + soft iron, AND the measured
                     mag -> IMU axis map. Used first if present.
  mag_cal.json      (mag_calibrate.py): mag hard + soft iron only, axis map from --mag-map.
  If the Uno runs mag_cal_check (it prints "# MAG CAL ..." at start), the magnetometer is already
  calibrated on the Uno, so it is NOT corrected a second time here.
  --mag-cal FILE / --full-cal FILE pick a file, --mag-cal none turns the correction off.

At start, keep the board still for 3 s: the gyro bias is measured then.
If imu_cal.json from imu_calibrate.py looks valid, its accel offsets/gains are applied.
Close the window to stop. Angles are saved to mahony_<time>.csv.
Right column: probability density of each angle over the last --pdf-window seconds (default 10):
histogram + smooth curve (Gaussian kernel density), with mean and standard deviation.

Notes on --mag: the IIS2MDC axes are not the same as the BMI270 axes (and the IIS2MDC frame
is left-handed). --mag-map says how to build mag X,Y,Z in BMI270 axes, e.g. "x,-y,z".
The default "x,-y,z" only fixes handedness. It is NOT verified for your board yet.
"""
import argparse
import collections
import csv
import json
import math
import os
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import serial

from imu_common import AXIS_COLORS, SURFACE, TEXT_2, style
from mahony import Mahony

A_LSB, G_LSB = 8192.0, 16.0
D2R = math.pi / 180


def parse(line):
    if not line.startswith("Raw:"):
        return None
    try:
        v = [int(x) for x in line[4:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 9 else None


def load_accel_cal(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        c = json.load(f)
    off, gain = np.array(c["accel_offset_g"]), np.array(c["accel_gain"])
    if np.all((gain > 0.9) & (gain < 1.1)) and np.all(np.abs(off) < 0.2):
        return off, gain
    print(f"{path} looks invalid (gain {gain}, offset {off}), ignoring it")
    return None


def find_file(arg, name):
    """explicit path, 'none', or look in this folder and in ground/"""
    if arg and arg.lower() == "none":
        return None
    if arg:
        return arg if os.path.exists(arg) else None
    here = os.path.dirname(os.path.abspath(__file__))
    for d in (os.getcwd(), here, os.path.join(here, "ground")):
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    return None


def mag_mapper(spec):
    parts = [p.strip() for p in spec.split(",")]
    idx = {"x": 0, "y": 1, "z": 2}
    m = [(idx[p[-1]], -1.0 if p.startswith("-") else 1.0) for p in parts]
    return lambda v: [s * v[i] for i, s in m]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--kp", type=float, default=1.0)
    ap.add_argument("--ki", type=float, default=0.05)
    ap.add_argument("--mag", action="store_true")
    ap.add_argument("--mag-map", default=None, help="mag axes in BMI270 axes, e.g. x,-y,z (default: from "
                    "imu_full_cal.json, else x,-y,z)")
    ap.add_argument("--mag-cal", default=None, help="mag_cal.json from mag_calibrate.py ('none' = off)")
    ap.add_argument("--full-cal", default=None, help="imu_full_cal.json from full_calibration.py ('none' = off)")
    ap.add_argument("--cal", default="imu_cal.json")
    ap.add_argument("--window", type=float, default=30.0)
    ap.add_argument("--pdf-window", type=float, default=10.0, help="seconds of data in the density plots")
    args = ap.parse_args()

    acal = load_accel_cal(args.cal)
    # ---- magnetometer calibration: full cal first, then mag-only cal ----
    full_p, mag_p = find_file(args.full_cal, "imu_full_cal.json"), find_file(args.mag_cal, "mag_cal.json")
    hard, soft, map_mat, mag_src = np.zeros(3), np.eye(3), None, "raw (no calibration file)"
    if full_p:
        c = json.load(open(full_p))
        hard, soft = np.array(c["mag_hard_iron_mG"]), np.array(c["mag_soft_iron"])
        map_mat = np.array(c["mag_axis_map_matrix"])
        mag_src = f"{os.path.basename(full_p)} (map {c.get('mag_axis_map', '?')})"
        off, gain = np.array(c["accel_offset_g"]), np.array(c["accel_gain"])
        acal = (off, gain)
    elif mag_p:
        c = json.load(open(mag_p))
        hard, soft = np.array(c["hard_iron_mG"]), np.array(c["soft_iron_W"])
        mag_src = f"{os.path.basename(mag_p)} ({c.get('created', '?')}, spread {c.get('B_spread_pct_cal', '?')} %)"
    if args.mag_map:
        map_mat = None
    mmap = mag_mapper(args.mag_map or "x,-y,z")

    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print("Port open, Uno restarting ...")
    time.sleep(0.3)
    ser.reset_input_buffer()                      # Windows can replay stale data when the port opens
    uno_cal = False
    t0 = time.time()
    while time.time() - t0 < 2.2:                 # read the sketch's start-up lines
        line = ser.readline().decode("ascii", errors="ignore").strip()
        if line.startswith("# MAG CAL") and "NONE" not in line:
            uno_cal = True
            print("Uno says:", line)
    if uno_cal:
        hard, soft = np.zeros(3), np.eye(3)
        mag_src = "calibrated ON THE UNO (mag_cal_check)" + ("" if map_mat is None else ", map from " + os.path.basename(full_p))
    ser.reset_input_buffer()
    if args.mag:
        print("Magnetometer:", mag_src)

    def mag_vec(v):
        m = soft @ (np.array(v, float) - hard)
        return list(map_mat @ m) if map_mat is not None else mmap(m)

    # ---- gyro bias + sample rate, board still ----
    print("Keep the board STILL for 3 s (measuring gyro bias) ...")
    rows, t0 = [], time.time()
    while time.time() - t0 < 3.0:
        v = parse(ser.readline().decode("ascii", errors="ignore").strip())
        if v:
            rows.append(v)
    if len(rows) < 30:
        print("No 'Raw:' data. Upload motioncal_stream first."); return
    R = np.array(rows, float)
    rate = len(R) / 3.0
    gbias = R[:, 3:6].mean(axis=0) / G_LSB
    print(f"rate {rate:.1f} Hz, gyro bias {gbias.round(3)} deg/s, accel cal {'ON' if acal else 'off'}")

    f = Mahony(args.kp, args.ki)
    # start from the accel tilt so it does not need to converge from level
    a0 = R[:, 0:3].mean(axis=0) / A_LSB
    roll0 = math.atan2(a0[1], a0[2]); pitch0 = math.atan2(-a0[0], math.hypot(a0[1], a0[2]))
    cr, sr, cp, sp = math.cos(roll0 / 2), math.sin(roll0 / 2), math.cos(pitch0 / 2), math.sin(pitch0 / 2)
    f.q = [cr * cp, sr * cp, cr * sp, -sr * sp]

    dt = 1.0 / rate
    out = time.strftime("mahony_%Y%m%d_%H%M%S.csv")
    fh = open(out, "w", newline=""); w = csv.writer(fh)
    w.writerow(["t_s", "roll_deg", "pitch_deg", "yaw_deg", "a_norm_g", "accel_used"])
    T = collections.deque(); E = [collections.deque() for _ in range(3)]
    st = {"buf": b"", "n": 0, "t": 0.0, "anorm": 1.0, "used": True}

    # ---- figure ----
    fig = plt.figure(figsize=(15, 7.5), facecolor=SURFACE)
    gs = fig.add_gridspec(3, 3, width_ratios=[1.1, 1.6, 0.75])
    ax3 = fig.add_subplot(gs[:, 0], projection="3d")
    ax3.set_facecolor(SURFACE)
    for p in (ax3.xaxis.pane, ax3.yaxis.pane, ax3.zaxis.pane):
        p.set_facecolor(SURFACE); p.set_edgecolor("#e4e3df")
    ax3.set_xlim(-1.2, 1.2); ax3.set_ylim(-1.2, 1.2); ax3.set_zlim(-1.2, 1.2); ax3.set_box_aspect((1, 1, 1))
    ax3.set_xlabel("world X", fontsize=8, color=TEXT_2); ax3.set_ylabel("world Y", fontsize=8, color=TEXT_2)
    ax3.set_zlabel("world Z (up)", fontsize=8, color=TEXT_2)
    title3 = ax3.set_title("", loc="left", fontsize=10)
    # board outline (long side along X) and body axes
    L, Wd = 1.0, 0.7
    corners = np.array([[L, Wd, 0], [-L, Wd, 0], [-L, -Wd, 0], [L, -Wd, 0], [L, Wd, 0]]) * 0.8
    board, = ax3.plot([], [], [], color="#0b0b0b", linewidth=2)
    axes_lines = [ax3.plot([], [], [], color=AXIS_COLORS[k], linewidth=3, label=f"board {k.upper()}")[0] for k in "xyz"]
    ax3.legend(loc="upper left", fontsize=8, frameon=False)

    names = ["Roll (about X)", "Pitch (about Y)", "Yaw (about Z)" + ("" if args.mag else ", gyro only: drifts slowly")]
    cols = [AXIS_COLORS["x"], AXIS_COLORS["y"], AXIS_COLORS["z"]]
    tax, lines, bigs = [], [], []
    for i in range(3):
        a = fig.add_subplot(gs[i, 1]); style(a, names[i], "deg")
        lines.append(a.plot([], [], color=cols[i], linewidth=1.6)[0])
        bigs.append(a.text(0.99, 0.9, "", transform=a.transAxes, ha="right", va="top", fontsize=18, fontweight="bold"))
        tax.append(a)
    tax[0].set_ylim(-180, 180); tax[1].set_ylim(-90, 90); tax[2].set_ylim(-180, 180)
    tax[2].set_xlabel("time (s)", fontsize=9, color=TEXT_2)
    # probability density panels (last pdf_window seconds)
    pax, phist, pkde, ptxt = [], [], [], []
    short = ["Roll", "Pitch", "Yaw"]
    for i in range(3):
        a = fig.add_subplot(gs[i, 2]); style(a, f"{short[i]} PDF, last {args.pdf_window:g} s", "density (1/deg)")
        phist.append(None)
        pkde.append(a.plot([], [], color=cols[i], linewidth=2)[0])
        ptxt.append(a.text(0.02, 0.95, "", transform=a.transAxes, ha="left", va="top", fontsize=8,
                           color=TEXT_2, family="monospace"))
        pax.append(a)
    pax[2].set_xlabel("angle (deg)", fontsize=9, color=TEXT_2)
    status = fig.text(0.99, 0.995, "", ha="right", va="top", fontsize=9, color=TEXT_2, family="monospace")
    fig.text(0.01, 0.995, f"Mahony  Kp={args.kp}  Ki={args.ki}  {'9-DOF, mag: ' + mag_src if args.mag else '6-DOF accel+gyro'}"
             f"   accel cal {'ON' if acal else 'off'}", ha="left", va="top", fontsize=10, color=TEXT_2)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

    def update(_):
        st["buf"] += ser.read(ser.in_waiting or 1)
        while b"\n" in st["buf"]:
            raw, st["buf"] = st["buf"].split(b"\n", 1)
            v = parse(raw.decode("ascii", errors="ignore").strip())
            if not v:
                continue
            a = np.array(v[0:3], float) / A_LSB
            if acal:
                a = (a - acal[0]) * acal[1]
            g = (np.array(v[3:6], float) / G_LSB - gbias) * D2R
            an = float(np.linalg.norm(a))
            used = abs(an - 1.0) < 0.15          # ignore accel while it sees more than gravity
            if args.mag:
                m = mag_vec(v[6:9])
                f.update(*g, *a, dt, *m, accel_ok=used)
            else:
                f.update(*g, *a, dt, accel_ok=used)
            st["t"] += dt; st["n"] += 1; st["anorm"] = an; st["used"] = used
            r, p, y = f.euler_deg()
            w.writerow([f"{st['t']:.3f}", f"{r:.3f}", f"{p:.3f}", f"{y:.3f}", f"{an:.4f}", int(used)])
            if st["n"] % 2 == 0:
                T.append(st["t"]); E[0].append(r); E[1].append(p); E[2].append(y)
                while T and T[0] < st["t"] - args.window:
                    T.popleft(); [e.popleft() for e in E]
        fh.flush()
        if not T:
            return []
        Rm = np.array(f.rotmat())
        c = corners @ Rm.T
        board.set_data(c[:, 0], c[:, 1]); board.set_3d_properties(c[:, 2])
        for k in range(3):
            e = Rm[:, k] * 1.1
            axes_lines[k].set_data([0, e[0]], [0, e[1]]); axes_lines[k].set_3d_properties([0, e[2]])
        r, p, y = E[0][-1], E[1][-1], E[2][-1]
        title3.set_text(f"roll {r:+6.1f}   pitch {p:+6.1f}   yaw {y:+6.1f} deg")
        for i in range(3):
            lines[i].set_data(T, E[i]); bigs[i].set_text(f"{E[i][-1]:+.1f}°")
            tax[i].set_xlim(max(0, T[-1] - args.window), max(T[-1], 1))
        # density of each angle over the last pdf_window seconds
        st["frame"] = st.get("frame", 0) + 1
        if st["frame"] % 3 == 0:                          # ~5 times a second
            tt = np.fromiter(T, float)
            sel = tt >= tt[-1] - args.pdf_window
            for i in range(3):
                v = np.fromiter(E[i], float)[sel]
                if len(v) < 5:
                    continue
                if i != 1 and np.ptp(v) > 180:                 # roll / yaw crossing +-180
                    v = np.degrees(np.unwrap(np.radians(v)))
                    if np.mean(v) > 180: v = v - 360
                    if np.mean(v) < -180: v = v + 360
                mu, sd = v.mean(), v.std()
                half = max(4 * sd, (v.max() - v.min()) / 2 + sd, 0.5)
                lo, hi = mu - half, mu + half
                if phist[i] is not None:
                    phist[i].remove()
                _, _, bars = pax[i].hist(v, bins=30, range=(lo, hi), density=True, color=cols[i], alpha=0.25)
                phist[i] = bars
                # Gaussian kernel density, Silverman bandwidth
                h = max(1.06 * sd * len(v) ** -0.2, (hi - lo) / 200)
                x = np.linspace(lo, hi, 200)
                pdf = np.exp(-0.5 * ((x[:, None] - v[None, :]) / h) ** 2).sum(1) / (len(v) * h * math.sqrt(2 * math.pi))
                pkde[i].set_data(x, pdf)
                pax[i].set_xlim(lo, hi); pax[i].set_ylim(0, pdf.max() * 1.25 + 1e-9)
                ptxt[i].set_text(f"mean {mu:+.2f}\nsd   {sd:.2f}\nn    {len(v)}")
        bdeg = [-math.degrees(b) for b in f.ib]
        status.set_text(f"|a| {st['anorm']:.3f} g  accel {'used' if st['used'] else 'IGNORED'}   "
                        f"learned extra bias {bdeg[0]:+.3f} {bdeg[1]:+.3f} {bdeg[2]:+.3f} deg/s   samples {st['n']}")
        return []

    anim = FuncAnimation(fig, update, interval=60, cache_frame_data=False)
    try:
        plt.show()
    finally:
        fh.close(); ser.close()
        print(f"Saved {out}")


if __name__ == "__main__":
    main()
