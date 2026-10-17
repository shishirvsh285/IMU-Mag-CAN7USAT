"""
Live window: magnetic field vector projected onto the three axis planes.

  XY plane (top view)    -> points (X, Y)
  YZ plane (side view)   -> points (Y, Z)
  ZX plane (front view)  -> points (Z, X)

Each panel shows the trail of recent samples and an arrow for the latest vector,
with its angle in the plane. Rotating the board about one axis traces a circle
in the plane at right angles to it. The circle centre is the hard-iron offset,
which is what calibration removes.
If mag_cal.json exists, the X/Y correction is applied.

  python live_projection.py --port COM5
  python live_projection.py --port COM5 --trail 1500 --out proj1.csv
"""
import argparse
import csv
import json
import math
import os
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

from imu_common import AXIS_COLORS, SURFACE, TEXT_2, parse_mag_line, style

INK = "#0b0b0b"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default=None)
    p.add_argument("--trail", type=int, default=1000, help="number of recent samples drawn (50 per second)")
    p.add_argument("--cal", default="mag_cal.json")
    args = p.parse_args()

    cal = None
    if os.path.exists(args.cal):
        with open(args.cal) as f:
            cal = json.load(f)

    out = args.out or time.strftime("proj_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "mx_mG", "my_mG", "mz_mG"])

    X, Y, Z = deque(maxlen=args.trail), deque(maxlen=args.trail), deque(maxlen=args.trail)
    t0 = None; buf = b""; n = 0
    lim = [300.0]

    # (horizontal data, vertical data, labels, colour, angle formula label)
    planes = [
        ("XY plane (top view)", "X (mG)", "Y (mG)", AXIS_COLORS["z"], "atan2(−Y, X)"),
        ("YZ plane (side view)", "Y (mG)", "Z (mG)", AXIS_COLORS["x"], "atan2(Z, Y)"),
        ("ZX plane (front view)", "Z (mG)", "X (mG)", AXIS_COLORS["y"], "atan2(X, Z)"),
    ]
    fig, axs = plt.subplots(1, 3, figsize=(15, 5.6), facecolor=SURFACE)
    trails, heads, arrows, labels = [], [], [], []
    for a, (title, xl, yl, c, fl) in zip(axs, planes):
        style(a, f"{title}   angle = {fl}", yl)
        a.set_xlabel(xl, fontsize=9, color=TEXT_2)
        a.axhline(0, color=TEXT_2, linewidth=0.7); a.axvline(0, color=TEXT_2, linewidth=0.7)
        a.set_aspect("equal")
        trails.append(a.plot([], [], linestyle="none", marker=".", markersize=2.5, color=c, alpha=0.45)[0])
        heads.append(a.plot([], [], linestyle="none", marker="o", markersize=8, color=c,
                            markeredgecolor=SURFACE, markeredgewidth=2)[0])
        arrows.append(a.annotate("", xy=(0, 0), xytext=(0, 0),
                                 arrowprops=dict(arrowstyle="-|>", color=INK, linewidth=1.8)))
        labels.append(a.text(0.03, 0.97, "", transform=a.transAxes, ha="left", va="top",
                             fontsize=15, fontweight="bold", color=INK))
    status = fig.text(0.99, 0.99, "", ha="right", va="top", fontsize=9, color=TEXT_2, family="monospace")
    fig.text(0.01, 0.99, "IIS2MDC field projections, " + ("calibrated XY" if cal else "RAW (uncalibrated)"),
             ha="left", va="top", fontsize=10, color=TEXT_2)
    fig.tight_layout(rect=(0, 0, 1, 0.94))

    def update(_):
        nonlocal buf, t0, n
        buf += ser.read(ser.in_waiting or 1)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", errors="ignore").strip()
            v = parse_mag_line(line)
            if v is None:
                if line:
                    print(">>", line)
                continue
            now = time.time(); t0 = t0 or now
            w.writerow([f"{now - t0:.3f}", *v])
            x, y, z = v
            if cal:
                x = (x - cal["off_x_mG"]) * cal["scale_x"]
                y = (y - cal["off_y_mG"]) * cal["scale_y"]
            X.append(x); Y.append(y); Z.append(z)
            n += 1
        f.flush()
        if not X:
            return []
        data = [(X, Y), (Y, Z), (Z, X)]
        x, y, z = X[-1], Y[-1], Z[-1]
        cur = [(x, y), (y, z), (z, x)]
        angs = [math.degrees(math.atan2(-y, x)) % 360, math.degrees(math.atan2(z, y)) % 360,
                math.degrees(math.atan2(x, z)) % 360]
        # grow the shared, square limits to fit the data (never shrink, so the view stays steady)
        m = max(max(abs(v) for v in X), max(abs(v) for v in Y), max(abs(v) for v in Z))
        if m > lim[0] * 0.95:
            lim[0] = m * 1.15
        for k, a in enumerate(axs):
            trails[k].set_data(list(data[k][0]), list(data[k][1]))
            heads[k].set_data([cur[k][0]], [cur[k][1]])
            arrows[k].xy = cur[k]; arrows[k].set_position((0, 0))
            labels[k].set_text(f"{angs[k]:.1f}°")
            a.set_xlim(-lim[0], lim[0]); a.set_ylim(-lim[0], lim[0])
        B = math.sqrt(x * x + y * y + z * z)
        status.set_text(f"X {x:7.1f}  Y {y:7.1f}  Z {z:7.1f} mG   |B| {B:6.1f}   samples {n}")
        return []

    anim = FuncAnimation(fig, update, interval=80, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        f.close(); ser.close()
        print(f"Saved {n} samples to {out}")


if __name__ == "__main__":
    main()
