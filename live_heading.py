"""
Live window with three magnetometer angles, one per plane.

  Heading (XY plane) = atan2(-Y, X)   compass heading of the chip X axis, board flat, 0 = N, 90 = E
  YZ plane angle     = atan2(Z, Y)    field direction seen in the Y-Z plane
  ZX plane angle     = atan2(X, Z)    field direction seen in the Z-X plane

Each is the direction of the magnetic field projected onto that plane, 0..360 deg.
If mag_cal.json exists (from mag_heading.py --make-cal) the X/Y hard-iron correction is applied.
Red dots = |B| far from normal (magnet, phone or steel nearby).

  python live_heading.py --port COM5
  python live_heading.py --port COM5 --out head1.csv --window 30
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

DISTURB = "#e34948"


def ang(a, b):
    return math.degrees(math.atan2(a, b)) % 360.0


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default=None)
    p.add_argument("--window", type=float, default=30.0)
    p.add_argument("--cal", default="mag_cal.json")
    p.add_argument("--disturb", type=float, default=0.5, help="flag if |B| off the running median by this fraction")
    args = p.parse_args()

    cal = None
    if os.path.exists(args.cal):
        with open(args.cal) as f:
            cal = json.load(f)
        print(f"Using calibration {args.cal}")
    else:
        print("No mag_cal.json, showing RAW angles (hard-iron offset not removed)")

    out = args.out or time.strftime("heading_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "mx_mG", "my_mG", "mz_mG", "heading_xy_deg", "angle_yz_deg", "angle_zx_deg", "disturbed"])

    t = deque(); A = [deque(), deque(), deque()]; D = deque(); Bh = deque(maxlen=500)
    t0 = None; buf = b""; n = 0

    names = ["Heading, XY plane  atan2(−Y, X)", "YZ plane  atan2(Z, Y)", "ZX plane  atan2(X, Z)"]
    cols = [AXIS_COLORS["z"], AXIS_COLORS["x"], AXIS_COLORS["y"]]
    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(11, 8.5), facecolor=SURFACE)
    good_l, bad_l = [], []
    for a, nm, c in zip(axs, names, cols):
        style(a, nm, "deg")
        a.set_ylim(0, 360); a.set_yticks([0, 90, 180, 270, 360])
        good_l.append(a.plot([], [], linestyle="none", marker=".", markersize=3, color=c)[0])
        bad_l.append(a.plot([], [], linestyle="none", marker=".", markersize=3, color=DISTURB)[0])
    axs[0].set_yticklabels(["0 N", "90 E", "180 S", "270 W", "360 N"])
    axs[-1].set_xlabel("time (s)", fontsize=9, color=TEXT_2)
    big = [a.text(0.995, 0.93, "", transform=a.transAxes, ha="right", va="top",
                  fontsize=20, fontweight="bold", color="#0b0b0b") for a in axs]
    status = fig.text(0.99, 0.995, "", ha="right", va="top", fontsize=9, color=TEXT_2, family="monospace")
    mode = "calibrated XY" if cal else "RAW (uncalibrated)"
    fig.text(0.01, 0.995, f"IIS2MDC live angles, {mode}", ha="left", va="top", fontsize=10, color=TEXT_2)
    fig.tight_layout(rect=(0, 0, 1, 0.96))

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
            x, y, z = v
            if cal:
                x = (x - cal["off_x_mG"]) * cal["scale_x"]
                y = (y - cal["off_y_mG"]) * cal["scale_y"]
            B = math.sqrt(x * x + y * y + z * z)
            Bh.append(B)
            med = sorted(Bh)[len(Bh) // 2]
            dist = len(Bh) > 20 and abs(B - med) > args.disturb * med
            angs = (ang(-y, x), ang(z, y), ang(x, z))
            now = time.time(); t0 = t0 or now; ts = now - t0
            w.writerow([f"{ts:.3f}", *v, *[f"{a:.2f}" for a in angs], int(dist)])
            n += 1
            t.append(ts); D.append(dist)
            for k in range(3):
                A[k].append(angs[k])
            while t and t[0] < ts - args.window:
                t.popleft(); D.popleft()
                for k in range(3):
                    A[k].popleft()
        f.flush()
        if t:
            tl, dl = list(t), list(D)
            for k in range(3):
                ak = list(A[k])
                good_l[k].set_data([a for a, d in zip(tl, dl) if not d], [b for b, d in zip(ak, dl) if not d])
                bad_l[k].set_data([a for a, d in zip(tl, dl) if d], [b for b, d in zip(ak, dl) if d])
                big[k].set_text(f"{ak[-1]:6.1f}°")
                big[k].set_color(DISTURB if dl[-1] else "#0b0b0b")
            axs[-1].set_xlim(max(0, tl[-1] - args.window), max(tl[-1], 1))
            status.set_text(f"|B| {Bh[-1]:.0f} mG   samples {n}" + ("   DISTURBED" if dl[-1] else ""))
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
