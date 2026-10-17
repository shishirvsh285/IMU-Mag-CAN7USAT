"""
Read the IIS2MDC sketch output from the serial port, save to CSV, and live-plot.

Panels (board kept still):
  - each axis minus its running mean (mG)   -> noise
  - field magnitude |B| (mG)                -> should stay constant
  - raw heading atan2(-Y, X) (deg, 0..360)  -> should stay constant (board flat, uncalibrated)

  pip install pyserial matplotlib
  python log_mag.py --port COM5
  python log_mag.py --port COM5 --out mag1.csv --duration 60

Close the Arduino Serial Monitor first. Close the plot window to stop.
"""
import argparse
import csv
import math
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

from imu_common import AXIS_COLORS, MAG_COLOR, MAG_COLUMNS, SURFACE, TEXT_2, parse_mag_line, style


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default=None)
    p.add_argument("--window", type=float, default=60.0)
    p.add_argument("--duration", type=float, default=0)
    args = p.parse_args()

    out = args.out or time.strftime("mag_log_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print(f"Opened {args.port}, waiting for data (the Uno resets on open)...")
    ser.reset_input_buffer()
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(MAG_COLUMNS)

    t = deque(); m = {k: deque() for k in "xyz"}; mag = deque(); hd = deque()
    t0 = None; buf = b""; n = 0

    fig, (a1, a2, a3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8), facecolor=SURFACE)
    style(a1, "Mag noise (reading − window mean)", "mG")
    style(a2, "Field magnitude |B|", "mG")
    style(a3, "Raw heading atan2(−Y, X), 0 = N, 90 = E, uncalibrated", "deg")
    a3.set_xlabel("time (s)", fontsize=9, color=TEXT_2)
    l1 = {k: a1.plot([], [], color=AXIS_COLORS[k], linewidth=1.3, label=f"mag {k.upper()}")[0] for k in "xyz"}
    a1.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=3)
    l2, = a2.plot([], [], color=MAG_COLOR, linewidth=1.5)
    l3, = a3.plot([], [], color=MAG_COLOR, linestyle="none", marker=".", markersize=3)
    stats = fig.text(0.99, 0.995, "", ha="right", va="top", fontsize=8, color=TEXT_2, family="monospace")
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
            now = time.time()
            t0 = t0 or now
            ts = now - t0
            w.writerow([f"{ts:.3f}"] + v)
            n += 1
            t.append(ts)
            for k, val in zip("xyz", v):
                m[k].append(val)
            mag.append(math.sqrt(sum(c * c for c in v)))
            hd.append(math.degrees(math.atan2(-v[1], v[0])) % 360.0)
            while t and t[0] < ts - args.window:
                t.popleft(); mag.popleft(); hd.popleft()
                for k in "xyz":
                    m[k].popleft()
        f.flush()
        if t:
            for k in "xyz":
                mean = sum(m[k]) / len(m[k])
                l1[k].set_data(t, [x - mean for x in m[k]])
            l2.set_data(t, mag)
            l3.set_data(t, hd)
            for a in (a1, a2, a3):
                a.relim(); a.autoscale_view()
            a3.set_xlim(max(0, t[-1] - args.window), max(t[-1], 1))
            mb = sum(mag) / len(mag)
            sd = math.sqrt(sum((x - mb) ** 2 for x in mag) / len(mag))
            cs = sum(math.cos(math.radians(h)) for h in hd); sn = sum(math.sin(math.radians(h)) for h in hd)
            hm = math.degrees(math.atan2(sn, cs)) % 360.0
            devs = [((h - hm + 180) % 360) - 180 for h in hd]
            hsd = math.sqrt(sum(e * e for e in devs) / len(devs))
            stats.set_text(f"|B| {mb:.1f} mG  sd {sd:.2f}   heading {hm:.2f}°  sd {hsd:.2f}°   samples {n}")
            if args.duration and t[-1] >= args.duration:
                plt.close(fig)
        return []

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        f.close(); ser.close()
        print(f"Saved {n} samples to {out}\nNext: python plot_mag.py {out}")


if __name__ == "__main__":
    main()
