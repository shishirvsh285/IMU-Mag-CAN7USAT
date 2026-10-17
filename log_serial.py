"""
Read the BMI270 init sketch output from the serial port, save it to CSV,
and live-plot the errors while the board sits still.

Errors shown (board at rest):
  - accel magnitude error  |a| - 1 g   (mg)
  - gyro reading on each axis          (deg/s, should be 0)

Usage:
  pip install pyserial matplotlib
  python log_serial.py --port COM5
  python log_serial.py --port COM5 --out run1.csv --window 120 --duration 600

Close the Arduino Serial Monitor first, only one program can hold the port.
Close the plot window (or Ctrl+C) to stop. The CSV is written as data arrives.
"""
import argparse
import csv
import math
import sys
import time
from collections import deque

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial

from imu_common import AXIS_COLORS, COLUMNS, MAG_COLOR, SURFACE, TEXT_2, parse_line, style


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="e.g. COM5 or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default=None, help="CSV file (default: imu_log_<time>.csv)")
    p.add_argument("--window", type=float, default=60.0, help="seconds shown in live plot")
    p.add_argument("--duration", type=float, default=0, help="stop after N seconds (0 = run until closed)")
    args = p.parse_args()

    out = args.out or time.strftime("imu_log_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print(f"Opened {args.port}. Opening the port resets the Uno, waiting for data...")
    ser.reset_input_buffer()

    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(COLUMNS)

    t = deque(); mag = deque(); g = {k: deque() for k in "xyz"}
    t0 = None
    buf = b""
    n = 0

    fig, (ax_a, ax_g) = plt.subplots(2, 1, sharex=True, figsize=(10, 6.5), facecolor=SURFACE)
    style(ax_a, "Accel magnitude error  |a| − 1 g", "mg")
    style(ax_g, "Gyro reading at rest (should be 0)", "deg/s")
    ax_g.set_xlabel("time (s)", fontsize=9, color=TEXT_2)
    l_mag, = ax_a.plot([], [], color=MAG_COLOR, linewidth=1.5)
    l_g = {k: ax_g.plot([], [], color=AXIS_COLORS[k], linewidth=1.5, label=f"gyro {k.upper()}")[0] for k in "xyz"}
    ax_g.axhline(0, color=TEXT_2, linewidth=0.8)
    ax_g.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=3)
    stats = fig.text(0.99, 0.995, "", ha="right", va="top", fontsize=8, color=TEXT_2, family="monospace")
    fig.tight_layout(rect=(0, 0, 1, 0.93))

    def update(_):
        nonlocal buf, t0, n
        buf += ser.read(ser.in_waiting or 1)
        while b"\n" in buf:
            raw, buf = buf.split(b"\n", 1)
            line = raw.decode("ascii", errors="ignore").strip()
            vals = parse_line(line)
            if vals is None:
                if line:
                    print(">>", line)           # status lines like "BMI270 init OK"
                continue
            now = time.time()
            if t0 is None:
                t0 = now
            ts = now - t0
            w.writerow([f"{ts:.3f}"] + vals)
            n += 1
            ax, ay, az, gx, gy, gz = vals
            t.append(ts)
            mag.append((math.sqrt(ax * ax + ay * ay + az * az) - 1.0) * 1000.0)
            for k, v in zip("xyz", (gx, gy, gz)):
                g[k].append(v)
            while t and t[0] < ts - args.window:
                t.popleft(); mag.popleft()
                for k in "xyz":
                    g[k].popleft()
        f.flush()

        if t:
            l_mag.set_data(t, mag)
            for k in "xyz":
                l_g[k].set_data(t, g[k])
            for a in (ax_a, ax_g):
                a.relim(); a.autoscale_view()
            ax_g.set_xlim(max(0, t[-1] - args.window), max(t[-1], 1))
            m = sum(mag) / len(mag)
            sd = math.sqrt(sum((v - m) ** 2 for v in mag) / len(mag))
            bias = "  ".join(f"{k.upper()} {sum(g[k]) / len(g[k]):+.3f}" for k in "xyz")
            stats.set_text(f"|a| err mean {m:+.2f} mg  sd {sd:.2f} mg\n"
                           f"gyro mean (deg/s): {bias}\nsamples {n}")
            if args.duration and t[-1] >= args.duration:
                plt.close(fig)
        return []

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        f.close()
        ser.close()
        print(f"Saved {n} samples to {out}")
        print(f"Next: python plot_errors.py {out}")


if __name__ == "__main__":
    sys.exit(main())
