"""
Plot the errors from a CSV saved by log_serial.py (board kept still while logging).

  python plot_errors.py imu_log_xxx.csv            -> shows plots, saves <csv>_errors.png
  python plot_errors.py imu_log_xxx.csv --skip 5   -> drop the first 5 s (warm-up)
  python plot_errors.py imu_log_xxx.csv --skip 2 --end 27 --tag still  -> analyse only 2..27 s

Also accepts a raw text copy of the Serial Monitor (lines with "acc g: ... gyr dps: ...").
In that case time is the sample index / --rate.

Panels:
  1. accel per-axis error, reading minus its own mean (mg)  -> noise and drift
  2. accel magnitude error |a| - 1 g (mg)                    -> offset + scale error
  3. gyro per-axis reading (deg/s), at rest this is the error -> bias, noise, drift
  4. histograms of the gyro error
A summary table is printed to the terminal.
"""
import argparse
import csv
import math
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

from imu_common import AXIS_COLORS, MAG_COLOR, SURFACE, TEXT_2, parse_line, style


def load(path, rate):
    with open(path, newline="", encoding="utf-8", errors="ignore") as f:
        first = f.readline()
        f.seek(0)
        if first.startswith("t_s"):
            r = csv.DictReader(f)
            rows = [[float(x[c]) for c in ("t_s", "ax_g", "ay_g", "az_g", "gx_dps", "gy_dps", "gz_dps")] for x in r]
            return np.array(rows)
        data = [v for v in (parse_line(line) for line in f) if v is not None]
    arr = np.array(data)
    t = np.arange(len(arr)) / rate
    return np.column_stack([t, arr])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("file")
    p.add_argument("--skip", type=float, default=0, help="drop the first N seconds")
    p.add_argument("--end", type=float, default=0, help="ignore data after N seconds (0 = keep all)")
    p.add_argument("--rate", type=float, default=5.0, help="sample rate for raw text logs (Hz)")
    p.add_argument("--tag", default="", help="added to output file names, e.g. still")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    d = load(args.file, args.rate)
    d = d[d[:, 0] >= args.skip]
    if args.end:
        d = d[d[:, 0] <= args.end]
    if len(d) < 10:
        raise SystemExit("Not enough samples in file.")

    t = d[:, 0]
    acc = d[:, 1:4]
    gyr = d[:, 4:7]
    acc_err = (acc - acc.mean(axis=0)) * 1000.0               # mg
    mag_err = (np.linalg.norm(acc, axis=1) - 1.0) * 1000.0    # mg

    # ---- summary ----
    dur = t[-1] - t[0]
    print(f"\n{len(t)} samples over {dur:.1f} s  (~{len(t) / max(dur, 1e-9):.1f} Hz)\n")
    print(f"{'channel':<14}{'mean':>10}{'std':>10}{'min':>10}{'max':>10}{'drift':>10}   unit")
    def row(name, v, unit):
        half = len(v) // 2
        drift = v[half:].mean() - v[:half].mean()
        print(f"{name:<14}{v.mean():>10.3f}{v.std():>10.3f}{v.min():>10.3f}{v.max():>10.3f}{drift:>10.3f}   {unit}")
    for i, k in enumerate("XYZ"):
        row(f"acc {k} raw", acc[:, i], "g")
    row("|a| - 1g", mag_err, "mg")
    for i, k in enumerate("XYZ"):
        row(f"gyro {k}", gyr[:, i], "deg/s")
    print("\ndrift = mean of 2nd half minus mean of 1st half")
    if np.ptp(acc[:, 2]) * 1000 < 3 or np.ptp(gyr[:, 0]) < 0.03:
        print("Note: values are printed with few decimals by the sketch, so steps of 1 mg / 0.01 deg/s show up.")

    # ---- plots ----
    fig, axs = plt.subplots(4, 1, figsize=(10, 11), facecolor=SURFACE,
                            gridspec_kw={"height_ratios": [1, 1, 1, 0.9]})
    a1, a2, a3, a4 = axs

    style(a1, "Accel noise (reading − mean)", "mg")
    for i, k in enumerate("xyz"):
        a1.plot(t, acc_err[:, i], color=AXIS_COLORS[k], linewidth=1.2,
                label=f"{k.upper()}: sd {acc_err[:, i].std():.2f} mg")
    a1.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=3)

    style(a2, f"Accel magnitude error  |a| − 1 g   (mean {mag_err.mean():+.2f} mg, sd {mag_err.std():.2f} mg)", "mg")
    a2.plot(t, mag_err, color=MAG_COLOR, linewidth=1.2)
    a2.axhline(mag_err.mean(), color=TEXT_2, linewidth=0.8, linestyle="--")

    style(a3, "Gyro at rest (should be 0)", "deg/s")
    for i, k in enumerate("xyz"):
        a3.plot(t, gyr[:, i], color=AXIS_COLORS[k], linewidth=1.2,
                label=f"{k.upper()}: bias {gyr[:, i].mean():+.3f}, sd {gyr[:, i].std():.3f}")
    a3.axhline(0, color=TEXT_2, linewidth=0.8)
    a3.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=3)
    a3.set_xlabel("time (s)", fontsize=9, color=TEXT_2)

    style(a4, "Gyro error distribution", "count")
    lo, hi = gyr.min(), gyr.max()
    bins = np.linspace(lo, hi, 40) if hi > lo else 10
    for i, k in enumerate("xyz"):
        a4.hist(gyr[:, i], bins=bins, histtype="step", linewidth=1.5,
                color=AXIS_COLORS[k], label=f"gyro {k.upper()}")
    a4.axvline(0, color=TEXT_2, linewidth=0.8)
    a4.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=3)
    a4.set_xlabel("deg/s", fontsize=9, color=TEXT_2)

    fig.tight_layout()
    out = os.path.splitext(args.file)[0] + (f"_{args.tag}" if args.tag else "") + "_errors.png"
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"\nSaved plot to {out}")

    # ---- value histograms: accel and gyro, one panel per axis ----
    # Each panel gets its own x range, since accel Z sits near 1 g and X/Y near 0.
    fig2, hx = plt.subplots(2, 3, figsize=(12, 6.5), facecolor=SURFACE)
    sets = [(acc, "Accel", "g", 4), (gyr, "Gyro", "deg/s", 3)]
    for r, (data, name, unit, dec) in enumerate(sets):
        for i, k in enumerate("xyz"):
            a = hx[r, i]
            v = data[:, i]
            style(a, f"{name} {k.upper()}", "count" if i == 0 else "")
            # one bin per printed step when values are quantised, else ~40 bins
            steps = np.unique(v)
            step = np.min(np.diff(steps)) if len(steps) > 1 else 1.0
            nstep = (steps[-1] - steps[0]) / step + 1 if len(steps) > 1 else 1
            if nstep <= 60:
                bins = np.arange(steps[0] - step / 2, steps[-1] + step, step)
            else:
                # wide spread (e.g. board moved): ~40 bins, aligned to the print step
                width = step * max(1, round((steps[-1] - steps[0]) / 40 / step))
                bins = np.arange(steps[0] - step / 2, steps[-1] + width, width)
            a.hist(v, bins=bins, color=AXIS_COLORS[k], edgecolor=SURFACE, linewidth=1.0)
            a.axvline(v.mean(), color=TEXT_2, linewidth=1.0, linestyle="--")
            a.set_xlabel(unit, fontsize=9, color=TEXT_2)
            a.text(0.98, 0.95, f"mean {v.mean():+.{dec}f}\nsd {v.std():.{dec}f}",
                   transform=a.transAxes, ha="right", va="top", fontsize=8, color=TEXT_2,
                   family="monospace")
            a.ticklabel_format(axis="x", useOffset=False)
            a.xaxis.set_major_locator(MaxNLocator(5))
    fig2.tight_layout()
    out2 = os.path.splitext(args.file)[0] + (f"_{args.tag}" if args.tag else "") + "_histograms.png"
    fig2.savefig(out2, dpi=150, facecolor=SURFACE)
    print(f"Saved histograms to {out2}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
