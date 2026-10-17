"""
Plot magnetometer errors from a CSV saved by log_mag.py (board kept still).

  python plot_mag.py mag1.csv
  python plot_mag.py mag1.csv --skip 2 --end 40 --tag still

Figure 1 (<csv>_mag_errors.png):
  1. per-axis noise, reading minus mean (mG)
  2. field magnitude |B| (mG), should be constant
  3. raw heading atan2(-Y, X) minus its mean (deg), the heading jitter you would see
Figure 2 (<csv>_mag_histograms.png):
  value histograms for X, Y, Z (mG) and heading (deg)
Also accepts a raw Serial Monitor text copy (lines with "mag mG: x y z"); time = index / --rate.
Note: heading here is uncalibrated and assumes the board is flat. It shows
stability (noise, drift), not true north.
"""
import argparse
import csv
import os

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import MaxNLocator

from imu_common import AXIS_COLORS, MAG_COLOR, SURFACE, TEXT_2, parse_mag_line, style


def load(path, rate):
    with open(path, newline="", encoding="utf-8", errors="ignore") as f:
        first = f.readline(); f.seek(0)
        if first.startswith("t_s"):
            return np.array([[float(r[c]) for c in ("t_s", "mx_mG", "my_mG", "mz_mG")] for r in csv.DictReader(f)])
        data = [v for v in (parse_mag_line(l) for l in f) if v is not None]
    arr = np.array(data)
    return np.column_stack([np.arange(len(arr)) / rate, arr])


def hist_bins(v, n=40):
    steps = np.unique(v)
    if len(steps) < 2:
        return 10
    step = np.min(np.diff(steps))
    if (steps[-1] - steps[0]) / step + 1 <= 60:
        return np.arange(steps[0] - step / 2, steps[-1] + step, step)
    width = step * max(1, round((steps[-1] - steps[0]) / n / step))
    return np.arange(steps[0] - step / 2, steps[-1] + width, width)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("file")
    p.add_argument("--skip", type=float, default=0)
    p.add_argument("--end", type=float, default=0)
    p.add_argument("--rate", type=float, default=50.0, help="sample rate for raw text logs (Hz)")
    p.add_argument("--tag", default="")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    d = load(args.file, args.rate)
    d = d[d[:, 0] >= args.skip]
    if args.end:
        d = d[d[:, 0] <= args.end]
    if len(d) < 10:
        raise SystemExit("Not enough samples.")

    t, m = d[:, 0], d[:, 1:4]
    noise = m - m.mean(axis=0)
    B = np.linalg.norm(m, axis=1)
    head = np.degrees(np.unwrap(np.arctan2(-m[:, 1], m[:, 0])))
    head = head - 360 * np.floor(head.mean() / 360)   # keep the mean inside 0..360
    head_err = head - head.mean()

    dur = t[-1] - t[0]
    print(f"\n{len(t)} samples over {dur:.1f} s  (~{len(t) / max(dur, 1e-9):.1f} Hz)\n")
    print(f"{'channel':<12}{'mean':>10}{'std':>10}{'min':>10}{'max':>10}{'drift':>10}   unit")
    def row(name, v, unit):
        h = len(v) // 2
        print(f"{name:<12}{v.mean():>10.2f}{v.std():>10.2f}{v.min():>10.2f}{v.max():>10.2f}"
              f"{v[h:].mean() - v[:h].mean():>10.2f}   {unit}")
    for i, k in enumerate("XYZ"):
        row(f"mag {k}", m[:, i], "mG")
    row("|B|", B, "mG")
    row("heading", head, "deg")
    print("\ndrift = mean of 2nd half minus mean of 1st half")
    print("datasheet: RMS noise 3 mG typ (4.6 max) high-res mode, 1.5 mG/LSB")

    suffix = (f"_{args.tag}" if args.tag else "")
    base = os.path.splitext(args.file)[0] + suffix

    fig, (a1, a2, a3) = plt.subplots(3, 1, sharex=True, figsize=(10, 8.5), facecolor=SURFACE)
    style(a1, "Mag noise (reading − mean)", "mG")
    for i, k in enumerate("xyz"):
        a1.plot(t, noise[:, i], color=AXIS_COLORS[k], linewidth=1.1,
                label=f"{k.upper()}: sd {noise[:, i].std():.2f} mG")
    a1.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=3)
    style(a2, f"Field magnitude |B|   (mean {B.mean():.1f} mG, sd {B.std():.2f} mG)", "mG")
    a2.plot(t, B, color=MAG_COLOR, linewidth=1.2)
    a2.axhline(B.mean(), color=TEXT_2, linewidth=0.8, linestyle="--")
    style(a3, f"Raw heading error (heading − mean {head.mean():+.2f}°)   sd {head_err.std():.3f}°", "deg")
    a3.plot(t, head_err, color=MAG_COLOR, linewidth=1.2)
    a3.axhline(0, color=TEXT_2, linewidth=0.8)
    a3.set_xlabel("time (s)", fontsize=9, color=TEXT_2)
    fig.tight_layout()
    fig.savefig(base + "_mag_errors.png", dpi=150, facecolor=SURFACE)
    print(f"\nSaved {base}_mag_errors.png")

    fig2, hx = plt.subplots(1, 4, figsize=(15, 3.8), facecolor=SURFACE)
    for i, k in enumerate("xyz"):
        a, v = hx[i], m[:, i]
        style(a, f"Mag {k.upper()}", "count" if i == 0 else "")
        a.hist(v, bins=hist_bins(v), color=AXIS_COLORS[k], edgecolor=SURFACE, linewidth=1.0)
        a.axvline(v.mean(), color=TEXT_2, linewidth=1.0, linestyle="--")
        a.set_xlabel("mG", fontsize=9, color=TEXT_2)
        a.text(0.98, 0.95, f"mean {v.mean():+.1f}\nsd {v.std():.2f}", transform=a.transAxes,
               ha="right", va="top", fontsize=8, color=TEXT_2, family="monospace")
        a.xaxis.set_major_locator(MaxNLocator(5))
    a = hx[3]
    style(a, "Raw heading", "")
    a.hist(head, bins=40, color=MAG_COLOR, edgecolor=SURFACE, linewidth=1.0)
    a.axvline(head.mean(), color=TEXT_2, linewidth=1.0, linestyle="--")
    a.set_xlabel("deg", fontsize=9, color=TEXT_2)
    a.text(0.98, 0.95, f"mean {head.mean():+.2f}\nsd {head.std():.3f}", transform=a.transAxes,
           ha="right", va="top", fontsize=8, color=TEXT_2, family="monospace")
    a.xaxis.set_major_locator(MaxNLocator(5))
    a.ticklabel_format(axis="x", useOffset=False)
    fig2.tight_layout()
    fig2.savefig(base + "_mag_histograms.png", dpi=150, facecolor=SURFACE)
    print(f"Saved {base}_mag_histograms.png")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
