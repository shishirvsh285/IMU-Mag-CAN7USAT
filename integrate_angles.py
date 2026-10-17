"""
Integrate each gyro axis into an angle, and plot each axis on its own panel.

  python integrate_angles.py run2.csv
  python integrate_angles.py run2.csv --bias-window 10      -> bias from first 10 s
  python integrate_angles.py run1.csv --skip 1 --end 27 --tag still

For each axis:
  solid line  = raw integration  angle(t) = sum of gyro * dt   (drifts by bias * t)
  dashed line = same, after subtracting the bias measured in the first
                --bias-window seconds (what is left is noise / random walk)

With the board still, both lines should stay at 0. Their slope is the error.
If the board was rotated, the raw line shows the rotation angle about that axis.
Saves <csv>[_tag]_angles.png next to the CSV.
"""
import argparse
import os

import matplotlib.pyplot as plt
import numpy as np

from imu_common import AXIS_COLORS, SURFACE, TEXT_2, style
from plot_errors import load


def integrate(t, w):
    """Trapezoid integration of rate w (deg/s) over time t (s). Returns angle in deg."""
    dt = np.diff(t)
    return np.concatenate([[0.0], np.cumsum(0.5 * (w[1:] + w[:-1]) * dt)])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("file")
    p.add_argument("--skip", type=float, default=0, help="drop the first N seconds")
    p.add_argument("--end", type=float, default=0, help="ignore data after N seconds (0 = keep all)")
    p.add_argument("--bias-window", type=float, default=10.0,
                   help="seconds at the start (board still) used to estimate gyro bias")
    p.add_argument("--rate", type=float, default=5.0, help="sample rate for raw text logs (Hz)")
    p.add_argument("--tag", default="", help="added to output file name")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    d = load(args.file, args.rate)
    d = d[d[:, 0] >= args.skip]
    if args.end:
        d = d[d[:, 0] <= args.end]
    if len(d) < 10:
        raise SystemExit("Not enough samples.")

    t = d[:, 0] - d[0, 0]
    gyr = d[:, 4:7]

    # make sure time is increasing (PC timestamps can repeat when lines arrive together)
    keep = np.concatenate([[True], np.diff(t) > 0])
    t, gyr = t[keep], gyr[keep]

    win = t <= args.bias_window
    if win.sum() < 5:
        win = slice(None)
    bias = gyr[win].mean(axis=0)

    dur = t[-1]
    print(f"\n{len(t)} samples over {dur:.1f} s, bias from first {min(args.bias_window, dur):.1f} s\n")
    print(f"{'axis':<6}{'bias':>10}{'raw end':>12}{'raw rate':>12}{'corr end':>12}{'corr max':>12}")
    print(f"{'':<6}{'deg/s':>10}{'deg':>12}{'deg/min':>12}{'deg':>12}{'deg':>12}")

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8.5), facecolor=SURFACE)
    names = {"x": "X (roll)", "y": "Y (pitch)", "z": "Z (yaw)"}
    for i, k in enumerate("xyz"):
        raw = integrate(t, gyr[:, i])
        corr = integrate(t, gyr[:, i] - bias[i])
        print(f"{k.upper():<6}{bias[i]:>+10.3f}{raw[-1]:>+12.2f}{raw[-1] / dur * 60:>+12.2f}"
              f"{corr[-1]:>+12.2f}{np.abs(corr).max():>12.2f}")

        a = axs[i]
        style(a, f"Gyro {names[k]} integrated angle   "
                 f"(raw end {raw[-1]:+.2f}°, bias-removed end {corr[-1]:+.2f}°)", "deg")
        a.plot(t, raw, color=AXIS_COLORS[k], linewidth=1.8, label="raw integration")
        a.plot(t, corr, color=AXIS_COLORS[k], linewidth=1.5, linestyle="--",
               label=f"bias removed ({bias[i]:+.3f} °/s)")
        a.axhline(0, color=TEXT_2, linewidth=0.8)
        if args.bias_window < dur:
            a.axvspan(0, args.bias_window, color=TEXT_2, alpha=0.06, linewidth=0)
        a.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=2)
    axs[-1].set_xlabel("time (s)   shaded = bias window", fontsize=9, color=TEXT_2)

    print("\nraw rate = drift you would see in flight with no gyro calibration")
    fig.tight_layout()
    out = os.path.splitext(args.file)[0] + (f"_{args.tag}" if args.tag else "") + "_angles.png"
    fig.savefig(out, dpi=150, facecolor=SURFACE)
    print(f"Saved {out}")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
