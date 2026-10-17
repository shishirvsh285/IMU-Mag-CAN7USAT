"""
Heading angles from the IIS2MDC magnetometer.

heading = atan2(-Y, X), 0..360 deg clockwise from magnetic north, for the chip's X axis.
Board must be flat and face up (no tilt correction yet).

Step 1, calibration (optional but needed for a true heading):
  Log while turning the board slowly flat on the table, 2 full turns, away from
  laptops / magnets / steel:   python log_mag.py --port COM5 --out magcal.csv
  Then:                        python mag_heading.py magcal.csv --make-cal
  This fits the hard-iron offset (circle centre) and a simple X/Y scale and saves mag_cal.json.

Step 2, headings:
  python mag_heading.py mag1.csv                 -> uses mag_cal.json if it exists
  python mag_heading.py mag1.csv --no-cal        -> raw heading
  python mag_heading.py mag1.csv --declination -0.5   (add local declination for true north)

Outputs: <csv>[_tag]_heading.csv (t, heading, disturbed flag) and <csv>[_tag]_heading.png
Samples where |B| is far from normal (magnet, phone, steel nearby) are flagged as disturbed
and left out of the summary.
"""
import argparse
import json
import os

import matplotlib.pyplot as plt
import numpy as np

from imu_common import MAG_COLOR, SURFACE, TEXT_2, AXIS_COLORS, mag_heading_deg, style
from plot_mag import load

DISTURB_COLOR = "#e34948"


def circ_stats(h):
    """Circular mean and spread (deg) of headings in degrees."""
    r = np.radians(h)
    c, s = np.cos(r).mean(), np.sin(r).mean()
    mean = np.degrees(np.arctan2(s, c)) % 360
    dev = (h - mean + 180) % 360 - 180
    return mean, dev.std()


def fit_cal(x, y):
    """
    Least-squares fit of an axis-aligned ellipse  x^2 + a*y^2 + b*x + c*y + d = 0.
    Centre = hard-iron offset. The two radii give the X/Y scale (simple soft-iron).
    """
    A = np.column_stack([y * y, x, y, np.ones_like(x)])
    (a, b, c, d), *_ = np.linalg.lstsq(A, -x * x, rcond=None)
    ox, oy = -b / 2, -c / (2 * a)
    rx = np.sqrt(ox * ox + a * oy * oy - d)
    ry = rx / np.sqrt(a)
    ravg = (rx + ry) / 2
    return {"off_x_mG": float(ox), "off_y_mG": float(oy),
            "scale_x": float(ravg / rx), "scale_y": float(ravg / ry), "radius_mG": float(ravg)}


def main():
    p = argparse.ArgumentParser()
    p.add_argument("file")
    p.add_argument("--make-cal", action="store_true", help="fit calibration from this file (flat 360 deg turns)")
    p.add_argument("--cal", default="mag_cal.json")
    p.add_argument("--no-cal", action="store_true")
    p.add_argument("--declination", type=float, default=0.0, help="deg, east positive; 0 = magnetic heading")
    p.add_argument("--disturb", type=float, default=0.25, help="flag if |B| differs from median by more than this fraction")
    p.add_argument("--skip", type=float, default=0)
    p.add_argument("--end", type=float, default=0)
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--tag", default="")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    d = load(args.file, args.rate)
    d = d[d[:, 0] >= args.skip]
    if args.end:
        d = d[d[:, 0] <= args.end]
    t, x, y, z = d[:, 0], d[:, 1], d[:, 2], d[:, 3]

    def flag(B, frac):
        med = np.median(B)
        return np.abs(B - med) > frac * med

    cal_dir = os.path.dirname(os.path.abspath(args.file))
    cal_path = args.cal if os.path.isabs(args.cal) else os.path.join(cal_dir, args.cal)

    if args.make_cal:
        # without calibration |B| swings with the hard-iron offset, so only drop gross outliers here
        ok = ~flag(np.sqrt(x * x + y * y + z * z), 0.6)
        cal = fit_cal(x[ok], y[ok])
        span = np.ptp(np.unwrap(np.radians(np.degrees(np.arctan2(-(y[ok] - cal["off_y_mG"]), x[ok] - cal["off_x_mG"])))))
        cal["turns_covered"] = float(np.degrees(span) / 360)
        with open(cal_path, "w") as f:
            json.dump(cal, f, indent=2)
        print(f"Saved {cal_path}: {json.dumps(cal)}")
        if cal["turns_covered"] < 1:
            print("Warning: less than one full turn in the data, calibration will be poor.")

    cal = None
    if not args.no_cal and os.path.exists(cal_path):
        with open(cal_path) as f:
            cal = json.load(f)
    if cal:
        xc = (x - cal["off_x_mG"]) * cal["scale_x"]
        yc = (y - cal["off_y_mG"]) * cal["scale_y"]
        mode = "calibrated"
    else:
        xc, yc = x, y
        mode = "RAW (no calibration, offset unknown)"

    head = (mag_heading_deg(xc, yc) + args.declination) % 360

    # disturbance flag: on calibrated data use the set threshold, on raw data be looser
    B = np.sqrt(xc * xc + yc * yc + z * z)
    medB = np.median(B)
    thr = args.disturb if cal else max(args.disturb, 0.6)
    disturbed = flag(B, thr)

    good = ~disturbed
    print(f"\n{len(t)} samples over {t[-1] - t[0]:.1f} s, heading mode: {mode}")
    print(f"|B| median {medB:.1f} mG, disturbed samples: {disturbed.sum()} ({100 * disturbed.mean():.1f} %)")
    if good.sum():
        m, sd = circ_stats(head[good])
        print(f"heading (undisturbed): circular mean {m:.2f} deg, spread sd {sd:.2f} deg")

    base = os.path.splitext(args.file)[0] + (f"_{args.tag}" if args.tag else "")
    with open(base + "_heading.csv", "w") as f:
        f.write("t_s,heading_deg,disturbed\n")
        for ti, hi, di in zip(t, head, disturbed):
            f.write(f"{ti:.3f},{hi:.2f},{int(di)}\n")
    print(f"Saved {base}_heading.csv")

    # ---- plots ----
    fig = plt.figure(figsize=(13, 8), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 2, width_ratios=[2.2, 1])
    a1 = fig.add_subplot(gs[0, 0]); a2 = fig.add_subplot(gs[1, 0], sharex=a1); a3 = fig.add_subplot(gs[:, 1])

    style(a1, f"Heading of chip X axis ({mode})", "deg")
    a1.scatter(t[good], head[good], s=3, color=MAG_COLOR, label="heading")
    if disturbed.any():
        a1.scatter(t[disturbed], head[disturbed], s=3, color=DISTURB_COLOR, label="disturbed |B|")
    a1.set_ylim(0, 360); a1.set_yticks([0, 90, 180, 270, 360])
    a1.set_yticklabels(["0 N", "90 E", "180 S", "270 W", "360 N"])
    a1.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=2, markerscale=3)

    style(a2, f"Field magnitude |B| (median {medB:.0f} mG, flagged beyond ±{thr * 100:.0f} %)", "mG")
    a2.plot(t, B, color=MAG_COLOR, linewidth=1.0)
    a2.axhspan(medB * (1 - thr), medB * (1 + thr), color=TEXT_2, alpha=0.08, linewidth=0)
    if B.max() > 3 * medB:
        a2.set_yscale("log")
    a2.set_xlabel("time (s)", fontsize=9, color=TEXT_2)

    style(a3, "X-Y field (top view)", "Y (mG)")
    a3.scatter(x[good], y[good], s=2, color=AXIS_COLORS["x"], label="raw")
    if cal:
        a3.scatter(xc[good], yc[good], s=2, color=AXIS_COLORS["z"], label="calibrated")
        th = np.linspace(0, 2 * np.pi, 200)
        a3.plot(cal["radius_mG"] * np.cos(th), cal["radius_mG"] * np.sin(th), color=TEXT_2, linewidth=0.8, linestyle="--")
    a3.axhline(0, color=TEXT_2, linewidth=0.6); a3.axvline(0, color=TEXT_2, linewidth=0.6)
    a3.set_aspect("equal", adjustable="datalim")
    a3.set_xlabel("X (mG)", fontsize=9, color=TEXT_2)
    a3.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False, ncol=2, markerscale=4)

    fig.tight_layout()
    fig.savefig(base + "_heading.png", dpi=150, facecolor=SURFACE)
    print(f"Saved {base}_heading.png")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
