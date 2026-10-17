"""
Apply a MotionCal magnetometer calibration to logged data and compare before / after.

MotionCal (visualize.c, apply_calibration):  cal = invW @ (raw - V)
with raw and V in uT. Our logs are in mG (1 uT = 10 mG), so V is scaled by 10.

  python apply_motioncal.py ell2.csv
  python apply_motioncal.py ell2.csv proj1.csv --cal mag_cal_motioncal.json --no-show

Writes <csv>_mcal.csv (raw, calibrated, |B|, heading) and <csv>_mcal.png.
A good calibration makes |B| nearly constant (sd a few % or less) in every orientation.
"""
import argparse
import json
import os

import matplotlib.pyplot as plt
import numpy as np

from imu_common import AXIS_COLORS, SURFACE, TEXT_2, mag_heading_deg, style
from plot_mag import load

RAW_C, CAL_C = AXIS_COLORS["x"], AXIS_COLORS["z"]


def main():
    p = argparse.ArgumentParser()
    p.add_argument("files", nargs="+")
    p.add_argument("--cal", default="mag_cal_motioncal.json")
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    with open(args.cal) as f:
        cal = json.load(f)
    V = np.array(cal["offset_uT"]) * 10.0          # mG
    invW = np.array(cal["invW"])
    Bexp = cal["field_uT"] * 10.0                   # mG
    print(f"offset {V.round(1)} mG, expected |B| {Bexp:.1f} mG")

    for path in args.files:
        d = load(path, args.rate)
        t, raw = d[:, 0], d[:, 1:4]
        B = np.linalg.norm(raw, axis=1)
        ok = np.abs(B - np.median(B)) < 0.6 * np.median(B)     # drop magnet spikes
        c = (raw - V) @ invW.T
        Bc = np.linalg.norm(c, axis=1)
        head = mag_heading_deg(c[:, 0], c[:, 1])

        print(f"\n{path}: {len(raw)} samples ({(~ok).sum()} disturbed, left out of stats)")
        print(f"  raw        |B| mean {B[ok].mean():6.1f} mG  sd {B[ok].std():5.1f} ({100 * B[ok].std() / B[ok].mean():4.1f} %)")
        print(f"  calibrated |B| mean {Bc[ok].mean():6.1f} mG  sd {Bc[ok].std():5.1f} ({100 * Bc[ok].std() / Bc[ok].mean():4.1f} %)"
              f"   (MotionCal expects {Bexp:.0f})")

        base = os.path.splitext(path)[0]
        with open(base + "_mcal.csv", "w") as f:
            f.write("t_s,mx_mG,my_mG,mz_mG,mx_cal_mG,my_cal_mG,mz_cal_mG,B_cal_mG,heading_cal_deg,disturbed\n")
            for i in range(len(t)):
                f.write(f"{t[i]:.3f},{raw[i,0]:.1f},{raw[i,1]:.1f},{raw[i,2]:.1f},"
                        f"{c[i,0]:.1f},{c[i,1]:.1f},{c[i,2]:.1f},{Bc[i]:.1f},{head[i]:.2f},{int(not ok[i])}\n")

        fig, axs = plt.subplots(1, 4, figsize=(17, 4.6), facecolor=SURFACE)
        th = np.linspace(0, 2 * np.pi, 200)
        for a, (i, j, nm) in zip(axs[:3], [(0, 1, "XY"), (1, 2, "YZ"), (2, 0, "ZX")]):
            style(a, f"{nm} plane: raw (blue) vs calibrated (green)", f"{'XYZ'[j]} (mG)")
            a.scatter(raw[ok, i], raw[ok, j], s=1.5, color=RAW_C, alpha=0.5)
            a.scatter(c[ok, i], c[ok, j], s=1.5, color=CAL_C, alpha=0.6)
            a.plot(Bexp * np.cos(th), Bexp * np.sin(th), color=TEXT_2, linewidth=0.8, linestyle="--")
            a.axhline(0, color=TEXT_2, linewidth=0.5); a.axvline(0, color=TEXT_2, linewidth=0.5)
            a.set_aspect("equal", adjustable="datalim")
            a.set_xlabel(f"{'XYZ'[i]} (mG)", fontsize=9, color=TEXT_2)
        a = axs[3]
        style(a, "|B| raw vs calibrated", "count")
        bins = np.linspace(min(B[ok].min(), Bc[ok].min()), max(B[ok].max(), Bc[ok].max()), 60)
        a.hist(B[ok], bins=bins, color=RAW_C, alpha=0.6, label=f"raw, sd {B[ok].std():.0f}")
        a.hist(Bc[ok], bins=bins, color=CAL_C, alpha=0.7, label=f"calibrated, sd {Bc[ok].std():.0f}")
        a.axvline(Bexp, color=TEXT_2, linewidth=1, linestyle="--")
        a.set_xlabel("mG (dashed = MotionCal field)", fontsize=9, color=TEXT_2)
        a.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False)
        fig.tight_layout()
        fig.savefig(base + "_mcal.png", dpi=140, facecolor=SURFACE)
        print(f"  saved {base}_mcal.csv and {base}_mcal.png")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
