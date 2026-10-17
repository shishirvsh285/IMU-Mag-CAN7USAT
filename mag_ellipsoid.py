"""
Ellipsoid fit for the IIS2MDC magnetometer (hard-iron + soft-iron calibration).

Turning the board through many orientations makes the raw readings lie on an
ellipsoid. A perfect sensor with nothing magnetic nearby gives a sphere centred
on 0. The fit finds:
  hard-iron offset  b  = centre of the ellipsoid (mG)
  soft-iron matrix  W  = 3x3 matrix that turns the ellipsoid into a sphere
  calibrated value     = W * (raw - b)

  python mag_ellipsoid.py proj1.csv              -> opens a rotatable 3D window, saves PNG + mag_cal3d.json
  python mag_ellipsoid.py proj1.csv --no-show
  python mag_ellipsoid.py proj1.csv --skip 5 --end 300

For a good fit, slowly turn the board through as many directions as you can
(all faces up and down, plus diagonals), away from magnets, phones and steel.
"""
import argparse
import json
import os

import matplotlib.pyplot as plt
import numpy as np

from imu_common import AXIS_COLORS, SURFACE, TEXT_2, style
from plot_mag import load

RAW_C = AXIS_COLORS["x"]
CAL_C = AXIS_COLORS["z"]
FIT_C = "#52514e"


def fit_ellipsoid(v):
    """
    Least-squares general ellipsoid
      a x^2 + b y^2 + c z^2 + 2d xy + 2e xz + 2f yz + 2g x + 2h y + 2i z = 1
    Returns centre (3,), shape matrix M (3x3), and scale k so that
      (v - centre)^T M (v - centre) = k  on the surface.
    """
    x, y, z = v.T
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z])
    p, *_ = np.linalg.lstsq(D, np.ones(len(v)), rcond=None)
    a, b, c, d, e, f, g, h, i = p
    M = np.array([[a, d, e], [d, b, f], [e, f, c]])
    centre = -np.linalg.solve(M, [g, h, i])
    k = 1 + centre @ M @ centre
    return centre, M, k


def soft_iron(M, k):
    """W = R * sqrtm(M / k): maps the ellipsoid onto a sphere of radius R (R = mean radius)."""
    vals, vecs = np.linalg.eigh(M / k)
    if np.any(vals <= 0):
        raise ValueError("Fit is not an ellipsoid (not enough orientations in the data).")
    radii = 1 / np.sqrt(vals)
    R = radii.mean()
    W = vecs @ np.diag(np.sqrt(vals)) @ vecs.T * R
    return W, R, radii, vecs


def ellipsoid_mesh(centre, radii, vecs, n=28):
    u, w = np.linspace(0, 2 * np.pi, n), np.linspace(0, np.pi, n // 2)
    pts = np.stack([np.outer(np.cos(u), np.sin(w)), np.outer(np.sin(u), np.sin(w)),
                    np.outer(np.ones_like(u), np.cos(w))], axis=-1) * radii
    pts = pts @ vecs.T + centre
    return pts[..., 0], pts[..., 1], pts[..., 2]


def set_equal(ax, pts):
    c = (pts.max(0) + pts.min(0)) / 2
    r = (pts.max(0) - pts.min(0)).max() / 2 * 1.05
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect((1, 1, 1))


def style3d(ax, title):
    ax.set_facecolor(SURFACE)
    ax.set_title(title, loc="left", fontsize=10, color="#0b0b0b")
    for lab, fn in (("X (mG)", ax.set_xlabel), ("Y (mG)", ax.set_ylabel), ("Z (mG)", ax.set_zlabel)):
        fn(lab, fontsize=8, color=TEXT_2)
    ax.tick_params(labelsize=7, colors=TEXT_2)
    for pane in (ax.xaxis.pane, ax.yaxis.pane, ax.zaxis.pane):
        pane.set_facecolor(SURFACE); pane.set_edgecolor("#e4e3df")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("file")
    p.add_argument("--skip", type=float, default=0)
    p.add_argument("--end", type=float, default=0)
    p.add_argument("--rate", type=float, default=50.0)
    p.add_argument("--max-points", type=int, default=4000, help="points drawn in 3D (fit uses all)")
    p.add_argument("--out-cal", default="mag_cal3d.json")
    p.add_argument("--tag", default="")
    p.add_argument("--no-show", action="store_true")
    args = p.parse_args()

    d = load(args.file, args.rate)
    d = d[d[:, 0] >= args.skip]
    if args.end:
        d = d[d[:, 0] <= args.end]
    v = d[:, 1:4]

    # 1) drop gross outliers (magnet nearby), 2) fit, 3) drop points far from the fit, refit
    B = np.linalg.norm(v, axis=1)
    keep = np.abs(B - np.median(B)) < 0.6 * np.median(B)
    centre, M, k = fit_ellipsoid(v[keep])
    for _ in range(2):
        W, R, radii, vecs = soft_iron(M, k)
        r_all = np.linalg.norm((v - centre) @ W.T, axis=1) / R
        keep = np.abs(r_all - 1) < 0.15
        centre, M, k = fit_ellipsoid(v[keep])
    W, R, radii, vecs = soft_iron(M, k)

    raw = v[keep]
    cal = (raw - centre) @ W.T
    r_raw = np.linalg.norm(raw, axis=1)          # raw |B|, includes hard/soft-iron effects
    r_cal = np.linalg.norm(cal, axis=1)

    # direction coverage: fraction of 26 direction bins that got data
    u = cal / r_cal[:, None]
    bins = np.round(u * 1.4).astype(int)
    coverage = len({tuple(b) for b in bins if np.any(b)}) / 26

    print(f"\n{len(v)} samples, {keep.sum()} used in fit ({100 * (1 - keep.mean()):.1f} % rejected as disturbed)")
    print(f"direction coverage: {coverage * 100:.0f} % of 26 directions")
    print(f"\nhard-iron offset (mG):  X {centre[0]:+.1f}   Y {centre[1]:+.1f}   Z {centre[2]:+.1f}")
    print(f"ellipsoid radii (mG):   {radii[0]:.1f}  {radii[1]:.1f}  {radii[2]:.1f}   (axis ratio {radii.max() / radii.min():.3f})")
    print(f"field radius after cal: {R:.1f} mG")
    print("soft-iron matrix W:")
    for row in W:
        print("   " + "  ".join(f"{x:+.4f}" for x in row))
    print(f"\n|B| spread  raw: sd {r_raw.std():.1f} mG ({100 * r_raw.std() / R:.1f} %)"
          f"   calibrated: sd {r_cal.std():.1f} mG ({100 * r_cal.std() / R:.2f} %)")
    if coverage < 0.6:
        print("Warning: data covers few directions, the soft-iron part of the fit is unreliable.")

    out_dir = os.path.dirname(os.path.abspath(args.file))
    cal_path = os.path.join(out_dir, args.out_cal)
    with open(cal_path, "w") as f:
        json.dump({"hard_iron_mG": centre.round(3).tolist(), "soft_iron": W.round(6).tolist(),
                   "field_radius_mG": round(float(R), 2),
                   "usage": "cal = soft_iron @ (raw - hard_iron); |cal| ~ field_radius_mG"}, f, indent=2)
    print(f"Saved {cal_path}")

    # ---- figure ----
    rng = np.random.default_rng(0)
    idx = rng.choice(len(raw), min(args.max_points, len(raw)), replace=False)
    fig = plt.figure(figsize=(15, 9), facecolor=SURFACE)
    gs = fig.add_gridspec(2, 3, height_ratios=[2.1, 1])

    a1 = fig.add_subplot(gs[0, 0], projection="3d")
    style3d(a1, "Raw data + fitted ellipsoid")
    ex, ey, ez = ellipsoid_mesh(centre, radii, vecs)
    a1.plot_wireframe(ex, ey, ez, color=FIT_C, linewidth=0.4, alpha=0.5)
    a1.scatter(*raw[idx].T, s=2, color=RAW_C, alpha=0.6, depthshade=False)
    a1.scatter(*centre, s=60, color="#e34948", marker="x", depthshade=False)
    set_equal(a1, np.vstack([raw, np.column_stack([ex.ravel(), ey.ravel(), ez.ravel()])]))

    a2 = fig.add_subplot(gs[0, 1], projection="3d")
    style3d(a2, f"Calibrated data + sphere (R = {R:.0f} mG)")
    sx, sy, sz = ellipsoid_mesh(np.zeros(3), np.array([R, R, R]), np.eye(3))
    a2.plot_wireframe(sx, sy, sz, color=FIT_C, linewidth=0.4, alpha=0.5)
    a2.scatter(*cal[idx].T, s=2, color=CAL_C, alpha=0.6, depthshade=False)
    set_equal(a2, np.vstack([cal, [[R, R, R], [-R, -R, -R]]]))

    a3 = fig.add_subplot(gs[0, 2])
    style(a3, "Field magnitude: raw vs calibrated", "count")
    bins_h = np.linspace(min(r_raw.min(), r_cal.min()), max(r_raw.max(), r_cal.max()), 60)
    a3.hist(r_raw, bins=bins_h, color=RAW_C, alpha=0.6, label=f"raw |B|, sd {r_raw.std():.1f}")
    a3.hist(r_cal, bins=bins_h, color=CAL_C, alpha=0.8, label=f"calibrated |B|, sd {r_cal.std():.1f}")
    a3.axvline(R, color=TEXT_2, linewidth=0.8, linestyle="--")
    a3.set_xlabel("mG", fontsize=9, color=TEXT_2)
    a3.legend(loc="lower right", bbox_to_anchor=(1, 1), fontsize=8, frameon=False)

    pairs = [((0, 1), "XY plane"), ((1, 2), "YZ plane"), ((2, 0), "ZX plane")]
    for col, ((i, j), nm) in enumerate(pairs):
        a = fig.add_subplot(gs[1, col])
        lab = "XYZ"
        style(a, f"{nm}: raw (blue) vs calibrated (green)", f"{lab[j]} (mG)")
        a.scatter(raw[idx, i], raw[idx, j], s=1.5, color=RAW_C, alpha=0.5)
        a.scatter(cal[idx, i], cal[idx, j], s=1.5, color=CAL_C, alpha=0.6)
        th = np.linspace(0, 2 * np.pi, 200)
        a.plot(R * np.cos(th), R * np.sin(th), color=FIT_C, linewidth=0.8, linestyle="--")
        a.plot(centre[i], centre[j], marker="x", color="#e34948", markersize=8)
        a.axhline(0, color=TEXT_2, linewidth=0.5); a.axvline(0, color=TEXT_2, linewidth=0.5)
        a.set_aspect("equal", adjustable="datalim")
        a.set_xlabel(f"{lab[i]} (mG)", fontsize=9, color=TEXT_2)

    fig.suptitle(f"IIS2MDC ellipsoid fit   hard-iron ({centre[0]:+.0f}, {centre[1]:+.0f}, {centre[2]:+.0f}) mG,"
                 f"   |B| spread {100 * r_raw.std() / R:.1f} % → {100 * r_cal.std() / R:.2f} %",
                 x=0.01, ha="left", fontsize=11, color="#0b0b0b")
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    base = os.path.splitext(args.file)[0] + (f"_{args.tag}" if args.tag else "")
    fig.savefig(base + "_ellipsoid.png", dpi=140, facecolor=SURFACE)
    print(f"Saved {base}_ellipsoid.png")
    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
