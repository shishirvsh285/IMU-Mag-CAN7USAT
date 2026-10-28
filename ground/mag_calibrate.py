"""
mag_calibrate.py : magnetometer-only calibration (hard iron + soft iron) for the IIS2MDC.

Uno must run motioncal_stream (raw data: Raw:ax,ay,az,gx,gy,gz,mx,my,mz, mag in mG).

  python mag_calibrate.py --port COM5               60 s tumble (Uno or ESP32, any COM port)
  python mag_calibrate.py --port COM5 --time 90     longer tumble
  python mag_calibrate.py --port COM5 --no-plot     no plot window at the end

What you do: when it says GO, turn the board slowly through EVERY direction (flip it, roll it, stand it on
each edge, point it along diagonals). Keep phones, magnets, laptops and steel away. The coverage number
shows how much of the sphere of directions you have visited. Aim for 80 % or more.

What it fits: the raw readings of a perfect sensor lie on a sphere of radius |B| (the local field).
Real ones lie on a shifted, squashed ellipsoid:
  hard iron  b : constant offset from magnetised parts near the sensor (shifts the centre)
  soft iron  W : iron and the sensor's own axis gains (squash and tilt the shape)
  calibrated   m = W (raw - b)        (mG, still in the IIS2MDC's own axes)

Outputs (next to this script):
  mag_cal.json                       numbers, quality, date
  mag_cal_report.txt                 readable summary
  mag_cal_check/mag_cal.h            constants for the mag_cal_check sketch (upload it, open MotionCal)
  esp32/mag_cal_check_esp32/mag_cal.h  the same, for the ESP32 version
  mag_cal_<time>.csv                 the raw tumble data, so the fit can be redone later
  mag_cal_<time>.png                 raw vs calibrated point clouds (3 projections)
"""
import argparse
import json
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))


def parse(line):
    if not line.startswith("Raw:"):
        return None
    try:
        v = [int(x) for x in line[4:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 9 else None


def fit_ellipsoid(v):
    """General ellipsoid -> hard-iron centre b and soft-iron W, so that W @ (m - b) lies on a sphere."""
    x, y, z = v.T
    D = np.column_stack([x * x, y * y, z * z, 2 * x * y, 2 * x * z, 2 * y * z, 2 * x, 2 * y, 2 * z])
    p, *_ = np.linalg.lstsq(D, np.ones(len(v)), rcond=None)
    M = np.array([[p[0], p[3], p[4]], [p[3], p[1], p[5]], [p[4], p[5], p[2]]])
    b = -np.linalg.solve(M, p[6:9])
    k = 1 + b @ M @ b
    vals, vecs = np.linalg.eigh(M / k)
    if np.any(vals <= 0):
        raise ValueError("not an ellipsoid: the tumble did not cover enough directions")
    radii = 1 / np.sqrt(vals)
    R = radii.mean()
    W = vecs @ np.diag(np.sqrt(vals)) @ vecs.T * R          # keeps the field size in mG
    return b, W, R, radii


def robust_fit(m):
    """fit, then drop points far off the ellipsoid (magnet passes, glitches) and refit twice"""
    B = np.linalg.norm(m, axis=1)
    keep = np.abs(B - np.median(B)) < 0.6 * np.median(B)
    b, W, R, radii = fit_ellipsoid(m[keep])
    for _ in range(2):
        r = np.linalg.norm((m - b) @ W.T, axis=1) / R
        keep = np.abs(r - 1) < 0.15
        b, W, R, radii = fit_ellipsoid(m[keep])
    return b, W, R, radii, keep


def coverage(u):
    """share of 26 direction cells (cube faces, edges, corners) visited"""
    n = np.linalg.norm(u, axis=1)
    u = u[n > 1e-9] / n[n > 1e-9, None]
    return len({tuple(c) for c in np.round(u * 1.4).astype(int) if np.any(c)}) / 26


def write_outputs(m, b, W, R, radii, keep, stamp, plot):
    mc = (m - b) @ W.T
    Braw, Bcal = np.linalg.norm(m[keep], axis=1), np.linalg.norm(mc[keep], axis=1)
    cov = coverage(mc[keep])
    spread_raw, spread_cal = Braw.std() / R * 100, Bcal.std() / R * 100
    ok = cov >= 0.8 and spread_cal < 2.0
    lines = [
        "=" * 60,
        f"MAGNETOMETER CALIBRATION   {stamp}",
        "=" * 60,
        f"samples used        {keep.sum()} of {len(keep)}  (rest were off the ellipsoid: magnets, glitches)",
        f"direction coverage  {cov * 100:.0f} %   (want 80 % or more)",
        f"hard iron b (mG)    X {b[0]:+8.1f}   Y {b[1]:+8.1f}   Z {b[2]:+8.1f}",
        "soft iron W",
        *["                    " + "  ".join(f"{x:+.5f}" for x in row) for row in W],
        f"field |B|           {R:.1f} mG  ({R / 10:.2f} uT)",
        f"ellipsoid radii     {radii.round(1)} mG   (axis ratio {radii.max() / radii.min():.3f})",
        f"|B| spread          raw {spread_raw:.1f} %  ->  calibrated {spread_cal:.2f} %   (want under 2 %)",
        f"result              {'GOOD' if ok else 'CHECK: redo the tumble with more variety, away from metal'}",
        "=" * 60,
    ]
    report = "\n".join(lines)
    print("\n" + report)

    cal = {"created": stamp, "units": "mG, IIS2MDC axes", "formula": "m_cal = W @ (m_raw - b)",
           "hard_iron_mG": b.round(3).tolist(), "soft_iron_W": W.round(6).tolist(),
           "field_mG": round(float(R), 2), "radii_mG": radii.round(2).tolist(),
           "coverage": round(cov, 3), "B_spread_pct_raw": round(float(spread_raw), 3),
           "B_spread_pct_cal": round(float(spread_cal), 3), "samples_used": int(keep.sum()), "good": bool(ok)}
    with open(os.path.join(HERE, "mag_cal.json"), "w") as f:
        json.dump(cal, f, indent=2)
    with open(os.path.join(HERE, "mag_cal_report.txt"), "w") as f:
        f.write(report + "\n")

    hdirs = [os.path.join(HERE, "mag_cal_check")]
    esp = os.path.join(HERE, "esp32", "mag_cal_check_esp32")
    if os.path.isdir(esp):
        hdirs.append(esp)
    fmt = lambda v: ", ".join(f"{x:.6f}f" for x in v)
    h = ["// Written by mag_calibrate.py " + stamp,
         "// m_cal = MAG_SOFT * (m_raw - MAG_HARD), mG, IIS2MDC axes",
         "#pragma once",
         f"const float MAG_HARD[3]    = {{{fmt(b)}}};",
         "const float MAG_SOFT[3][3] = {" + ", ".join("{" + fmt(r) + "}" for r in W) + "};",
         f"const float MAG_FIELD_MG   = {R:.1f}f;",
         f"#define MAG_CAL_STAMP \"{stamp}\""]
    for hdir in hdirs:
        os.makedirs(hdir, exist_ok=True)
        with open(os.path.join(hdir, "mag_cal.h"), "w") as f:
            f.write("\n".join(h) + "\n")
    print("Saved mag_cal.json, mag_cal_report.txt and mag_cal.h in:",
          ", ".join(os.path.relpath(d, HERE) for d in hdirs))

    if plot is not None:
        import matplotlib
        if plot == "file":
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 3, figsize=(13, 8.5))
        pairs = [(0, 1, "X", "Y"), (1, 2, "Y", "Z"), (2, 0, "Z", "X")]
        lim = np.abs(m).max() * 1.1
        th = np.linspace(0, 2 * np.pi, 200)
        for row, (data, name) in enumerate([(m, "raw"), (mc, "calibrated")]):
            for col, (i, j, a, c) in enumerate(pairs):
                A = ax[row, col]
                A.scatter(data[~keep, i], data[~keep, j], s=2, color="#ccc")
                A.scatter(data[keep, i], data[keep, j], s=2, color="#2e86ab" if row else "#d1495b")
                if row:
                    A.plot(R * np.cos(th), R * np.sin(th), color="#333", lw=0.8)
                A.axhline(0, color="#999", lw=0.5); A.axvline(0, color="#999", lw=0.5)
                A.set_xlim(-lim, lim); A.set_ylim(-lim, lim); A.set_aspect("equal")
                A.set_title(f"{name}: {a}-{c}", fontsize=10); A.set_xlabel(f"{a} (mG)"); A.set_ylabel(f"{c} (mG)")
        fig.suptitle(f"Raw (top) should be an off-centre ellipse. Calibrated (bottom) should sit on the circle of "
                     f"radius {R:.0f} mG.   coverage {cov * 100:.0f} %, spread {spread_cal:.2f} %", fontsize=10)
        fig.tight_layout()
        png = os.path.join(HERE, f"mag_cal_{stamp.replace(':', '').replace(' ', '_')}.png")
        fig.savefig(png, dpi=110)
        print("Saved", os.path.basename(png))
        if plot == "show":
            plt.show()
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM5 on Windows")
    ap.add_argument("--time", type=float, default=60.0, help="tumble time in seconds")
    ap.add_argument("--no-plot", action="store_true")
    ap.add_argument("--refit", help="redo the fit from a saved mag_cal_<time>.csv, no board needed")
    args = ap.parse_args()
    plot = None if args.no_plot else "show"

    if args.refit:
        m = np.loadtxt(args.refit, delimiter=",", skiprows=1, usecols=(1, 2, 3))
        b, W, R, radii, keep = robust_fit(m)
        write_outputs(m, b, W, R, radii, keep, time.strftime("%Y-%m-%d %H:%M:%S") + " (refit)", plot)
        return
    if not args.port:
        ap.error("give --port COM5 (or --refit file.csv)")

    import serial
    ser = serial.Serial(args.port, 115200, timeout=0.2)
    print("Port open, Uno restarting ...")
    time.sleep(3)
    ser.reset_input_buffer()                  # Windows can replay stale data on open
    t0, n = time.time(), 0
    while time.time() - t0 < 1.0:
        if parse(ser.readline().decode("ascii", "ignore").strip()):
            n += 1
    if n < 20:
        sys.exit("No 'Raw:' data. Upload motioncal_stream (raw data) to the Uno first.")
    print(f"Stream OK ({n} samples/s).\n")
    print("Turn the board slowly through EVERY direction: flip it, roll it, stand it on each edge, point it")
    print("along diagonals. Keep phones, magnets, laptops and steel away.")
    input(f"Press Enter, then start turning ({args.time:.0f} s) ...")

    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    csv_path = os.path.join(HERE, f"mag_cal_{stamp.replace(':', '').replace(' ', '_')}.csv")
    rows = []
    ser.reset_input_buffer()
    t0 = last = time.time()
    while time.time() - t0 < args.time:
        v = parse(ser.readline().decode("ascii", "ignore").strip())
        if v:
            rows.append([time.time() - t0, *v[6:9]])
        if time.time() - last > 0.5 and len(rows) > 20:
            last = time.time()
            m = np.array(rows)[:, 1:]
            cov = coverage(m - np.median(m, 0))
            bar = "#" * int(cov * 30)
            sys.stdout.write(f"\r  {time.time() - t0:4.0f} s   samples {len(rows):5d}   coverage [{bar:<30}] {cov * 100:3.0f} %  ")
            sys.stdout.flush()
    ser.close()
    print("\nDone turning.")
    m = np.array(rows, float)
    np.savetxt(csv_path, m, delimiter=",", header="t_s,mx_mG,my_mG,mz_mG", comments="", fmt="%.3f")
    print("Saved raw data", os.path.basename(csv_path))
    try:
        b, W, R, radii, keep = robust_fit(m[:, 1:])
    except (ValueError, np.linalg.LinAlgError) as e:
        sys.exit(f"Fit failed: {e}. Turn the board through more directions and run again.")
    write_outputs(m[:, 1:], b, W, R, radii, keep, stamp, plot)
    print("\nNext: upload mag_cal_check to the Uno, open MotionCal, check the sphere.")


if __name__ == "__main__":
    main()
