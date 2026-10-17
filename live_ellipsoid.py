"""
Live 3D window of the RAW (uncalibrated) magnetometer data with a fitted ellipsoid.

Slowly turn the board through every direction. The points build up a shell.
Every few seconds an ellipsoid is fitted to all points so far and drawn as a
wireframe. Its centre (red x) is the hard-iron offset and its radii show the
soft-iron stretch. Nothing is corrected, the points are the raw sensor values.
Drag with the mouse to rotate the view.

  python live_ellipsoid.py --port COM5
  python live_ellipsoid.py --port COM5 --out ell1.csv --show 3000
Closing the window saves the data; run mag_ellipsoid.py on the CSV for the full fit report.
"""
import argparse
import csv
import math
import time

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import serial

from imu_common import AXIS_COLORS, SURFACE, TEXT_2, parse_mag_line
from mag_ellipsoid import fit_ellipsoid, soft_iron, ellipsoid_mesh, style3d

RAW_C = AXIS_COLORS["x"]
NEW_C = "#0b0b0b"
FIT_C = "#52514e"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True)
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", default=None)
    p.add_argument("--show", type=int, default=3000, help="max points drawn (fit uses all)")
    p.add_argument("--refit", type=float, default=2.0, help="seconds between ellipsoid fits")
    args = p.parse_args()

    out = args.out or time.strftime("ell_%Y%m%d_%H%M%S.csv")
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    f = open(out, "w", newline="")
    w = csv.writer(f)
    w.writerow(["t_s", "mx_mG", "my_mG", "mz_mG"])

    pts = []
    state = {"t0": None, "buf": b"", "last_fit": 0.0, "wire": None, "lim": 300.0, "centre": None}

    fig = plt.figure(figsize=(10, 9), facecolor=SURFACE)
    ax = fig.add_subplot(111, projection="3d")
    style3d(ax, "Raw magnetometer data (uncalibrated) + fitted ellipsoid")
    sc = ax.scatter([], [], [], s=3, color=RAW_C, alpha=0.55, depthshade=False)
    head = ax.scatter([], [], [], s=60, color=NEW_C, depthshade=False)
    cmark = ax.scatter([], [], [], s=80, color="#e34948", marker="x", depthshade=False)
    ax.set_xlim(-300, 300); ax.set_ylim(-300, 300); ax.set_zlim(-300, 300); ax.set_box_aspect((1, 1, 1))
    info = fig.text(0.01, 0.01, "waiting for data...", ha="left", va="bottom", fontsize=10,
                    color=TEXT_2, family="monospace")
    fig.tight_layout(rect=(0, 0.1, 1, 1))

    def update(_):
        st = state
        st["buf"] += ser.read(ser.in_waiting or 1)
        while b"\n" in st["buf"]:
            raw, st["buf"] = st["buf"].split(b"\n", 1)
            line = raw.decode("ascii", errors="ignore").strip()
            v = parse_mag_line(line)
            if v is None:
                if line:
                    print(">>", line)
                continue
            now = time.time(); st["t0"] = st["t0"] or now
            w.writerow([f"{now - st['t0']:.3f}", *v])
            pts.append(v)
        f.flush()
        if not pts:
            return []

        P = np.asarray(pts)
        shown = P if len(P) <= args.show else P[np.linspace(0, len(P) - 1, args.show).astype(int)]
        sc._offsets3d = (shown[:, 0], shown[:, 1], shown[:, 2])
        head._offsets3d = ([P[-1, 0]], [P[-1, 1]], [P[-1, 2]])

        m = np.abs(P).max()
        if m > st["lim"] * 0.95:
            st["lim"] = m * 1.15
            L = st["lim"]
            ax.set_xlim(-L, L); ax.set_ylim(-L, L); ax.set_zlim(-L, L)

        # direction coverage around the current data centre
        c0 = st["centre"] if st["centre"] is not None else np.median(P, 0)
        u = P - c0
        u = u / np.maximum(np.linalg.norm(u, axis=1)[:, None], 1e-9)
        cov = len({tuple(b) for b in np.round(u * 1.4).astype(int) if np.any(b)}) / 26

        now = time.time()
        fit_txt = "fit: needs more directions"
        if len(P) >= 300 and cov >= 0.35 and now - st["last_fit"] > args.refit:
            st["last_fit"] = now
            try:
                B = np.linalg.norm(P, axis=1)
                ok = np.abs(B - np.median(B)) < 0.6 * np.median(B)      # skip magnet spikes
                centre, M, k = fit_ellipsoid(P[ok])
                W, R, radii, vecs = soft_iron(M, k)
                if st["wire"] is not None:
                    st["wire"].remove()
                ex, ey, ez = ellipsoid_mesh(centre, radii, vecs, n=24)
                st["wire"] = ax.plot_wireframe(ex, ey, ez, color=FIT_C, linewidth=0.5, alpha=0.6)
                cmark._offsets3d = ([centre[0]], [centre[1]], [centre[2]])
                st["centre"] = centre
                st["fit"] = (centre, radii)
            except Exception:
                pass
        if st.get("fit"):
            c, r = st["fit"]
            fit_txt = (f"centre (hard-iron) X {c[0]:+6.1f}  Y {c[1]:+6.1f}  Z {c[2]:+6.1f} mG\n"
                       f"radii {r[0]:.0f} / {r[1]:.0f} / {r[2]:.0f} mG   axis ratio {r.max() / r.min():.3f}")
        x, y, z = P[-1]
        info.set_text(f"now  X {x:+7.1f}  Y {y:+7.1f}  Z {z:+7.1f} mG   |B| {math.sqrt(x*x+y*y+z*z):6.1f}\n"
                      f"samples {len(P)}   direction coverage {cov * 100:3.0f} %\n{fit_txt}")
        return []

    anim = FuncAnimation(fig, update, interval=150, cache_frame_data=False)
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        f.close(); ser.close()
        print(f"Saved {len(pts)} samples to {out}\nFull report: python mag_ellipsoid.py {out}")


if __name__ == "__main__":
    main()
