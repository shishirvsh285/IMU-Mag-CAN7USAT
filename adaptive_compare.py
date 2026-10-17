"""
adaptive_compare.py : standard Mahony vs Adaptive Mahony on the REAL board, side by side.

Uno must run motioncal_stream (Raw: lines, ~50 Hz).

  python adaptive_compare.py --port COM5            live window + logs raw data to adapt_<time>.csv
  python adaptive_compare.py --replay adapt_x.csv   re-run both filters on a log, print the test score
  python adaptive_compare.py --replay adapt_x.csv --set kp_min=0.05 --set tau_c=0.5   try settings

Calibration: uses imu_full_cal.json (from full_calibration.py) if it is there: accel offset/gain,
gyro bias, mag hard/soft iron + axis map (then both filters are 9-DOF). Otherwise accel raw,
gyro bias from the first 3 s, 6-DOF. Keep the board STILL for the first 3 s either way.

HOW TO GET A GROUND TRUTH WITHOUT A MOTION RIG
  A still board is its own truth: the accel then measures pure gravity. So:
  1. "Shake test": hold the board at a fixed angle and move it back and forth / up and down
     hard WITHOUT rotating it (slide it on the table, or shake it in your hand keeping it level),
     then put it down and hold still 2 s. Repeat at a few angles.
     While it shakes, the true tilt is the one it has when it stops (gyro confirms it did not
     rotate), so the error DURING the shaking is known.
  2. "Pendulum test": hang it on a string ~0.5-1 m and let it swing and spin, then stop it and
     hold still. The error at the moment it stops is known.
  --replay prints, for every still period that follows motion:
     tilt error of each filter at the moment the motion stopped, and (if the board did not
     rotate during the motion) the max tilt error during the motion.
"""
import argparse
import collections
import csv
import json
import math
import os
import time

import numpy as np

from adaptive_mahony import AdaptiveMahony
from mahony import Mahony

A_LSB, G_LSB = 8192.0, 16.0
D2R = math.pi / 180


def parse(line):
    if not line.startswith("Raw:"):
        return None
    try:
        v = [int(x) for x in line[4:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 9 else None


class Cal:
    def __init__(self, path="imu_full_cal.json"):
        self.ok = os.path.exists(path)
        self.has_mag = False
        if self.ok:
            c = json.load(open(path))
            self.aoff, self.again = np.array(c["accel_offset_g"]), np.array(c["accel_gain"])
            self.gbias = np.array(c["gyro_bias_dps"])
            self.mb, self.mW = np.array(c["mag_hard_iron_mG"]), np.array(c["mag_soft_iron"])
            self.mS = np.array(c["mag_axis_map_matrix"])
            self.B, self.dip = c["mag_field_mG"], c["dip_deg"]
            self.has_mag = bool(c.get("mag_axis_map_clear", False))
        else:
            self.aoff, self.again, self.gbias = np.zeros(3), np.ones(3), None
            self.B = self.dip = None

    def apply(self, v):
        a = (np.array(v[0:3]) / A_LSB - self.aoff) * self.again
        g = (np.array(v[3:6]) / G_LSB - self.gbias) * D2R
        m = self.mS @ (self.mW @ (np.array(v[6:9], float) - self.mb)) if self.has_mag else None
        return a, g, m


def make_filters(cal, overrides):
    std = Mahony(1.0, 0.05)
    kw = dict(mag_field=cal.B, dip_deg=cal.dip) if cal.has_mag else {}
    kw.update(overrides)
    return std, AdaptiveMahony(**kw)


def step(std, adp, a, g, m, dt):
    ok = abs(np.linalg.norm(a) - 1) < 0.15
    if m is not None:
        std.update(*g, *a, dt, *m, accel_ok=ok)
        adp.update(*g, *a, dt, *m)
    else:
        std.update(*g, *a, dt, accel_ok=ok)
        adp.update(*g, *a, dt)


def up_of(f):
    q0, q1, q2, q3 = f.q
    return np.array([2 * (q1 * q3 - q0 * q2), 2 * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3])


# ------------------------------------------------------------------ replay + score
def replay(path, overrides, plot=True):
    rows = list(csv.DictReader(open(path)))
    t = np.array([float(r["t_s"]) for r in rows])
    raw = np.array([[int(r[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")] for r in rows])
    cal = Cal()
    n0 = int(np.searchsorted(t, 3.0))
    if cal.gbias is None:
        cal.gbias = raw[:n0, 3:6].mean(0) / G_LSB
    dt = np.median(np.diff(t))
    std, adp = make_filters(cal, overrides)
    N = len(t)
    A = np.zeros((N, 3)); Gm = np.zeros((N, 3)); UPs = np.zeros((N, 3)); UPa = np.zeros((N, 3)); KP = np.zeros(N)
    for k in range(N):
        a, g, m = cal.apply(raw[k])
        step(std, adp, a, g, m, dt)
        A[k], Gm[k], UPs[k], UPa[k], KP[k] = a, g, up_of(std), up_of(adp), adp.kp

    # still detection
    wdeg = np.degrees(np.linalg.norm(Gm, axis=1))
    moving = (wdeg > 8) | (np.abs(np.linalg.norm(A, axis=1) - 1) > 0.04)
    k = int(0.3 / dt)
    moving = np.convolve(moving.astype(float), np.ones(k), "same") > 0
    edges = np.flatnonzero(np.diff(moving.astype(int)))
    segs, start, mv = [], 0, moving[0]
    for e in list(edges) + [N - 1]:
        segs.append((start, e + 1, mv)); start = e + 1; mv = not mv
    ang = lambda u, v: np.degrees(np.arccos(np.clip(np.sum(u * v, -1) /
                                                    (np.linalg.norm(u, axis=-1) * np.linalg.norm(v, axis=-1)), -1, 1)))
    print(f"\n{path}: {N} samples, {t[-1]:.0f} s, {'9-DOF' if cal.has_mag else '6-DOF'}, "
          f"{'imu_full_cal.json' if cal.ok else 'no calibration file'}")
    print(f"{'motion':>15} {'rot':>6} {'peak|a|':>8} |{'at stop: std':>13} {'adaptive':>9} |"
          f"{'during: std':>12} {'adaptive':>9}")
    res = []
    for i in range(1, len(segs)):
        s0, s1, mv = segs[i]
        p0, p1, pmv = segs[i - 1]
        if mv or not pmv or (s1 - s0) * dt < 1.0 or (p1 - p0) * dt < 1.5:
            continue
        ref = A[s0 + int(0.3 / dt): s1].mean(0)           # gravity while still
        ks = s0
        e_s, e_a = ang(UPs[ks], ref), ang(UPa[ks], ref)
        # did it rotate during the motion? integrate gyro
        rot = np.degrees(np.linalg.norm(np.sum(Gm[p0:p1], 0) * dt))
        rot_max = np.degrees(np.max(np.linalg.norm(np.cumsum(Gm[p0:p1], 0) * dt, axis=1)))
        dur_s = dur_a = float("nan")
        if rot_max < 5:                                   # translation only: truth = ref
            dur_s, dur_a = ang(UPs[p0:p1], ref).max(), ang(UPa[p0:p1], ref).max()
        peak = np.abs(np.linalg.norm(A[p0:p1], axis=1) - 1).max() + 1
        res.append((e_s, e_a, dur_s, dur_a))
        print(f"{t[p0]:6.1f}-{t[p1]:6.1f}s {rot_max:6.1f} {peak:7.2f}g |{e_s:12.2f}° {e_a:8.2f}° |"
              f"{dur_s:11.2f}° {dur_a:8.2f}°")
    if res:
        r = np.array(res)
        print(f"{'mean':>31} |{np.nanmean(r[:, 0]):12.2f}° {np.nanmean(r[:, 1]):8.2f}° |"
              f"{np.nanmean(r[:, 2]):11.2f}° {np.nanmean(r[:, 3]):8.2f}°")
    else:
        print("No still periods after motion found. Do the shake / pendulum test (see top of file).")

    if plot:
        import matplotlib.pyplot as plt
        tilt = lambda u: np.degrees(np.arccos(np.clip(u[:, 2] / np.linalg.norm(u, axis=1), -1, 1)))
        fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
        ax[0].plot(t, tilt(A), color="#bbb", lw=0.6, label="accel tilt (truth only when still)")
        ax[0].plot(t, tilt(UPs), color="#d1495b", lw=1, label="standard")
        ax[0].plot(t, tilt(UPa), color="#2e86ab", lw=1, label="adaptive")
        ax[0].set_ylabel("tilt from level, deg"); ax[0].legend(fontsize=8)
        ax[1].plot(t, np.linalg.norm(A, axis=1), color="#e0a030", lw=0.6); ax[1].set_ylabel("|a|, g")
        ax[1].fill_between(t, 0, 3, where=moving, color="#eee", step="mid")
        ax[1].set_ylim(0, 3)
        ax[2].semilogy(t, KP, color="#2e86ab", lw=0.8, label="adaptive kp")
        ax[2].axhline(1, color="#d1495b", lw=0.8, label="standard kp"); ax[2].legend(fontsize=8)
        ax[2].set_xlabel("time, s")
        fig.tight_layout()
        out = os.path.splitext(path)[0] + "_compare.png"
        fig.savefig(out, dpi=120); print("saved", out)
        plt.show()


# ------------------------------------------------------------------ live
def live(port, overrides, window):
    import serial
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    ser = serial.Serial(port, 115200, timeout=0.05)
    print("Port open, Uno restarting ..."); time.sleep(2.5); ser.reset_input_buffer()
    cal = Cal()
    print("Keep the board STILL for 3 s ...")
    rows, t0 = [], time.time()
    while time.time() - t0 < 3.0:
        v = parse(ser.readline().decode("ascii", errors="ignore").strip())
        if v:
            rows.append(v)
    if len(rows) < 30:
        print("No 'Raw:' data. Upload motioncal_stream first."); return
    R0 = np.array(rows, float)
    if cal.gbias is None:
        cal.gbias = R0[:, 3:6].mean(0) / G_LSB
    dt = 3.0 / len(R0)
    std, adp = make_filters(cal, overrides)
    out = time.strftime("adapt_%Y%m%d_%H%M%S.csv")
    fh = open(out, "w", newline=""); w = csv.writer(fh)
    w.writerow(["t_s", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
                "std_roll", "std_pitch", "std_yaw", "adp_roll", "adp_pitch", "adp_yaw", "adp_kp", "adp_wa"])
    st = {"t": 0.0, "buf": b""}
    for v in rows:                                       # still samples go in the log too
        a, g, m = cal.apply(v); step(std, adp, a, g, m, dt); st["t"] += dt
        w.writerow([f"{st['t']:.3f}", *v, *[f"{x:.2f}" for x in std.euler_deg() + adp.euler_deg()],
                    f"{adp.kp:.4f}", f"{adp.wa:.3f}"])
    T = collections.deque(); S = [collections.deque() for _ in range(5)]
    fig, ax = plt.subplots(3, 1, figsize=(11, 8), sharex=True)
    names = ["roll, deg", "pitch, deg"]
    Ls = [ax[i].plot([], [], color="#d1495b", lw=1.2, label="standard")[0] for i in range(2)]
    La = [ax[i].plot([], [], color="#2e86ab", lw=1.2, label="adaptive")[0] for i in range(2)]
    for i in range(2):
        ax[i].set_ylabel(names[i]); ax[i].legend(fontsize=8, loc="upper left"); ax[i].grid(alpha=0.3)
    ax[0].set_ylim(-180, 180); ax[1].set_ylim(-90, 90)
    Lk, = ax[2].semilogy([], [], color="#2e86ab", lw=1, label="adaptive kp")
    ax[2].axhline(1, color="#d1495b", lw=0.8, label="standard kp"); ax[2].set_ylim(0.01, 10)
    ax[2].legend(fontsize=8, loc="upper left"); ax[2].set_xlabel("time, s")
    fig.suptitle(f"standard vs adaptive Mahony  ({'9-DOF' if cal.has_mag else '6-DOF'})   logging {out}")

    def upd(_):
        st["buf"] += ser.read(ser.in_waiting or 1)
        while b"\n" in st["buf"]:
            line, st["buf"] = st["buf"].split(b"\n", 1)
            v = parse(line.decode("ascii", errors="ignore").strip())
            if not v:
                continue
            a, g, m = cal.apply(v)
            step(std, adp, a, g, m, dt); st["t"] += dt
            es, ea = std.euler_deg(), adp.euler_deg()
            w.writerow([f"{st['t']:.3f}", *v, *[f"{x:.2f}" for x in es + ea], f"{adp.kp:.4f}", f"{adp.wa:.3f}"])
            T.append(st["t"]); S[0].append(es[0]); S[1].append(es[1]); S[2].append(ea[0]); S[3].append(ea[1])
            S[4].append(max(adp.kp, 0.011))
            while T and T[0] < st["t"] - window:
                T.popleft(); [s.popleft() for s in S]
        fh.flush()
        if T:
            Ls[0].set_data(T, S[0]); Ls[1].set_data(T, S[1]); La[0].set_data(T, S[2]); La[1].set_data(T, S[3])
            Lk.set_data(T, S[4])
            ax[2].set_xlim(max(0, T[-1] - window), max(T[-1], 1))
        return []

    anim = FuncAnimation(fig, upd, interval=60, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        fh.close(); ser.close(); print("Saved", out, "\nNow run:  python adaptive_compare.py --replay", out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--replay")
    ap.add_argument("--set", action="append", default=[], help="adaptive setting, e.g. kp_min=0.05")
    ap.add_argument("--window", type=float, default=30.0)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    ov = {k: float(v) for k, v in (s.split("=") for s in args.set)}
    if args.replay:
        replay(args.replay, ov, not args.no_plot)
    elif args.port:
        live(args.port, ov, args.window)
    else:
        ap.error("give --port COM5 or --replay file.csv")


if __name__ == "__main__":
    main()
