"""
heading.py : calibrated attitude and compass heading from the sensor board, on the ground station.

Uno runs motioncal_stream (Raw: lines, ~50 Hz). Needs imu_full_cal.json from full_calibration.py.

  python heading.py --port COM5                       live window + CSV log
  python heading.py --port COM5 --no-plot             terminal only
  python heading.py --port COM5 --filter standard     plain Mahony instead of adaptive
  python heading.py --port COM5 --declination -0.5    true north instead of magnetic north
  python heading.py --replay flight_raw.txt --rate 50  offline: a saved log of Raw: lines (or a CSV
                                                       with ax..mz columns) -> heading CSV + plot

Each sample:
  1. calibrate   gyro  w = raw/16 - bias                       (deg/s -> rad/s)
                 accel a = (raw/8192 - offset) * gain          (g)
                 mag   m = MAP @ SOFT @ (raw - HARD)            (mG, in BMI270 axes)
  2. filter      Mahony (adaptive by default) on w, a, m  ->  quaternion, body -> world
                 world frame: X = magnetic north, Y = west, Z = up
  3. output      roll, pitch (deg) and HEADING = compass direction of the board's +X axis,
                 0..360 deg, clockwise from north (90 = east), like a normal compass.

Keep the board still for the first 3 s: the gyro bias is re-measured then (it moves with
temperature). The filter needs a few seconds to swing the heading round to north.

Output CSV heading_<time>.csv:
  t_s, roll_deg, pitch_deg, heading_deg, a_norm_g, B_norm_mG, accel_trust, mag_trust
"""
import argparse
import collections
import csv
import json
import math
import os
import sys
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


class Calibration:
    """applies imu_full_cal.json to one Raw: sample"""

    def __init__(self, path):
        if not os.path.exists(path):
            sys.exit(f"{path} not found. Run full_calibration.py first (with motioncal_stream on the Uno).")
        c = json.load(open(path))
        self.gbias = np.array(c["gyro_bias_dps"])
        self.aoff, self.again = np.array(c["accel_offset_g"]), np.array(c["accel_gain"])
        self.hard = np.array(c["mag_hard_iron_mG"])
        self.soft = np.array(c["mag_soft_iron"])
        self.map = np.array(c["mag_axis_map_matrix"])
        self.B, self.dip = float(c["mag_field_mG"]), float(c["dip_deg"])
        self.map_clear = bool(c.get("mag_axis_map_clear", False))
        self.created = c.get("created", "?")
        self.MS = self.map @ self.soft

    def apply(self, v):
        a = (np.array(v[0:3]) / A_LSB - self.aoff) * self.again
        w = (np.array(v[3:6]) / G_LSB - self.gbias) * D2R
        m = self.MS @ (np.array(v[6:9], float) - self.hard)
        return a, w, m


def heading_deg(q, declination=0.0):
    """compass heading of body +X: clockwise from north, 0..360 (world Y points west)"""
    q0, q1, q2, q3 = q
    xn = 1 - 2 * (q2 * q2 + q3 * q3)            # body X along world X (north)
    xw = 2 * (q1 * q2 + q0 * q3)                # body X along world Y (west)
    return (math.degrees(math.atan2(-xw, xn)) + declination) % 360


def tilt_start(a):
    """quaternion for the tilt the accel shows, heading 0 (so the filter does not start upside down)"""
    roll = math.atan2(a[1], a[2]); pitch = math.atan2(-a[0], math.hypot(a[1], a[2]))
    cr, sr, cp, sp = math.cos(roll / 2), math.sin(roll / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return [cr * cp, sr * cp, cr * sp, -sr * sp]


class Runner:
    def __init__(self, cal, kind, dt, declination):
        self.cal, self.kind, self.dt, self.decl = cal, kind, dt, declination
        if kind == "adaptive":
            self.f = AdaptiveMahony(mag_field=cal.B, dip_deg=cal.dip)
        else:
            self.f = Mahony(kp=1.0, ki=0.05)
        self.t = 0.0

    def step(self, v):
        a, w, m = self.cal.apply(v)
        if self.kind == "adaptive":
            self.f.update(*w, *a, self.dt, *m)
            ta, tm = self.f.wa, self.f.wm
        else:
            ok = abs(np.linalg.norm(a) - 1) < 0.15
            self.f.update(*w, *a, self.dt, *m, accel_ok=ok)
            ta, tm = float(ok), 1.0
        self.t += self.dt
        r, p, _ = self.f.euler_deg()
        h = heading_deg(self.f.q, self.decl)
        return dict(t=self.t, roll=r, pitch=p, heading=h, an=float(np.linalg.norm(a)),
                    bn=float(np.linalg.norm(m)), ta=ta, tm=tm)


def load_raw(path):
    """Raw: lines from a text log, or a CSV with ax..mz columns (adaptive_compare / physical_test logs)"""
    with open(path, encoding="utf-8", errors="ignore") as f:
        first = f.readline(); f.seek(0)
        if first.startswith("t_s") or first.startswith("ax"):
            return [[int(r[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")]
                    for r in csv.DictReader(f)]
        return [v for v in (parse(l.strip()) for l in f) if v]


def replay(args, cal):
    raw = load_raw(args.replay)
    if len(raw) < 50:
        sys.exit("fewer than 50 samples in " + args.replay)
    R0 = np.array(raw[:int(3 * args.rate)], float)
    if R0[:, 3:6].std(0).max() / G_LSB < 0.5:            # still at the start: re-measure gyro bias
        cal.gbias = R0[:, 3:6].mean(0) / G_LSB
    run = Runner(cal, args.filter, 1.0 / args.rate, args.declination)
    run.f.q = tilt_start(cal.apply(raw[0])[0])
    out = os.path.splitext(args.replay)[0] + "_heading.csv"
    S = [run.step(v) for v in raw]
    with open(out, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t_s", "roll_deg", "pitch_deg", "heading_deg", "a_norm_g", "B_norm_mG", "accel_trust", "mag_trust"])
        for s in S:
            w.writerow([f"{s['t']:.3f}", f"{s['roll']:.2f}", f"{s['pitch']:.2f}", f"{s['heading']:.2f}",
                        f"{s['an']:.4f}", f"{s['bn']:.1f}", f"{s['ta']:.3f}", f"{s['tm']:.3f}"])
    print(f"{len(S)} samples, {S[-1]['t']:.1f} s. Final heading {S[-1]['heading']:.1f} deg, "
          f"roll {S[-1]['roll']:+.1f}, pitch {S[-1]['pitch']:+.1f}. Saved {out}")
    if not args.no_plot:
        import matplotlib.pyplot as plt
        t = [s["t"] for s in S]
        fig, ax = plt.subplots(3, 1, figsize=(11, 7), sharex=True)
        for a, k, lab in zip(ax, ("heading", "roll", "pitch"), ("heading (deg, cw from N)", "roll (deg)", "pitch (deg)")):
            a.plot(t, [s[k] for s in S], lw=1); a.set_ylabel(lab); a.grid(alpha=0.3)
        ax[0].set_ylim(0, 360); ax[2].set_xlabel("time (s)")
        fig.tight_layout(); fig.savefig(os.path.splitext(out)[0] + ".png", dpi=120); plt.show()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", help="COM5 on Windows, /dev/ttyACM0 on Linux")
    ap.add_argument("--replay", help="saved log: text with Raw: lines, or CSV with ax..mz columns")
    ap.add_argument("--rate", type=float, default=50.0, help="sample rate of the --replay log, Hz")
    ap.add_argument("--cal", default="imu_full_cal.json")
    ap.add_argument("--filter", choices=["adaptive", "standard"], default="adaptive")
    ap.add_argument("--declination", type=float, default=0.0,
                    help="magnetic declination in deg (east positive) to get TRUE heading")
    ap.add_argument("--no-plot", action="store_true", help="terminal output only")
    ap.add_argument("--window", type=float, default=30.0, help="seconds shown in the plots")
    args = ap.parse_args()

    cal = Calibration(args.cal)
    if args.replay:
        replay(args, cal); return
    if not args.port:
        ap.error("give --port COM5 (live) or --replay file (offline)")
    print(f"Calibration from {args.cal} ({cal.created}): field {cal.B:.0f} mG, dip {cal.dip:.1f} deg, "
          f"mag map {'CLEAR' if cal.map_clear else 'UNCERTAIN (heading may be wrong, redo the tumble)'}")

    import serial
    ser = serial.Serial(args.port, 115200, timeout=0.05)
    print("Port open, Uno restarting ...")
    time.sleep(2.5)
    print("Keep the board STILL for 3 s (gyro bias, sample rate) ...")
    ser.reset_input_buffer()                       # drop anything that piled up meanwhile
    rows, t0 = [], time.time()
    while time.time() - t0 < 3.0:
        v = parse(ser.readline().decode("ascii", errors="ignore").strip())
        if v:
            rows.append(v)
    t_meas = time.time() - t0
    if len(rows) < 30:
        sys.exit("No 'Raw:' data. Upload motioncal_stream to the Uno first.")
    R0 = np.array(rows, float)
    rate = len(R0) / t_meas
    dt = 1.0 / rate
    moved = R0[:, 3:6].std(0).max() / G_LSB > 0.5
    if moved:
        print("   board moved, keeping the gyro bias from the calibration file")
    else:
        cal.gbias = R0[:, 3:6].mean(0) / G_LSB
    print(f"   {rate:.1f} Hz, gyro bias {cal.gbias.round(3)} deg/s, filter: {args.filter}, "
          f"heading reference: {'true' if args.declination else 'magnetic'} north")

    run = Runner(cal, args.filter, dt, args.declination)
    a0, _, _ = cal.apply(R0[-1])
    run.f.q = tilt_start(a0)
    for v in rows:
        run.step(v)

    out = time.strftime("heading_%Y%m%d_%H%M%S.csv")
    fh = open(out, "w", newline="")
    w = csv.writer(fh)
    w.writerow(["t_s", "roll_deg", "pitch_deg", "heading_deg", "a_norm_g", "B_norm_mG", "accel_trust", "mag_trust"])

    def log(s):
        w.writerow([f"{s['t']:.3f}", f"{s['roll']:.2f}", f"{s['pitch']:.2f}", f"{s['heading']:.2f}",
                    f"{s['an']:.4f}", f"{s['bn']:.1f}", f"{s['ta']:.3f}", f"{s['tm']:.3f}"])

    def warn(s):
        msg = []
        if abs(s["bn"] / cal.B - 1) > 0.15:
            msg.append("MAG DISTURBED")
        if abs(s["an"] - 1) > 0.15:
            msg.append("ACCELERATING")
        return "  ".join(msg)

    if args.no_plot:
        print(f"Logging to {out}. Ctrl+C to stop.\n")
        buf, last = b"", 0.0
        try:
            while True:
                buf += ser.read(ser.in_waiting or 1)
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    v = parse(line.decode("ascii", errors="ignore").strip())
                    if v:
                        s = run.step(v); log(s)
                        if time.time() - last > 0.2:
                            last = time.time()
                            sys.stdout.write(f"\r heading {s['heading']:6.1f} deg   roll {s['roll']:+7.1f}   "
                                             f"pitch {s['pitch']:+6.1f}   |B| {s['bn']:5.0f} mG   {warn(s):<30}")
                            sys.stdout.flush()
        except KeyboardInterrupt:
            pass
        finally:
            fh.close(); ser.close(); print(f"\nSaved {out}")
        return

    # ---------------- live window
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    T = collections.deque(); H = collections.deque(); Rr = collections.deque(); P = collections.deque()
    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(3, 2, width_ratios=[1, 1.7])
    axc = fig.add_subplot(gs[:, 0], projection="polar")
    axc.set_theta_zero_location("N"); axc.set_theta_direction(-1)       # compass: N up, clockwise
    axc.set_rticks([]); axc.set_ylim(0, 1)
    axc.set_xticks(np.radians([0, 45, 90, 135, 180, 225, 270, 315]))
    axc.set_xticklabels(["N", "NE", "E", "SE", "S", "SW", "W", "NW"])
    needle, = axc.plot([0, 0], [0, 0.95], color="#d1495b", linewidth=4)
    big = axc.set_title("", fontsize=22, fontweight="bold", pad=18)
    names = ["Heading (deg, clockwise from north)", "Roll (deg)", "Pitch (deg)"]
    cols = ["#2e86ab", "#3bb273", "#e0a030"]
    axs, lines = [], []
    for i in range(3):
        a = fig.add_subplot(gs[i, 1]); a.set_title(names[i], loc="left", fontsize=10); a.grid(alpha=0.3)
        lines.append(a.plot([], [], color=cols[i], linewidth=1.5)[0]); axs.append(a)
    axs[0].set_ylim(0, 360); axs[0].set_yticks([0, 90, 180, 270, 360])
    axs[1].set_ylim(-180, 180); axs[2].set_ylim(-90, 90); axs[2].set_xlabel("time (s)")
    status = fig.text(0.01, 0.01, "", fontsize=9, family="monospace")
    fig.suptitle(f"{args.filter} Mahony, calibrated 9-DOF   ({'true' if args.declination else 'magnetic'} north)   "
                 f"log: {out}", fontsize=10)
    fig.tight_layout(rect=(0, 0.03, 1, 0.96))
    st = {"buf": b"", "s": None}

    def update(_):
        st["buf"] += ser.read(ser.in_waiting or 1)
        while b"\n" in st["buf"]:
            line, st["buf"] = st["buf"].split(b"\n", 1)
            v = parse(line.decode("ascii", errors="ignore").strip())
            if not v:
                continue
            s = run.step(v); log(s); st["s"] = s
            T.append(s["t"]); H.append(s["heading"]); Rr.append(s["roll"]); P.append(s["pitch"])
            while T and T[0] < s["t"] - args.window:
                T.popleft(); H.popleft(); Rr.popleft(); P.popleft()
        fh.flush()
        s = st["s"]
        if s is None:
            return []
        needle.set_data([math.radians(s["heading"])] * 2, [0, 0.95])
        big.set_text(f"{s['heading']:.1f}°")
        for ln, d in zip(lines, (H, Rr, P)):
            ln.set_data(T, d)
        for a in axs:
            a.set_xlim(max(0, T[-1] - args.window), max(T[-1], 1))
        status.set_text(f"|a| {s['an']:.3f} g   |B| {s['bn']:.0f} mG (cal {cal.B:.0f})   "
                        f"accel trust {s['ta']:.2f}   mag trust {s['tm']:.2f}   {warn(s)}")
        return []

    anim = FuncAnimation(fig, update, interval=60, cache_frame_data=False)  # noqa: F841
    try:
        plt.show()
    finally:
        fh.close(); ser.close(); print(f"Saved {out}")


if __name__ == "__main__":
    main()
