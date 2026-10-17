"""
physical_test.py : guided hands-on test of standard Mahony vs Adaptive Mahony on the real board.

Uno must run motioncal_stream (Raw: lines, ~50 Hz).

  python physical_test.py --port COM5               all tests
  python physical_test.py --port COM5 --only shake  one test (shake, pendulum, spin, tumble, magnet)
  python physical_test.py --port COM5 --set kp_min=0.05   try other adaptive settings
  python physical_test.py --replay phystest_x.csv [--set tau_avg=5]   re-score a recording,
                                                   e.g. with other settings (no board needed)

The script tells you what to do, counts down, and records. Both filters run on the same data.
If imu_full_cal.json (from full_calibration.py) is in the folder, its calibration is used and
the filters are 9-DOF. Without it: 6-DOF, gyro bias from the first still period, magnet test
skipped.

WHERE THE TRUTH COMES FROM
  A still board is its own truth: the accel then reads only gravity. Every test ends with the
  board held still. From that known attitude the gyro is run BACKWARDS through the motion.
  Over a few seconds a calibrated gyro is accurate to about 0.1 deg/s, and running it backwards
  never uses the accel, so it is a fair truth for both filters. The script prints how good
  that truth is ("truth +-": 0.1 deg/s x time + 1 % of the tilt excursion + 0.1 % of all the
  rotation) and skips the "during" score when it is worse than 5 deg (long fast spins).
  In the magnet test the board is still, so the true heading must not change.

You need: a string about 0.5 to 1 m long (for pendulum and spin), a table, a phone or magnet
(magnet test only). Tape the board flat to something stiff if the wires flop around.

Outputs: phystest_<time>.csv (raw + both filters, readable by adaptive_compare.py --replay),
         phystest_<time>_results.txt, phystest_<time>.png
"""
import argparse
import csv
import math
import sys
import threading
import time

import numpy as np

from adaptive_compare import A_LSB, G_LSB, Cal, make_filters, parse, step, up_of

TESTS = {
    "shake": dict(
        title="SHAKE TEST (no rotation)",
        reps=3,
        setup=["Hold the board at a fixed angle. Rep 1: flat. Rep 2: tilted about 30 deg.",
               "Rep 3: tilted about 60 deg the other way. Prop your elbow on the table."],
        hold="Hold it still at this angle",
        move="SHAKE it hard: back-forth, side to side, up-down. Do NOT rotate it",
        move_s=8),
    "pendulum": dict(
        title="PENDULUM TEST",
        reps=2,
        setup=["Hang the board from the string (tie it so the board hangs flat).",
               "Hold the string end steady in your hand or tie it to something."],
        hold="Let it hang and hold it still with your other hand",
        move="Pull it 30-40 deg to the side and LET GO. Let it swing. Push it again if it dies down",
        move_s=20),
    "spin": dict(
        title="SPIN + SWING TEST (parachute-like)",
        reps=2,
        setup=["Same string. Twist the board 10-20 turns so it will spin when you let go."],
        hold="Hold the twisted board still",
        move="LET GO: it spins. Give the string a few jerks sideways so it also swings",
        move_s=20),
    "tumble": dict(
        title="TUMBLE TEST (deployment-like)",
        reps=2,
        setup=["Hold the board in your hand."],
        hold="Hold it still",
        move="Tumble it fast in every direction (flip, twist, shake). Then put it down flat",
        move_s=6),
    "magnet": dict(
        title="MAGNET TEST (heading)",
        reps=1,
        setup=["Put the board flat on the table. It stays still the whole test."],
        hold="Do not touch the board",
        move="Bring the phone / magnet to about 5 cm from the board, move it around slowly",
        move_s=8),
}
HOLD_S, END_S = 4.0, 5.0


class Recorder:
    """reads the serial port in a thread, runs both filters, stores everything"""

    def __init__(self, ser, cal, overrides, dt, fh):
        self.ser, self.cal, self.dt = ser, cal, dt
        self.std, self.adp = make_filters(cal, overrides)
        self.lock = threading.Lock()
        self.phase = "start"
        self.rows = []                 # (t, phase, a, g, m, up_std, up_adp, yaw_std, yaw_adp, kp)
        self.t = 0.0
        self.run = True
        self.w = csv.writer(fh)
        self.fh = fh
        self.w.writerow(["t_s", "ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz",
                         "std_roll", "std_pitch", "std_yaw", "adp_roll", "adp_pitch", "adp_yaw",
                         "adp_kp", "adp_wa", "phase"])
        self.th = threading.Thread(target=self.loop, daemon=True)

    def feed(self, v):
        a, g, m = self.cal.apply(v)
        step(self.std, self.adp, a, g, m, self.dt)
        self.t += self.dt
        es, ea = self.std.euler_deg(), self.adp.euler_deg()
        with self.lock:
            self.rows.append((self.t, self.phase, a, g, m, up_of(self.std), up_of(self.adp),
                              es[2], ea[2], self.adp.kp))
            ph = self.phase
        self.w.writerow([f"{self.t:.3f}", *v, *[f"{x:.2f}" for x in es + ea],
                         f"{self.adp.kp:.4f}", f"{self.adp.wa:.3f}", ph])

    def loop(self):
        buf = b""
        while self.run:
            buf += self.ser.read(self.ser.in_waiting or 1)
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                v = parse(line.decode("ascii", errors="ignore").strip())
                if v:
                    self.feed(v)
        self.fh.flush()

    def last(self):
        with self.lock:
            return self.rows[-1] if self.rows else None


def tilt_deg(u):
    u = np.asarray(u)
    return np.degrees(np.arccos(np.clip(u[..., 2] / np.linalg.norm(u, axis=-1), -1, 1)))


def ang(u, v):
    u, v = np.asarray(u), np.asarray(v)
    c = np.sum(u * v, -1) / (np.linalg.norm(u, axis=-1) * np.linalg.norm(v, axis=-1))
    return np.degrees(np.arccos(np.clip(c, -1, 1)))


def countdown(rec, seconds, label, wait=input):
    t_end = time.time() + seconds
    while time.time() < t_end:
        r = rec.last()
        if r:
            an = np.linalg.norm(r[2])
            sys.stdout.write(f"\r   {label:<9} {t_end - time.time():4.1f} s   |a| {an:4.2f} g   "
                             f"tilt: accel {tilt_deg(r[2]):5.1f}  standard {tilt_deg(r[5]):5.1f}  "
                             f"adaptive {tilt_deg(r[6]):5.1f}   kp {r[9]:5.3f}   ")
            sys.stdout.flush()
        time.sleep(0.2)
    print()


def set_phase(rec, name):
    with rec.lock:
        rec.phase = name


def run_test(rec, key, rep, ask, beep):
    T = TESTS[key]
    tag = f"{key}{rep + 1}"
    print(f"\n{T['title']}  rep {rep + 1}/{T['reps']}")
    if rep == 0:
        for s in T["setup"]:
            print("   " + s)
    ask(f"   {T['hold']}. Press Enter when ready ...")
    set_phase(rec, tag + ":hold"); countdown(rec, HOLD_S, "HOLD")
    print(beep + "   NOW: " + T["move"])
    set_phase(rec, tag + ":move"); countdown(rec, T["move_s"], "MOVE")
    print(beep + ("   STOP. Put it down / stop it and hold it STILL" if key != "magnet"
                  else "   Take the magnet away"))
    set_phase(rec, tag + ":settle"); countdown(rec, 1.5, "SETTLE")
    set_phase(rec, tag + ":end"); countdown(rec, END_S, "STILL")
    set_phase(rec, "idle")


def truth_back(G, ref, k0, k1, dt):
    """true 'up' (body frame) for samples k0..k1, by running the gyro BACKWARDS from the still
    attitude at k1 (u' = -w x u). Short gyro runs are very accurate (bias error ~0.1 deg/s)."""
    u = np.zeros((k1 - k0 + 1, 3))
    u[-1] = ref / np.linalg.norm(ref)
    for i in range(k1 - 1, k0 - 1, -1):
        x = u[i - k0 + 1]
        x = x + np.cross(G[i], x) * dt
        u[i - k0] = x / np.linalg.norm(x)
    return u


def score(rows, keys):
    t = np.array([r[0] for r in rows]); ph = np.array([r[1] for r in rows])
    A = np.array([r[2] for r in rows]); G = np.array([r[3] for r in rows])
    US = np.array([r[5] for r in rows]); UA = np.array([r[6] for r in rows])
    YS = np.array([r[7] for r in rows]); YA = np.array([r[8] for r in rows])
    dt = np.median(np.diff(t))
    out, L = [], []
    L.append(f"{'test':<11}{'rotated':>8}{'peak |a|':>9} | {'tilt error at stop':^19} | "
             f"{'tilt error during motion':^34}")
    L.append(f"{'':<11}{'deg':>8}{'g':>9} | {'standard':>9}{'adaptive':>10} | "
             f"{'std rms/max':>12}{'adp rms/max':>12}{'truth +-':>10}")
    for key in keys:
        for rep in range(TESTS[key]["reps"]):
            tag = f"{key}{rep + 1}"
            mv = ph == tag + ":move"; en = ph == tag + ":end"; ho = ph == tag + ":hold"
            if mv.sum() < 10 or en.sum() < 10:
                continue
            i_mv, i_en = np.flatnonzero(mv), np.flatnonzero(en)
            wend = np.degrees(np.linalg.norm(G[i_en], axis=1))
            moved_end = wend.max() > 10 or np.abs(np.linalg.norm(A[i_en], axis=1) - 1).max() > 0.08
            ref = A[i_en[int(0.5 / dt):]].mean(0)             # gravity while still
            k0, k_stop, k1 = i_mv[0], i_mv[-1] + 1, i_en[0]
            tru = truth_back(G, ref, k0, k1, dt)             # true up from move start to still
            rot_total = np.degrees(np.sum(np.linalg.norm(G[k0:k1], axis=1)) * dt)
            peak = np.linalg.norm(A[k0:k_stop], axis=1).max()
            # at the moment the motion stopped (truth from 1.5 s of backward gyro: accurate)
            j = k_stop - k0
            e_s, e_a = ang(US[k_stop], tru[j]), ang(UA[k_stop], tru[j])
            # whole motion, if the backward-gyro truth is good enough
            T_move = (k1 - k0) * dt
            excursion = ang(tru, tru[-1]).max()
            # bias 0.1 deg/s, 1 % scale/cross-axis on the excursion, 0.1 % of the path (coning)
            unc = 0.1 * T_move + 0.01 * excursion + 0.001 * rot_total
            es_all = ang(US[k0:k1 + 1], tru); ea_all = ang(UA[k0:k1 + 1], tru)
            rms = lambda x: float(np.sqrt(np.mean(x * x)))
            during = (rms(es_all), es_all.max(), rms(ea_all), ea_all.max()) if unc < 5 else (np.nan,) * 4
            flag = "  (board not still at the end: ignore this row)" if moved_end else ""
            if unc >= 5:
                flag += f"  (too much rotation for a gyro truth during the motion)"
            if key == "magnet":
                wrap = lambda x: (x + 180) % 360 - 180
                hs = np.abs(wrap(YS[k0:k1] - np.mean(YS[ho]))).max()
                ha = np.abs(wrap(YA[k0:k1] - np.mean(YA[ho]))).max()
                flag += f"  HEADING error max: standard {hs:.1f}, adaptive {ha:.1f} deg"
            out.append(dict(test=tag, rot=rot_total, peak=peak, stop_std=e_s, stop_adp=e_a,
                            during=during, truth_unc=unc, valid=not moved_end))
            d = during
            L.append(f"{tag:<11}{rot_total:8.0f}{peak:9.2f} | {e_s:9.2f}{e_a:10.2f} | "
                     f"{d[0]:5.1f}/{d[1]:5.1f} {d[2]:5.1f}/{d[3]:5.1f}{unc:9.1f}{flag}")
    v = [o for o in out if o["valid"]]
    if v:
        dv = [o["during"] for o in v if not np.isnan(o["during"][0])]
        L.append(f"{'mean':<28} | {np.mean([o['stop_std'] for o in v]):9.2f}"
                 f"{np.mean([o['stop_adp'] for o in v]):10.2f} | "
                 + (f"{np.mean([x[0] for x in dv]):5.1f}/{np.mean([x[1] for x in dv]):5.1f} "
                    f"{np.mean([x[2] for x in dv]):5.1f}/{np.mean([x[3] for x in dv]):5.1f}" if dv else ""))
    return out, L


def plot(rows, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = np.array([r[0] for r in rows]); ph = [r[1] for r in rows]
    A = np.array([r[2] for r in rows]); US = np.array([r[5] for r in rows]); UA = np.array([r[6] for r in rows])
    KP = np.array([r[9] for r in rows])
    fig, ax = plt.subplots(3, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [2, 1, 1]})
    ax[0].plot(t, tilt_deg(A), color="#c8c8c8", lw=0.6, label="accel tilt (true only when still)")
    ax[0].plot(t, tilt_deg(US), color="#d1495b", lw=1.1, label="standard Mahony")
    ax[0].plot(t, tilt_deg(UA), color="#2e86ab", lw=1.1, label="adaptive Mahony")
    ax[0].set_ylabel("tilt from level, deg"); ax[0].legend(fontsize=8, loc="upper right")
    ax[1].plot(t, np.linalg.norm(A, axis=1), color="#e0a030", lw=0.6); ax[1].set_ylabel("|a|, g")
    ax[2].semilogy(t, np.maximum(KP, 1e-3), color="#2e86ab", lw=0.8, label="adaptive kp")
    ax[2].axhline(1, color="#d1495b", lw=0.8, label="standard kp"); ax[2].legend(fontsize=8, loc="upper right")
    ax[2].set_ylim(1e-2, 10); ax[2].set_xlabel("time, s")
    # shade the move phases, label the tests
    k = 0
    while k < len(ph):
        if ph[k].endswith(":move"):
            j = k
            while j < len(ph) and ph[j] == ph[k]:
                j += 1
            for a in ax:
                a.axvspan(t[k], t[j - 1], color="#f3e9d2", alpha=0.6, lw=0)
            ax[0].text(t[k], ax[0].get_ylim()[1] * 0.95, ph[k].split(":")[0], fontsize=8, va="top")
            k = j
        else:
            k += 1
    for a in ax:
        a.grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=120); plt.close(fig)


def replay(path, ov):
    """re-run both filters on a recorded phystest CSV (optionally other settings) and score"""
    rows_csv = list(csv.DictReader(open(path)))
    raw = [[int(r[k]) for k in ("ax", "ay", "az", "gx", "gy", "gz", "mx", "my", "mz")] for r in rows_csv]
    phases = [r["phase"] for r in rows_csv]
    t = np.array([float(r["t_s"]) for r in rows_csv])
    dt = float(np.median(np.diff(t)))
    cal = Cal()
    if cal.gbias is None:
        n = int(5.0 / dt)
        cal.gbias = np.array(raw[:n], float)[:, 3:6].mean(0) / G_LSB
    import io
    rec = Recorder(None, cal, ov, dt, io.StringIO())
    for v, ph in zip(raw, phases):
        rec.phase = ph
        rec.feed(v)
    keys = [k for k in TESTS if any(p.startswith(k) for p in phases)]
    out, L = score(rec.rows, keys)
    report = "\n".join([f"Replay of {path}   adaptive settings: {ov or 'default'}", ""] + L)
    print(report)
    base = path[:-4] + ("_replay" if ov else "_rescore")
    open(base + "_results.txt", "w").write(report + "\n")
    plot(rec.rows, base + ".png")
    print(f"Saved {base}_results.txt, {base}.png")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--replay", help="phystest_<time>.csv: re-score a recorded test")
    ap.add_argument("--only", choices=list(TESTS), action="append")
    ap.add_argument("--set", action="append", default=[], help="adaptive setting, e.g. kp_min=0.05")
    ap.add_argument("--selftest", action="store_true", help=argparse.SUPPRESS)
    args = ap.parse_args()
    ov = {k: float(v) for k, v in (s.split("=") for s in args.set)}
    if args.replay:
        replay(args.replay, ov); return
    if not args.port:
        ap.error("give --port COM5 (or --replay file.csv)")
    cal = Cal()
    keys = args.only or list(TESTS)
    if not cal.has_mag and "magnet" in keys:
        keys.remove("magnet")
        print("No calibrated magnetometer (imu_full_cal.json with a CLEAR map): magnet test skipped, 6-DOF.")

    if args.selftest:
        from physical_test_fake import FakeSerial
        ser, ask, beep = FakeSerial(), (lambda s: (print(s), FakeSerial.next_phase())), ""
    else:
        import serial
        ser, ask, beep = serial.Serial(args.port, 115200, timeout=0.05), input, "\a"
        print("Port open, Uno restarting ..."); time.sleep(2.5)
    ser.reset_input_buffer()

    print("\nPut the board flat on the table and do not touch it.")
    ask("Press Enter to start (5 s still: gyro bias and sample rate) ...")
    raw, t0 = [], time.time()
    while time.time() - t0 < 5.0:
        v = parse(ser.readline().decode("ascii", errors="ignore").strip())
        if v:
            raw.append(v)
    if len(raw) < 50:
        print("No 'Raw:' data. Upload motioncal_stream and try again."); return
    R0 = np.array(raw, float)
    dt = 5.0 / len(R0)
    gb = R0[:, 3:6].mean(0) / G_LSB
    if cal.gbias is None:
        cal.gbias = gb
    print(f"   {len(R0) / 5:.1f} Hz, gyro bias now {gb.round(3)} deg/s"
          f"{'' if cal.ok else ' (used)'},   calibration file: {'yes' if cal.ok else 'no'}")

    stamp = time.strftime("%Y%m%d_%H%M%S")
    name = f"phystest_{stamp}"
    fh = open(name + ".csv", "w", newline="")
    rec = Recorder(ser, cal, ov, dt, fh)
    for v in raw:
        rec.feed(v)
    rec.th.start()

    try:
        for key in keys:
            for rep in range(TESTS[key]["reps"]):
                run_test(rec, key, rep, ask, beep)
    except KeyboardInterrupt:
        print("\nStopped early, scoring what was recorded.")
    rec.run = False
    rec.th.join(timeout=2)
    ser.close(); fh.close()

    with rec.lock:
        rows = list(rec.rows)
    out, L = score(rows, keys)
    head = [f"Physical test {stamp}   {'9-DOF' if cal.has_mag else '6-DOF'}   "
            f"calibration file: {'yes' if cal.ok else 'no'}   adaptive settings: {ov or 'default'}",
            "Truth = accel while the board is still at the end of each test.",
            "'at stop' = error at the moment you stopped the motion. 'during' = error over the motion,",
            "against the gyro run backwards from the still attitude (truth +- = how good that is).", ""]
    report = "\n".join(head + L)
    print("\n" + report)
    open(name + "_results.txt", "w").write(report + "\n")
    plot(rows, name + ".png")
    print(f"\nSaved {name}.csv, {name}_results.txt, {name}.png")
    if not args.selftest:
        input("Press Enter to close.")


if __name__ == "__main__":
    main()
