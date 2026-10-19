"""
full_calibration.py : one-shot calibration of the BMI270 + IIS2MDC board.

Uno must run motioncal_stream (RAW magnetometer, not mag_calibrated). Lines at ~50 Hz:
  Raw:ax,ay,az,gx,gy,gz,mx,my,mz   (accel 8192/g, gyro 16/(deg/s), mag 1 mG)

  python full_calibration.py --port COM5

Steps (the script tells you what to do at each one):
  1. Gyro bias     : board still for 10 s
  2. Accelerometer : 6 still poses (+Z, -Z, +X, -X, +Y, -Y up), 3 s each
  3. Magnetometer  : slow tumble through every direction (~60 s), away from metal
  4. Fit + checks  : accel offset/gain, gyro bias, mag hard/soft iron, mag -> IMU axis map
Outputs: imu_full_cal.json, imu_cal.h (constants for Arduino code), printed summary.
"""
import argparse
import itertools
import json
import sys
import time

import numpy as np
import serial

A_LSB, G_LSB = 8192.0, 16.0          # stream units
POSES = [("+Z up", "flat, components UP", 2, 1), ("-Z up", "flat, components DOWN", 2, -1),
         ("+X up", "on its edge, +X pointing up", 0, 1), ("-X up", "flip it, -X up", 0, -1),
         ("+Y up", "on the other edge, +Y up", 1, 1), ("-Y up", "flip it, -Y up", 1, -1)]
DATASHEET = {"acc_off_mg": 20, "acc_sens_pct": 0.4, "gyr_off_dps": 0.5, "mag_noise_mG": 4.6}


# ---------------------------------------------------------------- serial
def parse(line):
    if not line.startswith("Raw:"):
        return None
    try:
        v = [int(x) for x in line[4:].split(",")]
    except ValueError:
        return None
    return v if len(v) == 9 else None


def read_for(ser, seconds, progress=None):
    ser.reset_input_buffer()
    rows, t0, last = [], time.time(), 0
    while time.time() - t0 < seconds:
        v = parse(ser.readline().decode("ascii", errors="ignore").strip())
        if v:
            rows.append(v)
        if progress and time.time() - last > 1:
            last = time.time(); progress(np.array(rows, float), time.time() - t0)
    d = np.array(rows, float)
    if len(d) == 0:
        return d, d, d
    return d[:, 0:3] / A_LSB, d[:, 3:6] / G_LSB, d[:, 6:9]   # g, deg/s, mG


# ---------------------------------------------------------------- fits
def fit_accel(P):
    """offset o, gain k per axis so that |(m - o) * k| = 1 in every pose (Gauss-Newton)."""
    o = (P.max(0) + P.min(0)) / 2
    k = 2 / (P.max(0) - P.min(0))
    for _ in range(25):
        c = (P - o) * k
        n = np.linalg.norm(c, axis=1)
        J = np.hstack([-(c * k) / n[:, None], (c * (P - o)) / n[:, None]])
        dx, *_ = np.linalg.lstsq(J, -(n - 1), rcond=None)
        o += dx[:3]; k += dx[3:]
    return o, k


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
        raise ValueError("not an ellipsoid: tumble did not cover enough directions")
    radii = 1 / np.sqrt(vals)
    R = radii.mean()
    W = vecs @ np.diag(np.sqrt(vals)) @ vecs.T * R
    return b, W, R, radii


def robust_mag_fit(m):
    B = np.linalg.norm(m, axis=1)
    keep = np.abs(B - np.median(B)) < 0.6 * np.median(B)          # drop magnet spikes
    b, W, R, radii = fit_ellipsoid(m[keep])
    for _ in range(2):
        r = np.linalg.norm((m - b) @ W.T, axis=1) / R
        keep = np.abs(r - 1) < 0.15
        b, W, R, radii = fit_ellipsoid(m[keep])
    return b, W, R, radii, keep


def coverage(u):
    u = u / np.linalg.norm(u, axis=1)[:, None]
    return len({tuple(c) for c in np.round(u * 1.4).astype(int) if np.any(c)}) / 26


def best_axis_map(a, m):
    """
    Signed permutation S (mag axes -> BMI270 axes) that keeps the angle between gravity and the
    magnetic field constant across orientations (it is a property of the place, not the board).
    """
    an = a / np.linalg.norm(a, axis=1)[:, None]
    mn = m / np.linalg.norm(m, axis=1)[:, None]
    res = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product((1, -1), repeat=3):
            S = np.zeros((3, 3))
            for i in range(3):
                S[i, perm[i]] = signs[i]
            dots = np.sum(an * (mn @ S.T), axis=1)
            res.append((dots.std(), S, dots.mean()))
    # S and -S give the same spread (all dots flip sign), so a sign check is needed.
    # The IIS2MDC frame is left-handed (datasheet Fig. 2) and the BMI270 frame is right-handed,
    # so the true map has det = -1. Keep only those.
    res = [r for r in res if np.linalg.det(r[1]) < 0]
    res.sort(key=lambda r: r[0])
    return res[0], res[1]


def map_name(S):
    out = []
    for i in range(3):
        j = int(np.argmax(np.abs(S[i])))
        out.append(("-" if S[i, j] < 0 else "") + "xyz"[j])
    return ",".join(out)


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--tumble", type=float, default=60.0, help="magnetometer tumble time, s")
    args = ap.parse_args()

    ser = serial.Serial(args.port, 115200, timeout=0.2)
    print("Opening port (the Uno restarts) ..."); time.sleep(3)
    a, g, m = read_for(ser, 1.0)
    if len(a) < 20:
        print("No 'Raw:' data. Upload motioncal_stream and try again."); input("Enter to close"); return
    print(f"Stream OK, {len(a)} samples/s.\n")

    # 1. gyro bias
    input("STEP 1/3  GYRO BIAS: put the board flat and do not touch it. Press Enter ...")
    print("   measuring 10 s ...")
    a1, g1, m1 = read_for(ser, 10.0)
    if a1.std(0).max() > 0.01:
        print("   WARNING: the board moved during the gyro step, bias may be off.")
    gbias, gnoise = g1.mean(0), g1.std(0)
    print(f"   gyro bias {gbias.round(4)} deg/s, noise {gnoise.round(3)} deg/s\n")

    # 2. accel poses
    print("STEP 2/3  ACCELEROMETER: 6 poses. Press each one against a box or book so it stays still.")
    P, PM = [], []
    i = 0
    while i < 6:
        name, how, ax_i, sgn = POSES[i]
        input(f"   [{i + 1}/6] {name}: {how}. Press Enter ...")
        a2, g2, m2 = read_for(ser, 3.0)
        if len(a2) < 30:
            print("      no data, again"); continue
        mean, sd = a2.mean(0), a2.std(0).max() * 1000
        k = int(np.argmax(np.abs(mean)))
        if sd > 8:
            print(f"      moved (noise {sd:.1f} mg), again"); continue
        if k != ax_i or np.sign(mean[k]) != sgn or abs(mean[k]) < 0.8:
            print(f"      WRONG POSE: board reads {'+' if mean[k] > 0 else '-'}{'XYZ'[k]} up, wanted {name}. Again."); continue
        print(f"      ok  {mean.round(4)} g")
        P.append(mean); PM.append(m2.mean(0)); i += 1
    P, PM = np.array(P), np.array(PM)

    # 3. mag tumble
    print(f"\nSTEP 3/3  MAGNETOMETER: slowly turn the board through EVERY direction for {args.tumble:.0f} s.")
    print("   Flip it, roll it, stand it on each edge, point it along diagonals. Keep magnets, phones and steel away.")
    input("   Press Enter to start ...")

    def prog(d, t):
        if len(d) > 20:
            cov = coverage(d[:, 6:9] - np.median(d[:, 6:9], 0))
            sys.stdout.write(f"\r   {t:4.0f} s   samples {len(d):5d}   direction coverage {cov * 100:3.0f} %   ")
            sys.stdout.flush()
    a3, g3, m3 = read_for(ser, args.tumble, prog)
    print()
    ser.close()

    # 4. fits
    acc_off, acc_gain = fit_accel(P)
    mag_all = np.vstack([m3, PM])
    b, W, R, radii, keep = robust_mag_fit(mag_all)
    mcal = (mag_all - b) @ W.T
    Braw, Bcal = np.linalg.norm(mag_all[keep], axis=1), np.linalg.norm(mcal[keep], axis=1)
    cov = coverage(mcal[keep])

    # mag -> IMU axis map from still-ish samples (|a| near 1 g, low rotation) plus the 6 poses
    a3c = (a3 - acc_off) * acc_gain
    g3c = g3 - gbias
    still = (np.abs(np.linalg.norm(a3c, axis=1) - 1) < 0.05) & (np.linalg.norm(g3c, axis=1) < 20)
    A_use = np.vstack([(P - acc_off) * acc_gain, a3c[still]])
    M_use = np.vstack([(PM - b) @ W.T, ((m3 - b) @ W.T)[still]])
    (best_sd, S, cosdip), (next_sd, S2, _) = best_axis_map(A_use, M_use)
    dip = np.degrees(np.arcsin(np.clip(-cosdip, -1, 1)))
    map_ok = next_sd > 2 * best_sd and best_sd < 0.05

    # 5. checks
    acc_after = np.abs(np.linalg.norm((P - acc_off) * acc_gain, axis=1) - 1)
    acc_before = np.abs(np.linalg.norm(P, axis=1) - 1)
    a_tumble = np.linalg.norm(a3c[still], axis=1)

    L = []
    L.append("=" * 64)
    L.append("FINAL CALIBRATION RESULT")
    L.append("=" * 64)
    L.append("GYROSCOPE   w = raw - bias")
    for j, n in enumerate("XYZ"):
        ok = "ok" if abs(gbias[j]) <= DATASHEET["gyr_off_dps"] else "OUT OF SPEC"
        L.append(f"  {n}: bias {gbias[j]:+.4f} deg/s   noise {gnoise[j]:.3f}   (datasheet +-0.5)  {ok}")
    L.append("ACCELEROMETER   a = (raw - offset) * gain")
    for j, n in enumerate("XYZ"):
        se = (1 / acc_gain[j] - 1) * 100
        L.append(f"  {n}: offset {acc_off[j] * 1000:+7.2f} mg   gain {acc_gain[j]:.5f}   scale error {se:+.3f} %"
                 f"   (datasheet +-20 mg, +-0.4 %)  {'ok' if abs(acc_off[j]) * 1000 <= 20 and abs(se) <= 0.4 else 'CHECK'}")
    L.append(f"  |a| error in the 6 poses: before {acc_before.max() * 1000:.2f} mg, after {acc_after.max() * 1000:.2f} mg")
    if still.sum() > 20:
        L.append(f"  |a| on {still.sum()} still-ish tumble samples: {a_tumble.mean():.4f} g, sd {a_tumble.std() * 1000:.1f} mg")
    L.append("MAGNETOMETER   m = W @ (raw - b)")
    L.append(f"  hard iron b: X {b[0]:+.1f}  Y {b[1]:+.1f}  Z {b[2]:+.1f} mG")
    L.append("  soft iron W:")
    for row in W:
        L.append("     " + "  ".join(f"{x:+.4f}" for x in row))
    L.append(f"  field radius {R:.1f} mG,  ellipsoid radii {radii.round(1)} (axis ratio {radii.max() / radii.min():.3f})")
    L.append(f"  |B| spread: raw {Braw.std() / R * 100:.1f} %  ->  calibrated {Bcal.std() / R * 100:.2f} %"
             f"   (good: under ~2 %)")
    L.append(f"  direction coverage {cov * 100:.0f} %  (good: over ~80 %),  samples used {keep.sum()} of {len(keep)}")
    L.append("MAG -> IMU AXES   m_imu = S @ m")
    L.append(f"  best map: \"{map_name(S)}\"   (det {np.linalg.det(S):+.0f}, gravity-field angle sd {best_sd:.4f}, next best {next_sd:.4f})")
    L.append(f"  magnetic dip angle here: {dip:.1f} deg   {'map is CLEAR' if map_ok else 'map is UNCERTAIN, redo the tumble with more variety'}")
    L.append("=" * 64)
    report = "\n".join(L)
    print("\n" + report)

    cal = {"created": time.strftime("%Y-%m-%d %H:%M:%S"),
           "gyro_bias_dps": gbias.round(5).tolist(), "gyro_noise_dps": gnoise.round(4).tolist(),
           "accel_offset_g": acc_off.round(6).tolist(), "accel_gain": acc_gain.round(6).tolist(),
           "mag_hard_iron_mG": b.round(3).tolist(), "mag_soft_iron": W.round(6).tolist(),
           "mag_field_mG": round(float(R), 2), "mag_axis_map": map_name(S), "mag_axis_map_matrix": S.tolist(),
           "mag_axis_map_clear": bool(map_ok), "dip_deg": round(float(dip), 2),
           "quality": {"mag_coverage": round(cov, 3), "mag_B_spread_pct": round(float(Bcal.std() / R * 100), 3)},
           "usage": {"gyro": "w = raw/16 - gyro_bias", "accel": "a = (raw/8192 - offset) * gain",
                     "mag": "m = mag_axis_map_matrix @ (mag_soft_iron @ (raw - mag_hard_iron))"}}
    with open("imu_full_cal.json", "w") as f:
        json.dump(cal, f, indent=2)
    with open("imu_full_cal_report.txt", "w") as f:
        f.write(report + "\n")

    fmt = lambda v: ", ".join(f"{x:.6f}f" for x in v)
    h = ["// Generated by full_calibration.py " + cal["created"],
         "#pragma once",
         f"const float GYRO_BIAS_DPS[3] = {{{fmt(gbias)}}};      // w = raw_dps - bias",
         f"const float ACC_OFFSET_G[3]  = {{{fmt(acc_off)}}};    // a = (raw_g - offset) * gain",
         f"const float ACC_GAIN[3]      = {{{fmt(acc_gain)}}};",
         f"const float MAG_HARD_MG[3]   = {{{fmt(b)}}};          // m = W * (raw_mG - hard)",
         "const float MAG_SOFT[3][3]   = {" + ", ".join("{" + fmt(r) + "}" for r in W) + "};",
         "const int   MAG_MAP[3][3]    = {" + ", ".join("{" + ", ".join(str(int(x)) for x in r) + "}" for r in S)
         + f"}};  // m_imu = MAP * m   (\"{map_name(S)}\")",
         f"#define USE_MAG {1 if map_ok else 0}            // 1 = mag map was CLEAR",
         f"const float MAG_DIP_DEG = {dip:.2f}f;",
         f"const float MAG_FIELD_MG = {R:.1f}f;"]
    with open("imu_cal.h", "w") as f:
        f.write("\n".join(h) + "\n")
    print("Saved imu_full_cal.json, imu_full_cal_report.txt, imu_cal.h")
    input("Press Enter to close.")


if __name__ == "__main__":
    main()
