"""
descent_sim.py : CanSat descent simulator with ground-truth attitude, for testing attitude filters.

Physics (world frame: X magnetic north, Y west, Z up):
  The payload hangs on its risers a length L below the canopy (the pivot). The pivot is pushed
  around by gusts (parachute) or flies circles (parafoil). The payload is a spherical pendulum
  driven by the pivot acceleration a_P:
      r = payload - pivot,  |r| = L,  u = -r/L (riser direction, "up the lines")
      r'' = -g z + tau u - a_P - c r'      tau = (|r'|^2 - g r_z - r . a_P) / L
  The accelerometer measures the specific force  f = tau u - c r'  (+ lever-arm terms), which is
  ALWAYS close to the riser direction, i.e. the accel says "level" whatever the swing angle.
  Body Z = u (plus a small wobble), body X = heading psi (canopy spin or parafoil track).

Scenarios: "parachute" (spin + gust-driven swing), "parafoil" (straight legs and steady turns),
"bench" (still, then slow hand rotations), "magdist" (parachute + magnetic disturbances).
All start with a 3 s deployment tumble that saturates the gyro, then a 4 g opening shock.

IMU error model taken from what we measured on the board (see IMU_measured_vs_datasheet.docx),
after calibration: residual errors only.
"""
import numpy as np

G0 = 9.80665
FS_TRUE = 1000.0


def rot_x(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def skew(w):
    return np.array([[0, -w[2], w[1]], [w[2], 0, -w[0]], [-w[1], w[0], 0]])


def expm_so3(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3) + skew(w)
    K = skew(w / th)
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def logm_so3(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1)
    th = np.arccos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / 2 if th < 1e-9 else v * th / (2 * np.sin(th))


def rot_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = np.sqrt(t + 1) * 2
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]) * 2
        q = [0.0] * 4
        q[0] = (R[k, j] - R[j, k]) / s
        q[1 + i] = 0.25 * s
        q[1 + j] = (R[j, i] + R[i, j]) / s
        q[1 + k] = (R[k, i] + R[i, k]) / s
    q = np.array(q)
    return q / np.linalg.norm(q)


def ou(n, dt, sigma, tau, rng, dims=3):
    """Ornstein-Uhlenbeck (low-pass filtered white) noise, std sigma, correlation time tau."""
    x = np.zeros((n, dims))
    a = np.exp(-dt / tau)
    b = sigma * np.sqrt(1 - a * a)
    for k in range(1, n):
        x[k] = a * x[k - 1] + b * rng.standard_normal(dims)
    return x


def simulate(scenario="parachute", T=120.0, fs=100.0, seed=0, L=1.0, dip_deg=26.0, B_mG=306.0,
             tumble_s=3.0):
    rng = np.random.default_rng(seed)
    dt = 1 / FS_TRUE
    n = int(T * FS_TRUE)
    t = np.arange(n) * dt
    c_damp = 0.15

    # ---------------- pivot acceleration a_P and payload heading psi
    aP = np.zeros((n, 3))
    vP = np.zeros((n, 3)); vP[:, 2] = -7.0          # descent rate (constant part)
    if scenario in ("parachute", "magdist"):
        aP[:, :2] = ou(n, dt, 1.2, 1.5, rng, 2)      # horizontal gusts, m/s^2
        aP[:, 2] = ou(n, dt, 0.5, 1.0, rng, 1)[:, 0]
        spin = np.radians(60 + 30 * np.sin(2 * np.pi * t / 40) + 10 * ou(n, dt, 1, 3, rng, 1)[:, 0])
        psi = np.cumsum(spin) * dt
    elif scenario == "parafoil":
        V = 7.0
        chi_dot = np.zeros(n)
        # straight 15 s, right turn 25 s, straight 10 s, left turn 25 s, ...
        legs = [(15, 0), (25, -V / 20), (10, 0), (25, V / 20), (15, 0), (30, -V / 15)]
        k = 0
        for dur, rate in legs * 3:
            m = int(dur * FS_TRUE)
            chi_dot[k:k + m] = rate
            k += m
            if k >= n:
                break
        # smooth the turn entries (a real wing rolls in over ~2 s)
        ker = np.ones(int(2 * FS_TRUE)) / (2 * FS_TRUE)
        chi_dot = np.convolve(chi_dot, ker, mode="same")
        chi = np.cumsum(chi_dot) * dt
        vP[:, 0], vP[:, 1] = V * np.cos(chi), V * np.sin(chi)
        aP[:, 0], aP[:, 1] = -V * chi_dot * np.sin(chi), V * chi_dot * np.cos(chi)
        aP[:, :2] += ou(n, dt, 0.4, 1.5, rng, 2)
        psi = chi.copy()
    elif scenario == "bench":
        psi = np.zeros(n)
    else:
        raise ValueError(scenario)

    # ---------------- pendulum (RK4) / bench motion
    r = np.zeros((n, 3)); rd = np.zeros((n, 3)); tau = np.full(n, G0)
    if scenario == "bench":
        # still 20 s, then slow hand tilts up to 40 deg and yaw turns, accel = gravity (+ small)
        up = np.zeros((n, 3))
        tb = np.clip(t - 20, 0, None)
        ramp = np.clip(tb / 5, 0, 1) ** 2 * (3 - 2 * np.clip(tb / 5, 0, 1))   # smooth start
        tilt_x = np.radians(40) * ramp * np.sin(2 * np.pi * tb / 17)
        tilt_y = np.radians(30) * ramp * np.sin(2 * np.pi * tb / 11)
        psi = np.radians(90) * ramp * np.sin(2 * np.pi * tb / 23)
        up = np.column_stack([np.sin(tilt_y), -np.sin(tilt_x) * np.cos(tilt_y),
                              np.cos(tilt_x) * np.cos(tilt_y)])
        up /= np.linalg.norm(up, axis=1)[:, None]
        r = -L * up
        rd = np.gradient(r, dt, axis=0)
        f_world = np.gradient(rd, dt, axis=0) * 0.02 + np.array([0, 0, G0])   # gravity + tiny hand accel
    else:
        def acc(rr, vv, ap):
            ta = (vv @ vv - G0 * rr[2] - rr @ ap) / L
            u = -rr / L
            return np.array([0, 0, -G0]) + ta * u - ap - c_damp * vv, ta
        th0 = np.radians(rng.uniform(10, 20)); ph0 = rng.uniform(0, 2 * np.pi)
        r[0] = L * np.array([np.sin(th0) * np.cos(ph0), np.sin(th0) * np.sin(ph0), -np.cos(th0)])
        for k in range(n - 1):
            x, v, ap = r[k], rd[k], aP[k]
            k1v, _ = acc(x, v, ap); k1x = v
            k2v, _ = acc(x + 0.5 * dt * k1x, v + 0.5 * dt * k1v, ap); k2x = v + 0.5 * dt * k1v
            k3v, _ = acc(x + 0.5 * dt * k2x, v + 0.5 * dt * k2v, ap); k3x = v + 0.5 * dt * k2v
            k4v, _ = acc(x + dt * k3x, v + dt * k3v, ap); k4x = v + dt * k3v
            xn = x + dt / 6 * (k1x + 2 * k2x + 2 * k3x + k4x)
            vn = v + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
            xn *= L / np.linalg.norm(xn)                 # keep on the sphere
            vn -= (vn @ xn) / (L * L) * xn
            r[k + 1], rd[k + 1] = xn, vn
        tau = (np.sum(rd * rd, 1) - G0 * r[:, 2] - np.sum(r * aP, 1)) / L
        up = -r / L
        f_world = tau[:, None] * up - c_damp * rd

    # ---------------- body attitude: Z along the lines, X towards heading, plus wobble
    wob = np.radians(3) * np.sin(2 * np.pi * 1.3 * t) if scenario != "bench" else np.zeros(n)
    Rb = np.zeros((n, 3, 3))
    for k in range(n):
        z = up[k]
        h = np.array([np.cos(psi[k]), np.sin(psi[k]), 0.0])
        x = h - (h @ z) * z; x /= np.linalg.norm(x)
        Rb[k] = np.column_stack([x, np.cross(z, x), z]) @ rot_x(wob[k])

    # angular velocity in body frame (from consecutive rotations)
    w_body = np.zeros((n, 3))
    for k in range(n - 1):
        w_body[k] = logm_so3(Rb[k].T @ Rb[k + 1]) / dt
    w_body[-1] = w_body[-2]

    # ---------------- deployment tumble before t = tumble_s (overrides the start)
    k_dep = int(tumble_s * FS_TRUE)
    if tumble_s > 0:
        wt = np.zeros((k_dep, 3))
        axis = rng.standard_normal(3); axis /= np.linalg.norm(axis)
        for k in range(k_dep):
            s = t[k] / tumble_s
            wt[k] = np.radians(2500 * (1 - s) + 100) * axis + np.radians(300) * np.sin(7 * t[k] + np.arange(3))
        for k in range(k_dep - 1, -1, -1):           # integrate backwards from the hanging attitude
            Rb[k] = Rb[k + 1] @ expm_so3(-wt[k] * dt)
        w_body[:k_dep] = wt
        f_world[:k_dep] = ou(k_dep, dt, 0.3 * G0, 0.05, rng)    # free fall: drag noise only
        shock = (t >= tumble_s) & (t < tumble_s + 0.3)
        f_world[shock] += 3 * G0 * up[shock]                     # opening shock along the lines

    # lever arm: IMU sits off the payload axis
    lever = np.array([0.02, 0.01, 0.03])
    alpha = np.gradient(w_body, dt, axis=0)
    f_body = np.einsum("nji,nj->ni", Rb, f_world) + np.cross(alpha, lever) + \
        np.cross(w_body, np.cross(w_body, lever))

    # magnetic field (world): north + down
    dip = np.radians(dip_deg)
    Bw = B_mG * np.array([np.cos(dip), 0, -np.sin(dip)])
    m_body = np.einsum("nji,j->ni", Rb, Bw)
    if scenario == "magdist":
        for t0, dur, amp in [(40, 6, 150), (75, 4, 300), (100, 10, 80)]:
            sel = (t >= t0) & (t < t0 + dur)
            m_body[sel] += amp * np.array([0.6, -0.5, 0.62])     # e.g. servo / heater current

    # payload velocity (for GPS)
    v_world = vP + rd

    # ---------------- sample at fs and add sensor errors
    step = int(FS_TRUE / fs)
    idx = np.arange(0, n, step)
    N = len(idx)
    # gyro: average of the true rate over the sample interval (what a real ODR filter does)
    wavg = np.array([w_body[i:i + step].mean(0) for i in idx])
    mis = np.eye(3) + rng.normal(0, 0.005, (3, 3))                # 0.5 % cross-axis
    scale = np.diag(1 + rng.normal(0, 0.003, 3))                 # 0.3 % scale
    gbias = np.radians(rng.normal(0, 0.05, 3))                   # residual bias after cal
    gwalk = np.cumsum(rng.normal(0, np.radians(0.002), (N, 3)), 0)
    gyro = (wavg @ (scale @ mis).T + gbias + gwalk + np.radians(0.1) * rng.standard_normal((N, 3)))
    gyro = np.clip(gyro, -np.radians(2000), np.radians(2000))

    acc = f_body[idx] / G0
    acc = acc @ (np.eye(3) + rng.normal(0, 0.001, (3, 3))).T + rng.normal(0, 0.002, 3) \
        + 0.002 * rng.standard_normal((N, 3))
    acc = np.clip(acc, -16, 16)

    soft = np.eye(3) + rng.normal(0, 0.01, (3, 3))
    mag = m_body[idx] @ soft.T + rng.normal(0, 5, 3) + 3.5 * rng.standard_normal((N, 3))

    # GPS velocity at 5 Hz, held, 0.1 m/s noise
    gps_step = int(fs / 5)
    vg = v_world[idx].copy()
    for k in range(0, N, gps_step):
        vg[k:k + gps_step] = vg[k] + rng.normal(0, 0.1, 3)

    return {"t": t[idx], "dt": 1 / fs, "gyro": gyro, "acc": acc, "mag": mag, "v_gps": vg,
            "R_true": Rb[idx], "w_true": wavg, "f_true": f_body[idx] / G0,
            "dip_deg": dip_deg, "B_mG": B_mG, "scenario": scenario, "tumble_s": tumble_s,
            "swing_deg": np.degrees(np.arccos(np.clip(up[idx, 2], -1, 1)))}


if __name__ == "__main__":
    d = simulate("parachute", T=30)
    print({k: (v.shape if hasattr(v, "shape") else v) for k, v in d.items()})
    print("swing angle: mean %.1f max %.1f deg" % (d["swing_deg"].mean(), d["swing_deg"].max()))
    an = np.linalg.norm(d["acc"], axis=1)
    print("|a| after deploy: %.3f .. %.3f g" % (an[500:].min(), an[500:].max()))
