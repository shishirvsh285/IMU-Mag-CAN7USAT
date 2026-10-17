"""
adaptive_mahony.py : Mahony filter whose accelerometer gain is set every sample from how much the
accelerometer can be trusted versus how much the gyro-propagated attitude can be trusted.
Built for a CanSat under a parachute / parafoil.

Same conventions as mahony.py: q = body -> world, world Z up, accel in g (reads +1 g "up" at rest),
gyro in rad/s, mag in any unit (normalised here; mag_field must be in the same unit if given).

THE IDEA (a one-state Kalman filter sitting on top of Mahony)
  P      : variance of our tilt error (rad^2). Grows with gyro error, shrinks with corrections.
           q = (drift_dps + drift_scale * |w|)^2   per second
           Gyro saturation / power-on sets P = pi^2 (attitude unknown).
  f_avg  : a gravity reference that does NOT depend on our estimate: the accel averaged over
           tau_avg s, carried along with the gyro (df/dt = -w x f + (a - f)/tau_avg), i.e. an
           average in the world frame. Linear acceleration averages to dv/tau_avg, which is
           small because velocity is bounded; gravity does not average away.
  R_acc  : how wrong the instant accel DIRECTION is believed to be right now (rad^2):
           R_acc = sigma0^2                                  sensor noise, vibration
                 + e_a^2                                     | |a| - 1 g |, instant or rms
                 + (1 - w_dir) * max(theta_f_rms^2, sigma_dyn^2)
           theta_f = angle(accel, f_avg).
             accel agrees with f_avg but not with our estimate -> our estimate drifted: trust it
             accel disagrees with f_avg                        -> acceleration: w_dir -> 0
           Magnitude near 1 g alone is NOT enough: under a pendulum the accel points up the
           risers at almost exactly 1 g. R_acc jumps up at once, relaxes with tau_rise.
  gains  : kp     = clamp(P / (R_acc * tau_c), kp_min, kp_max)      instant accel
           kp_avg = min(P / (sigma_avg^2 * tau_avg), kp_avg_max)   averaged accel
           dP/dt  = q - (kp + kp_avg) P
           P large / R small -> big gain (snap to the accel after deployment or on a calm bench).
           P small / R large -> only the average corrects: swings and gusts average out,
           gyro drift does not, so drift is still removed.

Magnetometer: corrects HEADING only (error projected on the vertical), gain kp_mag * w_m, with
  w_m from |B| vs mag_field and the dip angle vs dip_deg. A mag disturbance cannot tilt the
  estimate. If dip_deg is given, kp_mag_tilt > 0 lets the mag also hold tilt while the accel is
  out (scaled by 1 - w_a).
Gyro bias (Ki): learned only from trusted corrections (w_a or w_m above bias_wmin).
a_world (optional, g, world frame): known linear acceleration removed before the tests.
  GpsTurnAccel below makes it from GPS velocity: a = (turn rate of the GPS track) z x v.
  Needed for steady parafoil spirals, where the accel is steadily wrong by the bank angle. The
  track turn rate is used, NOT the gyro yaw rate: a payload spinning on its swivel under a round
  canopy does not turn its velocity.
"""
import math

G0 = 9.80665


class AdaptiveMahony:
    def __init__(self, kp_min=0.02, kp_max=5.0, tau_c=0.2, sigma0_deg=1.0, sigma_dyn_deg=10.0,
                 drift_dps=0.3, drift_scale=0.003, var_tau=0.5, tau_theta=3.0, tau_rise=0.5,
                 sigma_theta_deg=2.0, tau_avg=3.0, sigma_avg_deg=3.0, kp_avg_max=0.5,
                 kp_mag=0.5, kp_mag_tilt=0.0, mag_field=None, sigma_b=0.08, dip_deg=None,
                 sigma_dip_deg=4.0, ki=0.02, bias_wmin=0.5, bias_limit_dps=2.0,
                 gyro_range_dps=2000.0, startup_s=0.5):
        d = math.radians
        self.kp_min, self.kp_max, self.tau_c = kp_min, kp_max, tau_c
        self.s0, self.sdyn = d(sigma0_deg), d(sigma_dyn_deg)
        self.drift, self.drift_scale = d(drift_dps), drift_scale
        self.var_tau, self.tau_theta, self.tau_rise = var_tau, tau_theta, tau_rise
        self.sig_th = d(sigma_theta_deg)
        self.tau_avg, self.s_avg, self.kp_avg_max = tau_avg, d(sigma_avg_deg), kp_avg_max
        self.kp_mag, self.kp_mag_tilt = kp_mag, kp_mag_tilt
        self.mag_field, self.sigma_b = mag_field, sigma_b
        self.dip = None if dip_deg is None else d(dip_deg)
        self.sig_dip = d(sigma_dip_deg)
        self.ki, self.bias_wmin, self.bias_lim = ki, bias_wmin, d(bias_limit_dps)
        self.gyro_sat = 0.95 * d(gyro_range_dps)
        self.startup_s = startup_s
        self.reset()

    def reset(self, q=None):
        self.q = list(q) if q else [1.0, 0.0, 0.0, 0.0]
        self.ib = [0.0, 0.0, 0.0]
        self.t = 0.0
        self.P = math.pi ** 2          # tilt error variance: unknown at power-on
        self.Racc = self.s0 ** 2
        self.ea2 = 0.0                 # low-passed (|a|-1)^2
        self.th2 = 0.0                 # low-passed theta_f^2
        self.fb = [0.0, 0.0, 1.0]      # gyro-carried average of the accel (body frame)
        self.wa = self.wm = 1.0
        self.theta = self.ea = self.kp = self.kp_avg = 0.0
        self.e = [0.0, 0.0, 0.0]

    @property
    def U(self):
        """1-sigma tilt uncertainty, rad"""
        return math.sqrt(self.P)

    # ------------------------------------------------------------------ helpers
    def rotmat(self):
        q0, q1, q2, q3 = self.q
        return [[1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
                [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]]

    def euler_deg(self):
        q0, q1, q2, q3 = self.q
        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def _smooth_w(self, w, w_raw, dt):          # trust: drops at once, recovers slowly
        return w_raw if w_raw < w else w + (w_raw - w) * min(1.0, dt / self.tau_rise)

    # ------------------------------------------------------------------ main step
    def update(self, gx, gy, gz, ax, ay, az, dt, mx=None, my=None, mz=None, a_world=None):
        self.t += dt
        R = self.rotmat()
        vx, vy, vz = R[2][0], R[2][1], R[2][2]          # predicted "up" in body frame
        wn = math.sqrt(gx * gx + gy * gy + gz * gz)

        # --- prediction: tilt error variance grows with gyro error
        qd = self.drift + self.drift_scale * wn
        self.P += qd * qd * dt
        if max(abs(gx), abs(gy), abs(gz)) > self.gyro_sat or self.t < self.startup_s:
            self.P = math.pi ** 2                       # attitude unknown (tumble, power-on)
        self.P = min(self.P, math.pi ** 2)
        lost = self.P > math.radians(20) ** 2

        # --- remove known linear acceleration (parafoil turn, from GPS)
        if a_world is not None and not lost:
            awx, awy, awz = a_world
            ax -= R[0][0] * awx + R[1][0] * awy + R[2][0] * awz      # R^T a_world
            ay -= R[0][1] * awx + R[1][1] * awy + R[2][1] * awz
            az -= R[0][2] * awx + R[1][2] * awy + R[2][2] * awz

        ex = ey = ez = 0.0
        eax = eay = eaz = 0.0
        self.kp = self.kp_avg = 0.0
        # --- gravity reference that does not depend on our estimate: the accel averaged over
        #     tau_avg seconds, carried along with the gyro (= a world-frame average).
        #     Dynamic acceleration averages to dv/tau (velocity is bounded), gravity does not.
        fx, fy, fz = self.fb
        gxc, gyc, gzc = gx + self.ib[0], gy + self.ib[1], gz + self.ib[2]
        fx, fy, fz = (fx - (gyc * fz - gzc * fy) * dt, fy - (gzc * fx - gxc * fz) * dt,
                      fz - (gxc * fy - gyc * fx) * dt)
        k_avg = min(1.0, dt / self.tau_avg)
        fx, fy, fz = fx + (ax - fx) * k_avg, fy + (ay - fy) * k_avg, fz + (az - fz) * k_avg
        if lost:
            fx, fy, fz = ax, ay, az
        self.fb = [fx, fy, fz]
        fn = math.sqrt(fx * fx + fy * fy + fz * fz)

        an = math.sqrt(ax * ax + ay * ay + az * az)
        if an > 1e-6 and fn > 1e-6:
            ax, ay, az = ax / an, ay / an, az / an
            fx, fy, fz = fx / fn, fy / fn, fz / fn
            # magnitude: instant error or its recent rms, whichever is worse (in g ~ rad)
            e = an - 1.0
            self.ea2 += (e * e - self.ea2) * min(1.0, dt / self.var_tau)
            self.ea = math.sqrt(max(e * e, self.ea2))
            # direction: accel vs the averaged gravity. Agreeing with the average but not with
            # our estimate = our estimate drifted. Disagreeing with the average = acceleration.
            self.theta = math.acos(max(-1.0, min(1.0, ax * vx + ay * vy + az * vz)))
            th_f = math.acos(max(-1.0, min(1.0, ax * fx + ay * fy + az * fz)))
            self.th2 += (th_f * th_f - self.th2) * min(1.0, dt / self.tau_theta)
            x = max(0.0, th_f - self.s0)
            w_dir = math.exp(-x * x / (2 * self.sig_th ** 2))
            r_new = self.s0 ** 2 + self.ea ** 2 + (1.0 - w_dir) * max(self.th2, self.sdyn ** 2)
            # hysteresis: distrust at once, regain trust over tau_rise
            self.Racc = r_new if r_new > self.Racc else \
                self.Racc + (r_new - self.Racc) * min(1.0, dt / self.tau_rise)
            self.wa = self.s0 ** 2 / self.Racc          # 1 = clean gravity, 0 = useless
            # Kalman-like gains: instant accel (noise Racc) and averaged accel (noise s_avg^2)
            self.kp = max(self.kp_min, min(self.kp_max, self.P / (self.Racc * self.tau_c)))
            self.kp_avg = min(self.kp_avg_max, self.P / (self.s_avg ** 2 * self.tau_avg))
            self.P -= (self.kp + self.kp_avg) * self.P * dt
            self.P = max(self.P, 1e-8)
            eax, eay, eaz = ay * vz - az * vy, az * vx - ax * vz, ax * vy - ay * vx
            efx, efy, efz = fy * vz - fz * vy, fz * vx - fx * vz, fx * vy - fy * vx
            ex = self.kp * eax + self.kp_avg * efx
            ey = self.kp * eay + self.kp_avg * efy
            ez = self.kp * eaz + self.kp_avg * efz
        wa = self.wa

        # --- magnetometer: heading always, tilt only when accel is out (and dip known)
        emh = [0.0, 0.0, 0.0]
        if mx is not None:
            mn = math.sqrt(mx * mx + my * my + mz * mz)
            if mn > 1e-9:
                w_b = 1.0
                if self.mag_field:
                    r = mn / self.mag_field - 1.0
                    w_b = math.exp(-r * r / (2 * self.sigma_b ** 2))
                mx, my, mz = mx / mn, my / mn, mz / mn
                w_dip = 1.0
                if self.dip is not None:
                    elev = math.asin(max(-1.0, min(1.0, mx * vx + my * vy + mz * vz)))
                    x = max(0.0, abs(elev + self.dip) - 2.0 * math.sqrt(self.P))
                    w_dip = math.exp(-x * x / (2 * self.sig_dip ** 2))
                self.wm = self._smooth_w(self.wm, w_b * w_dip, dt)
                # field in the world frame, reference = north + vertical
                hx = R[0][0] * mx + R[0][1] * my + R[0][2] * mz
                hy = R[1][0] * mx + R[1][1] * my + R[1][2] * mz
                hz = R[2][0] * mx + R[2][1] * my + R[2][2] * mz
                if self.dip is not None:
                    bx, bz = math.cos(self.dip), -math.sin(self.dip)
                else:
                    bx, bz = math.sqrt(hx * hx + hy * hy), hz
                wx = R[0][0] * bx + R[2][0] * bz                  # R^T (bx, 0, bz)
                wy = R[0][1] * bx + R[2][1] * bz
                wz = R[0][2] * bx + R[2][2] * bz
                emx, emy, emz = my * wz - mz * wy, mz * wx - mx * wz, mx * wy - my * wx
                p = emx * vx + emy * vy + emz * vz                  # heading part = along "up"
                emh = [p * vx, p * vy, p * vz]
                k = self.kp_mag * self.wm * (4.0 if lost else 1.0)
                kt = self.kp_mag_tilt * self.wm * (1.0 - wa) if self.dip is not None else 0.0
                ex += k * emh[0] + kt * (emx - emh[0])
                ey += k * emh[1] + kt * (emy - emh[1])
                ez += k * emh[2] + kt * (emz - emh[2])
        self.e = [ex, ey, ez]

        # --- gyro bias: learn only from trusted corrections
        if self.ki > 0 and not lost:
            ga = self.ki * dt * (wa if wa > self.bias_wmin else 0.0)
            gm = self.ki * dt * (self.wm if self.wm > self.bias_wmin and mx is not None else 0.0)
            for i, (ea_i, em_i) in enumerate(zip((eax, eay, eaz), emh)):
                self.ib[i] = max(-self.bias_lim, min(self.bias_lim, self.ib[i] + ga * ea_i + gm * em_i))

        gx += ex + self.ib[0]
        gy += ey + self.ib[1]
        gz += ez + self.ib[2]

        q0, q1, q2, q3 = self.q
        h = 0.5 * dt
        q0, q1, q2, q3 = (q0 + (-q1 * gx - q2 * gy - q3 * gz) * h,
                          q1 + (q0 * gx + q2 * gz - q3 * gy) * h,
                          q2 + (q0 * gy - q1 * gz + q3 * gx) * h,
                          q3 + (q0 * gz + q1 * gy - q2 * gx) * h)
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / n, q1 / n, q2 / n, q3 / n]


class GpsTurnAccel:
    """Horizontal acceleration of a turning flight path, from GPS velocity fixes (world X, Y).
    a = chi_dot * z x v_h, chi = track angle. Off below min_speed (track angle meaningless)."""

    def __init__(self, tau=1.0, min_speed=2.0):
        self.tau, self.min_speed = tau, min_speed
        self.prev = None
        self.rate = 0.0
        self.a = (0.0, 0.0, 0.0)

    def fix(self, vx, vy, t):
        """call once per new GPS velocity fix (m/s, s). Returns a_world in g."""
        sp = math.hypot(vx, vy)
        if self.prev is not None and sp > self.min_speed:
            pvx, pvy, pt = self.prev
            dt = t - pt
            if dt > 0 and math.hypot(pvx, pvy) > self.min_speed:
                d = math.atan2(pvx * vy - pvy * vx, pvx * vx + pvy * vy)   # track change
                self.rate += (d / dt - self.rate) * min(1.0, dt / self.tau)
        else:
            self.rate = 0.0
        self.prev = (vx, vy, t)
        self.a = (-self.rate * vy / G0, self.rate * vx / G0, 0.0) if sp > self.min_speed else (0.0, 0.0, 0.0)
        return self.a
