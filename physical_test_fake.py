"""
physical_test_fake.py : fake Uno for checking physical_test.py without the board.
  python physical_test.py --port X --selftest
Pretends to be a person doing each test: plays physically consistent Raw: lines in real time
(attitude R(t), gyro from R, accel = specific force). Not used in a real test.
"""
import math
import time

import numpy as np

from physical_test import HOLD_S, TESTS

FS = 50.0
B_W = 306.0 * np.array([math.cos(math.radians(26)), 0, -math.sin(math.radians(26))])


def rx(a):
    c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def logm(R):
    c = np.clip((np.trace(R) - 1) / 2, -1, 1); th = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return v / 2 if th < 1e-9 else v * th / (2 * math.sin(th))


def smooth(x):
    x = min(max(x, 0.0), 1.0); return x * x * (3 - 2 * x)


class FakeSerial:
    seq = None
    idx = -1
    t_enter = None

    @classmethod
    def next_phase(cls):
        if cls.seq is None:
            cls.seq = [("bias", 0)] + [(k, r) for k in TESTS if k != "magnet" for r in range(TESTS[k]["reps"])]
        cls.idx += 1
        cls.t_enter = time.time()

    def __init__(self):
        self.rng = np.random.default_rng(3)
        self.t0 = time.time()
        self.k = 0
        self.R_prev = np.eye(3)
        self.cur = None
        self.R_seg0 = np.eye(3)
        self.gbias = np.radians([0.12, 0.06, -0.06])
        self.rng_axis = [v / np.linalg.norm(v) for v in self.rng.standard_normal((3, 3))]

    # ---------------- the "person"
    def pose(self, t):
        """attitude R (body->world) and specific force in body frame (g), at wall time t"""
        cls = FakeSerial
        if cls.idx < 0 or cls.seq is None or cls.idx >= len(cls.seq):
            return self.R_prev, self.R_prev.T @ [0, 0, 1]
        if self.cur != cls.idx:                       # new test: remember where we start
            self.cur = cls.idx
            self.R_seg0 = self.R_prev.copy()
        key, rep = cls.seq[cls.idx]
        tau = t - cls.t_enter
        if key == "bias":
            return self.R_seg0, self.R_seg0.T @ [0, 0, 1]
        start = {"shake": [np.eye(3), rx(math.radians(30)), ry(math.radians(-60))][rep]}.get(key, np.eye(3))
        # first second of "hold": the person moves the board to the start pose
        s = smooth(tau / 1.0)
        R0 = self.R_seg0 @ expm(logm(self.R_seg0.T @ start) * s)
        m0, m1 = HOLD_S, HOLD_S + TESTS[key]["move_s"]
        up = np.array([0, 0, 1.0])
        if key == "shake":
            aw = np.zeros(3)
            if m0 <= tau < m1:
                u = tau - m0
                aw = np.array([0.8 * math.sin(2 * math.pi * 2.1 * u), 0.6 * math.cos(2 * math.pi * 1.4 * u),
                               0.5 * math.sin(2 * math.pi * 3.2 * u)])
            return R0, R0.T @ (up + aw)
        if key in ("pendulum", "spin"):
            phi = psi = phid = 0.0
            if tau >= m0:
                u = tau - m0
                stop = math.exp(-max(0.0, tau - m1) / 0.3)   # hand stops it
                amp = math.radians(35) * math.exp(-u / 25) * stop
                w = 2 * math.pi / 1.6
                phi, phid = amp * math.sin(w * u), amp * w * math.cos(w * u)
                if key == "spin":                           # spin rate 1.5 turn/s, dying out
                    ue = min(u, m1 - m0)
                    psi = 2 * math.pi * 1.5 * 8 * (1 - math.exp(-ue / 8))
                    if tau > m1:
                        r_end = 2 * math.pi * 1.5 * math.exp(-ue / 8)
                        psi += r_end * 0.3 * (1 - math.exp(-(tau - m1) / 0.3))
            R = R0 @ rx(phi) @ rz(psi)
            T = 1 + (phid ** 2) * 0.8 / 9.81                # line tension (g), 0.8 m line
            return R, np.array([0, 0, T])
        if key == "tumble":
            if tau < m0:
                return R0, R0.T @ up
            ax = self.rng_axis[rep]
            def turn(u):
                return ax * 2.5 * u + np.radians(150) * np.array([math.sin(1.3 * u), math.cos(0.7 * u) - 1,
                                                                  math.sin(2.1 * u)])
            u = min(tau - m0, m1 - m0)
            R = R0 @ expm(turn(u))
            aw = 0.5 * np.array([math.sin(5 * u), math.cos(7 * u), math.sin(3 * u)]) if tau < m1 else np.zeros(3)
            if tau >= m1:                                   # put it down nearly flat
                s2 = smooth((tau - m1) / 1.0)
                R = R @ expm(logm(R.T @ rx(math.radians(5))) * s2)
            return R, R.T @ (up + aw)
        return R0, R0.T @ up

    def sample(self):
        t = self.t0 + self.k / FS
        R, f = self.pose(t)
        w = logm(self.R_prev.T @ R) * FS
        self.R_prev = R
        a = f + self.rng.normal(0, 0.002, 3)
        g = np.degrees(w + self.gbias) + self.rng.normal(0, 0.1, 3)
        g = np.clip(g, -2000, 2000)
        m = R.T @ B_W + self.rng.normal(0, 3.5, 3)
        v = [*np.round(a * 8192).astype(int), *np.round(g * 16).astype(int), *np.round(m).astype(int)]
        self.k += 1
        return ("Raw:" + ",".join(str(int(x)) for x in v) + "\r\n").encode()

    # ---------------- serial-like interface
    in_waiting = 0

    def readline(self):
        due = self.t0 + self.k / FS
        dt = due - time.time()
        if dt > 0:
            time.sleep(dt)
        return self.sample()

    def read(self, n=1):
        out = b""
        while self.t0 + self.k / FS <= time.time():
            out += self.sample()
        if not out:
            time.sleep(0.01)
        return out

    def reset_input_buffer(self):
        self.k = int((time.time() - self.t0) * FS)

    def close(self):
        pass


def expm(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.eye(3)
    k = w / th
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(th) * K + (1 - math.cos(th)) * K @ K
