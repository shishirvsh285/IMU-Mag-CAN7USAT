"""Mahony attitude filter (accel + gyro, optional mag), quaternion form."""
import math


class Mahony:
    def __init__(self, kp=1.0, ki=0.05):
        self.kp, self.ki = kp, ki
        self.q = [1.0, 0.0, 0.0, 0.0]      # w, x, y, z  (body -> world)
        self.ib = [0.0, 0.0, 0.0]          # integral term = learned gyro bias correction (rad/s)
        self.e = [0.0, 0.0, 0.0]           # last error vector

    def update(self, gx, gy, gz, ax, ay, az, dt, mx=None, my=None, mz=None, accel_ok=True):
        """g in rad/s, a in any unit (normalised here), m optional (normalised here)."""
        q0, q1, q2, q3 = self.q
        ex = ey = ez = 0.0

        an = math.sqrt(ax * ax + ay * ay + az * az)
        if accel_ok and an > 1e-6:
            ax, ay, az = ax / an, ay / an, az / an
            # gravity ("up", what the accel reads at rest) predicted in the body frame
            vx = 2 * (q1 * q3 - q0 * q2)
            vy = 2 * (q0 * q1 + q2 * q3)
            vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3
            # error = measured x predicted
            ex += ay * vz - az * vy
            ey += az * vx - ax * vz
            ez += ax * vy - ay * vx

        if mx is not None:
            mn = math.sqrt(mx * mx + my * my + mz * mz)
            if mn > 1e-6:
                mx, my, mz = mx / mn, my / mn, mz / mn
                # field in the world frame
                hx = 2 * (mx * (0.5 - q2 * q2 - q3 * q3) + my * (q1 * q2 - q0 * q3) + mz * (q1 * q3 + q0 * q2))
                hy = 2 * (mx * (q1 * q2 + q0 * q3) + my * (0.5 - q1 * q1 - q3 * q3) + mz * (q2 * q3 - q0 * q1))
                hz = 2 * (mx * (q1 * q3 - q0 * q2) + my * (q2 * q3 + q0 * q1) + mz * (0.5 - q1 * q1 - q2 * q2))
                bx, bz = math.sqrt(hx * hx + hy * hy), hz           # reference: north + vertical only
                # reference field predicted back in the body frame
                wx = 2 * (bx * (0.5 - q2 * q2 - q3 * q3) + bz * (q1 * q3 - q0 * q2))
                wy = 2 * (bx * (q1 * q2 - q0 * q3) + bz * (q0 * q1 + q2 * q3))
                wz = 2 * (bx * (q0 * q2 + q1 * q3) + bz * (0.5 - q1 * q1 - q2 * q2))
                ex += my * wz - mz * wy
                ey += mz * wx - mx * wz
                ez += mx * wy - my * wx

        self.e = [ex, ey, ez]
        if self.ki > 0:
            self.ib[0] += self.ki * ex * dt
            self.ib[1] += self.ki * ey * dt
            self.ib[2] += self.ki * ez * dt
        gx += self.kp * ex + self.ib[0]
        gy += self.kp * ey + self.ib[1]
        gz += self.kp * ez + self.ib[2]

        # q_dot = 0.5 * q (x) (0, g)
        h = 0.5 * dt
        q0, q1, q2, q3 = (q0 + (-q1 * gx - q2 * gy - q3 * gz) * h,
                          q1 + (q0 * gx + q2 * gz - q3 * gy) * h,
                          q2 + (q0 * gy - q1 * gz + q3 * gx) * h,
                          q3 + (q0 * gz + q1 * gy - q2 * gx) * h)
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / n, q1 / n, q2 / n, q3 / n]

    def euler_deg(self):
        """roll (about X), pitch (about Y), yaw (about Z), ZYX convention, degrees."""
        q0, q1, q2, q3 = self.q
        roll = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1))))
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
        return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)

    def rotmat(self):
        q0, q1, q2, q3 = self.q
        return [[1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
                [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]]
