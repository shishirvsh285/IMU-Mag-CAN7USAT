// AdaptiveMahony.h : adaptive Mahony attitude filter for a CanSat under parachute / parafoil.
// Line-by-line port of adaptive_mahony.py (same names, same equations), float only.
//
// Units: gyro rad/s, accel g, mag any unit (mag_field in the same unit, or 0 to skip |B| test).
// Frames: q = body -> world, world Z up, accel reads +1 g "up" at rest.
//
//   AdaptiveMahony f;              // defaults = the tuned values
//   f.magField = 306; f.dipDeg = 26; f.useDip = true;
//   f.update(gx, gy, gz, ax, ay, az, dt, mag /*float[3] or nullptr*/, aWorld /*float[3] or nullptr*/);
//   f.q[0..3], f.euler(roll, pitch, yaw)
//
// Cost per update: ~5 sqrt, 2 acos, 3 exp, 1 asin, ~220 mul/add (all float).
#pragma once
#include <math.h>

class AdaptiveMahony
{
public:
  // ---- settings (see adaptive_mahony.py for what each one does)
  float kpMin = 0.02f, kpMax = 5.0f, tauC = 0.2f;
  float sigma0 = 1.0f * DEG, sigmaDyn = 10.0f * DEG;
  float drift = 0.3f * DEG, driftScale = 0.003f;
  float varTau = 0.5f, tauTheta = 3.0f, tauRise = 0.5f, sigTheta = 2.0f * DEG;
  float tauAvg = 3.0f, sAvg = 3.0f * DEG, kpAvgMax = 0.5f;
  float kpMag = 0.5f, kpMagTilt = 0.0f, magField = 0.0f, sigmaB = 0.08f;
  bool useDip = false; float dipDeg = 0.0f; float sigDip = 4.0f * DEG;
  float ki = 0.02f, biasWmin = 0.5f, biasLim = 2.0f * DEG;
  float gyroSat = 0.95f * 2000.0f * DEG, startupS = 0.5f;

  // ---- state / diagnostics
  float q[4] = {1, 0, 0, 0};
  float ib[3] = {0, 0, 0};
  float t = 0, P = PI_F * PI_F, Racc = 0, ea2 = 0, th2 = 0;
  float wa = 1, wm = 1, theta = 0, ea = 0, kp = 0, kpAvg = 0;
  float fb[3] = {0, 0, 1};

  static constexpr float DEG = 0.017453292519943295f;
  static constexpr float PI_F = 3.14159265358979f;

  AdaptiveMahony() { Racc = sigma0 * sigma0; }

  void reset()
  {
    q[0] = 1; q[1] = q[2] = q[3] = 0; ib[0] = ib[1] = ib[2] = 0;
    t = 0; P = PI_F * PI_F; Racc = sigma0 * sigma0; ea2 = th2 = 0; wa = wm = 1;
    fb[0] = fb[1] = 0; fb[2] = 1;
  }

  void euler(float &roll, float &pitch, float &yaw) const
  {
    roll = atan2f(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) / DEG;
    pitch = asinf(clampf(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1)) / DEG;
    yaw = atan2f(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) / DEG;
  }

  void update(float gx, float gy, float gz, float ax, float ay, float az, float dt,
              const float *mag = nullptr, const float *aWorld = nullptr)
  {
    t += dt;
    const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
    const float R00 = 1 - 2 * (q2 * q2 + q3 * q3), R01 = 2 * (q1 * q2 - q0 * q3), R02 = 2 * (q1 * q3 + q0 * q2);
    const float R10 = 2 * (q1 * q2 + q0 * q3), R11 = 1 - 2 * (q1 * q1 + q3 * q3), R12 = 2 * (q2 * q3 - q0 * q1);
    const float R20 = 2 * (q1 * q3 - q0 * q2), R21 = 2 * (q2 * q3 + q0 * q1), R22 = 1 - 2 * (q1 * q1 + q2 * q2);
    const float vx = R20, vy = R21, vz = R22;               // predicted "up" in body frame
    const float wn = sqrtf(gx * gx + gy * gy + gz * gz);

    // prediction: tilt variance grows with gyro error
    const float qd = drift + driftScale * wn;
    P += qd * qd * dt;
    if (fabsf(gx) > gyroSat || fabsf(gy) > gyroSat || fabsf(gz) > gyroSat || t < startupS) P = PI_F * PI_F;
    if (P > PI_F * PI_F) P = PI_F * PI_F;
    const bool lost = P > (20 * DEG) * (20 * DEG);

    if (aWorld && !lost)                                     // known linear accel (GPS turn)
    {
      ax -= R00 * aWorld[0] + R10 * aWorld[1] + R20 * aWorld[2];
      ay -= R01 * aWorld[0] + R11 * aWorld[1] + R21 * aWorld[2];
      az -= R02 * aWorld[0] + R12 * aWorld[1] + R22 * aWorld[2];
    }

    float ex = 0, ey = 0, ez = 0, eax = 0, eay = 0, eaz = 0;
    kp = kpAvg = 0;
    // gravity reference independent of the estimate: accel averaged over tauAvg, carried by the gyro
    {
      const float gxc = gx + ib[0], gyc = gy + ib[1], gzc = gz + ib[2];
      const float fx = fb[0], fy = fb[1], fz = fb[2];
      float nx = fx - (gyc * fz - gzc * fy) * dt, ny = fy - (gzc * fx - gxc * fz) * dt,
            nz = fz - (gxc * fy - gyc * fx) * dt;
      const float k = fminf(1.0f, dt / tauAvg);
      nx += (ax - nx) * k; ny += (ay - ny) * k; nz += (az - nz) * k;
      if (lost) { nx = ax; ny = ay; nz = az; }
      fb[0] = nx; fb[1] = ny; fb[2] = nz;
    }
    const float fn = sqrtf(fb[0] * fb[0] + fb[1] * fb[1] + fb[2] * fb[2]);
    const float an = sqrtf(ax * ax + ay * ay + az * az);
    if (an > 1e-6f && fn > 1e-6f)
    {
      ax /= an; ay /= an; az /= an;
      const float fx = fb[0] / fn, fy = fb[1] / fn, fz = fb[2] / fn;
      const float e = an - 1.0f;
      ea2 += (e * e - ea2) * fminf(1.0f, dt / varTau);
      ea = sqrtf(fmaxf(e * e, ea2));
      theta = acosf(clampf(ax * vx + ay * vy + az * vz, -1, 1));
      const float thF = acosf(clampf(ax * fx + ay * fy + az * fz, -1, 1));
      th2 += (thF * thF - th2) * fminf(1.0f, dt / tauTheta);
      const float x = fmaxf(0.0f, thF - sigma0);
      const float wDir = expf(-x * x / (2 * sigTheta * sigTheta));
      const float rNew = sigma0 * sigma0 + ea * ea + (1.0f - wDir) * fmaxf(th2, sigmaDyn * sigmaDyn);
      Racc = (rNew > Racc) ? rNew : Racc + (rNew - Racc) * fminf(1.0f, dt / tauRise);
      wa = sigma0 * sigma0 / Racc;
      kp = clampf(P / (Racc * tauC), kpMin, kpMax);
      kpAvg = fminf(kpAvgMax, P / (sAvg * sAvg * tauAvg));
      P -= (kp + kpAvg) * P * dt;
      if (P < 1e-8f) P = 1e-8f;
      eax = ay * vz - az * vy; eay = az * vx - ax * vz; eaz = ax * vy - ay * vx;
      ex = kp * eax + kpAvg * (fy * vz - fz * vy);
      ey = kp * eay + kpAvg * (fz * vx - fx * vz);
      ez = kp * eaz + kpAvg * (fx * vy - fy * vx);
    }

    float emh0 = 0, emh1 = 0, emh2 = 0;
    if (mag)
    {
      float mx = mag[0], my = mag[1], mz = mag[2];
      const float mn = sqrtf(mx * mx + my * my + mz * mz);
      if (mn > 1e-9f)
      {
        float wB = 1;
        if (magField > 0) { const float r = mn / magField - 1; wB = expf(-r * r / (2 * sigmaB * sigmaB)); }
        mx /= mn; my /= mn; mz /= mn;
        float wDip = 1;
        const float dip = dipDeg * DEG;
        if (useDip)
        {
          const float elev = asinf(clampf(mx * vx + my * vy + mz * vz, -1, 1));
          const float x = fmaxf(0.0f, fabsf(elev + dip) - 2.0f * sqrtf(P));
          wDip = expf(-x * x / (2 * sigDip * sigDip));
        }
        const float wRaw = wB * wDip;
        wm = (wRaw < wm) ? wRaw : wm + (wRaw - wm) * fminf(1.0f, dt / tauRise);
        float bx, bz;
        if (useDip) { bx = cosf(dip); bz = -sinf(dip); }
        else
        {
          const float hx = R00 * mx + R01 * my + R02 * mz;
          const float hy = R10 * mx + R11 * my + R12 * mz;
          bx = sqrtf(hx * hx + hy * hy);
          bz = R20 * mx + R21 * my + R22 * mz;
        }
        const float wx = R00 * bx + R20 * bz, wy = R01 * bx + R21 * bz, wz = R02 * bx + R22 * bz;
        const float emx = my * wz - mz * wy, emy = mz * wx - mx * wz, emz = mx * wy - my * wx;
        const float p = emx * vx + emy * vy + emz * vz;           // heading part
        emh0 = p * vx; emh1 = p * vy; emh2 = p * vz;
        const float k = kpMag * wm * (lost ? 4.0f : 1.0f);
        const float kt = useDip ? kpMagTilt * wm * (1.0f - wa) : 0.0f;
        ex += k * emh0 + kt * (emx - emh0);
        ey += k * emh1 + kt * (emy - emh1);
        ez += k * emh2 + kt * (emz - emh2);
      }
    }

    if (ki > 0 && !lost)                                      // bias only from trusted data
    {
      const float ga = ki * dt * (wa > biasWmin ? wa : 0.0f);
      const float gm = ki * dt * ((mag && wm > biasWmin) ? wm : 0.0f);
      ib[0] = clampf(ib[0] + ga * eax + gm * emh0, -biasLim, biasLim);
      ib[1] = clampf(ib[1] + ga * eay + gm * emh1, -biasLim, biasLim);
      ib[2] = clampf(ib[2] + ga * eaz + gm * emh2, -biasLim, biasLim);
    }

    gx += ex + ib[0]; gy += ey + ib[1]; gz += ez + ib[2];
    const float h = 0.5f * dt;
    float n0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * h;
    float n1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * h;
    float n2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * h;
    float n3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * h;
    const float n = 1.0f / sqrtf(n0 * n0 + n1 * n1 + n2 * n2 + n3 * n3);
    q[0] = n0 * n; q[1] = n1 * n; q[2] = n2 * n; q[3] = n3 * n;
  }

private:
  static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
};
