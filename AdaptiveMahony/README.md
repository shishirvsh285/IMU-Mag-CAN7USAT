# AdaptiveMahony

An attitude filter for a CanSat under a parachute or parafoil. It is a Mahony filter that works out,
every sample, how much it can trust the accelerometer and the magnetometer, instead of trusting
them by a fixed amount.

- `AdaptiveMahony.h` : the filter as one Arduino/C++ header, float only.
- `../adaptive_mahony.py` : the same filter in Python, line for line (used for simulation and tuning).
- `../adaptive_mahony_uno/` : a ready sketch that runs it on the Uno with our sensors.

The two versions agree to within 0.35° on the same data (`../cpp_check/`).

---

## Why a normal Mahony filter struggles in descent

Mahony integrates the gyro, then corrects slow drift by pulling the estimate towards the
accelerometer, which it assumes measures gravity:

$$\boldsymbol\omega_{\text{used}} = \boldsymbol\omega_{\text{gyro}} + K_p\,\mathbf e + K_i\!\int\mathbf e\,dt, \qquad \mathbf e = \hat{\mathbf a}\times\hat{\mathbf v}$$

where $\hat{\mathbf a}$ is the measured accel direction and $\hat{\mathbf v}$ is where the filter thinks "up" is.

Under a canopy the payload is a pendulum. The accelerometer measures line tension, which always
points up the lines at roughly 1 g, whatever the swing angle. So the accelerometer keeps saying
"level" while the payload is tilted 20 to 30°. The usual guard, "ignore the accel when
$\big|\,|\mathbf a| - 1\,\big| > 0.15\,g$", does not catch it. A sideways push from a hand or a gust
fools it the same way.

The opposite mistake is just as bad. If we ignore the accelerometer completely, gyro drift is never
corrected. The filter has to tell "the gyro has drifted" apart from "the board is accelerating".

---

## How the adaptive version works

### 1. A one-state Kalman filter sets the gain

Two numbers are tracked:

- $P$: the variance of our own tilt error. It grows with gyro error and shrinks when we correct:
  $$\dot P = q - (K_p + K_{\text{avg}})\,P, \qquad q = \big(\text{drift} + \text{scale error}\cdot|\boldsymbol\omega|\big)^2$$
  Power-on and gyro saturation set $P = \pi^2$ ("no idea where we are").
- $R$: how wrong the accelerometer **direction** is right now (rad²).

The accelerometer gain is then

$$K_p = \text{clamp}\!\left(\frac{P}{R\,\tau_c},\; K_{\min},\; K_{\max}\right)$$

| Situation | P | R | K_p | Behaviour |
|---|---|---|---|---|
| Power-on, after a tumble | large | small | up to 5 | snaps to the accelerometer |
| Calm bench | small | small | about 1 | same as standard Mahony |
| Swinging under a canopy | small | large | 0.02 to 0.1 | trusts the gyro, averages over 10 to 50 s |

### 2. A second gravity reference that does not depend on the estimate

The accelerometer is averaged over $\tau_{\text{avg}}$ = 3 s and carried along with the gyro, which is
the same as averaging it in the world frame:

$$\dot{\mathbf f} = -\boldsymbol\omega\times\mathbf f + \frac{\mathbf a - \mathbf f}{\tau_{\text{avg}}}$$

Any real acceleration averages to roughly $\Delta v / \tau_{\text{avg}}$, which stays small because
speed is bounded. Gravity does not average away. This gives a clean way to tell the two problems apart:

| Instant accel vs average $\mathbf f$ | Instant accel vs our estimate | Meaning | Action |
|---|---|---|---|
| agree | disagree | our estimate drifted | trust the accel |
| disagree | anything | the board is being accelerated | distrust the accel |

### 3. How R is built

$$R = \sigma_0^2 \;+\; e_a^2 \;+\; (1 - w_{\text{dir}})\,\max\!\big(\overline{\theta_f^2},\, \sigma_{\text{dyn}}^2\big)$$

- $\sigma_0$: sensor noise and vibration (1°).
- $e_a$: $\big|\,|\mathbf a| - 1\,\big|$, the instant value or its recent rms, whichever is worse.
- $\theta_f$: angle between the instant accel and the average $\mathbf f$. $w_{\text{dir}}$ drops to 0 when it is
  bigger than the noise. $\overline{\theta_f^2}$ is its recent mean square.

$R$ jumps up at once and relaxes over 0.5 s, so trust is lost quickly and regained carefully.

### 4. Two corrections

$$\boldsymbol\omega_{\text{used}} = \boldsymbol\omega_{\text{gyro}} + K_p\,(\hat{\mathbf a}\times\hat{\mathbf v}) + K_{\text{avg}}\,(\hat{\mathbf f}\times\hat{\mathbf v}) + \text{mag term} + \mathbf b_i$$

with $K_{\text{avg}} = \min\!\big(P / (\sigma_{\text{avg}}^2\,\tau_{\text{avg}}),\; 0.5\big)$. Even when the instant
accelerometer is ignored, the averaged one keeps removing gyro drift.

### 5. Magnetometer: heading only, and only when it looks right

The magnetometer error is projected onto the vertical, so it can only turn the heading. A magnet near
the board can never tilt the estimate. Its weight $w_m$ drops when $|\mathbf B|$ differs from the
expected field, or when the dip angle (angle between field and gravity) is off.

### 6. Bias learning only from trusted data

The integral term $\mathbf b_i$ (gyro bias) only learns when $w_a$ or $w_m$ is above 0.5, and is limited to
±2 °/s. Shaking cannot teach it a fake bias.

### 7. Optional GPS turn correction

In a steady parafoil spiral the accelerometer is wrong by the bank angle for as long as the turn lasts.
`GpsTurnAccel` (Python) computes the turn acceleration from GPS velocity fixes,
$\mathbf a = \dot\chi\,\hat{\mathbf z}\times\mathbf v$, using the turn rate of the GPS **track**. Pass it as
`a_world` and it is subtracted before the tests. The gyro yaw rate is not used for this, because a
payload spinning on its swivel does not turn its velocity.

---

## Using it

### Arduino / C++

```cpp
#include "AdaptiveMahony.h"
AdaptiveMahony filt;

void setup() {
  filt.useDip   = true;     // only once the magnetometer is calibrated
  filt.dipDeg   = 26.0f;    // local dip angle (from full_calibration.py)
  filt.magField = 306.0f;   // local field, same unit as mag[] (0 = skip the |B| test)
}

void loop() {
  // gyro rad/s, accel g, mag any unit (already calibrated and in IMU axes), dt in s
  float mag[3] = {mx, my, mz};
  filt.update(gx, gy, gz, ax, ay, az, dt, mag);    // pass nullptr for 6-DOF
  float roll, pitch, yaw;
  filt.euler(roll, pitch, yaw);                     // degrees, ZYX
  // filt.q[0..3] is the quaternion (body to world), filt.kp the current accel gain
}
```

Conventions: world Z up, accel reads +1 g "up" when the board is still.

### Python

```python
from adaptive_mahony import AdaptiveMahony, GpsTurnAccel

f = AdaptiveMahony(mag_field=306.0, dip_deg=26.0)
f.update(gx, gy, gz, ax, ay, az, dt, mx, my, mz)   # mag optional
roll, pitch, yaw = f.euler_deg()
print(f.kp, f.wa, f.U)                             # gain, accel trust (0..1), tilt uncertainty (rad)
```

---

## Settings

Python names, with the C++ names in brackets where they differ. Defaults came from `../tune_sweep.py`
on simulated descents.

| Setting | Default | What it does |
|---|---|---|
| `kp_min`, `kp_max` | 0.02, 5.0 | Limits on the accelerometer gain (1/s) |
| `tau_c` (`tauC`) | 0.2 s | Correlation time of accel errors, scales the gain |
| `sigma0_deg` | 1° | Accel direction noise on a calm board |
| `sigma_dyn_deg` | 10° | Minimum assumed error when the direction test fails |
| `drift_dps` | 0.3 °/s | Gyro error growth when still |
| `drift_scale` | 0.003 | Extra gyro error per °/s of rotation (scale and cross-axis error) |
| `tau_avg` | 3 s | Averaging time of the gravity reference |
| `sigma_avg_deg` | 3° | Trust in the averaged reference |
| `kp_avg_max` | 0.5 | Limit on its gain |
| `tau_rise` | 0.5 s | How slowly trust comes back after a disturbance |
| `kp_mag` | 0.5 | Magnetometer heading gain |
| `mag_field`, `dip_deg` | none | Expected field and dip, for the magnetometer tests |
| `ki` | 0.02 | Bias learning rate |
| `gyro_range_dps` | 2000 | Saturation above 95 % of this resets the filter |

To try other values without the board, replay a log:
`python physical_test.py --replay phystest_x.csv --set tau_avg=5`.

---

## Cost

| Platform | Time per update | Memory |
|---|---|---|
| Arduino Uno (16 MHz AVR, software float) | 2.8 ms measured | 25 kB flash for the whole sketch, 629 B RAM |

At 50 Hz that is about 14 % of the CPU. Per update: about 5 square roots, 2 arc-cosines, 3
exponentials and 220 multiply/adds.

---

## Results

Simulated descents (`../descent_sim.py`, `../test_adaptive.py`), tilt error after deployment, rms, mean
of 5 runs that were not used for tuning:

| Scenario | Standard Mahony | Adaptive | Adaptive + GPS |
|---|---|---|---|
| Bench, slow hand tilts | 0.32° | 0.31° | 0.31° |
| Parachute: swing + 60 °/s spin + gusts | 9.4° | 4.5° | 4.5° |
| Parafoil: steady turns | 12.9° | 3.6° | 2.2° |
| Parachute + magnetic disturbances (max heading error) | 36.6° | 10.7° | 10.7° |

Real-board scores are still to come (see Known issues in the main README).

## Limits

- Gusts that push the canopy for a second or more change the accelerometer in a way that looks
  just like gravity, so no accelerometer-based filter can remove them.
- Without GPS, a long steady spiral still leaves a few degrees of error.
- Heading needs a calibrated magnetometer and the correct axis map (`../full_calibration.py`).
