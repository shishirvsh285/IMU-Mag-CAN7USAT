"""
test_adaptive.py : standard Mahony vs Adaptive Mahony on simulated descents with known truth.

  python test_adaptive.py                 -> all scenarios, 5 seeds each, table + plots
  python test_adaptive.py --seeds 1 --scenario parachute

Filters compared (same Kp for accel in both):
  standard  : mahony.py, kp=1, ki=0.05, accel skipped when | |a|-1 | > 0.15 g (as in mahony_live)
  adaptive  : adaptive_mahony.py, default settings
  adapt+GPS : same, plus GPS velocity (5 Hz) -> GpsTurnAccel -> turn acceleration removed
Errors: tilt = angle between true and estimated "up" in the body frame; heading = yaw error.
Statistics are taken after the deployment (t > tumble + 7 s).
"""
import argparse
import math

import matplotlib
import numpy as np

from adaptive_mahony import AdaptiveMahony, GpsTurnAccel
from descent_sim import simulate
from mahony import Mahony

SCEN = ["bench", "parachute", "parafoil", "magdist"]


def quat_to_R(q):
    q0, q1, q2, q3 = q
    return np.array([[1 - 2 * (q2 * q2 + q3 * q3), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
                     [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1 * q1 + q3 * q3), 2 * (q2 * q3 - q0 * q1)],
                     [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1 * q1 + q2 * q2)]])


def errors(R_est, R_true):
    up_e, up_t = R_est[:, 2, :], R_true[:, 2, :]                    # world up in body frame
    tilt = np.degrees(np.arccos(np.clip(np.sum(up_e * up_t, 1), -1, 1)))
    # heading: body X projected on the horizontal, compared
    xe, xt = R_est[:, :2, 0], R_true[:, :2, 0]
    he = np.degrees(np.arctan2(xe[:, 1], xe[:, 0]) - np.arctan2(xt[:, 1], xt[:, 0]))
    he = (he + 180) % 360 - 180
    return tilt, he


def run(d, kind, **kw):
    dt = d["dt"]
    N = len(d["t"])
    R = np.zeros((N, 3, 3))
    diag = np.zeros((N, 5))
    if kind == "standard":
        f = Mahony(kp=1.0, ki=0.05)
    else:
        params = dict(mag_field=d["B_mG"], dip_deg=d["dip_deg"])
        params.update(kw)
        f = AdaptiveMahony(**params)
        gps = GpsTurnAccel()
        gps_step = int(round(0.2 / dt))
    for k in range(N):
        g, a, m = d["gyro"][k], d["acc"][k], d["mag"][k]
        if kind == "standard":
            ok = abs(np.linalg.norm(a) - 1) < 0.15
            f.update(*g, *a, dt, *m, accel_ok=ok)
            diag[k] = [float(ok), 1, 0, 0, 1.0 * ok]
        else:
            aw = None
            if kind == "adapt+GPS":
                if k % gps_step == 0:                     # new 5 Hz GPS fix
                    gps.fix(d["v_gps"][k][0], d["v_gps"][k][1], d["t"][k])
                aw = gps.a
            f.update(*g, *a, dt, *m, a_world=aw)
            diag[k] = [f.wa, f.wm, math.degrees(f.theta), math.degrees(f.U), f.kp]
        R[k] = quat_to_R(f.q)
    return R, diag


def stats(x):
    x = np.abs(x)
    return np.sqrt(np.mean(x * x)), np.percentile(x, 95), x.max()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--scenario", default="all")
    ap.add_argument("--T", type=float, default=150)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    scen = SCEN if args.scenario == "all" else [args.scenario]
    kinds = ["standard", "adaptive", "adapt+GPS"]

    table = {}
    examples = {}
    for sc in scen:
        for seed in range(args.seeds):
            d = simulate(sc, T=args.T, seed=seed, tumble_s=0 if sc == "bench" else 3.0)
            sel = d["t"] > d["tumble_s"] + 7
            for kd in kinds:
                R, diag = run(d, kd)
                tilt, head = errors(R, d["R_true"])
                table.setdefault((sc, kd), []).append(stats(tilt[sel]) + stats(head[sel]))
                if seed == 0:
                    examples[(sc, kd)] = (d, tilt, head, diag)
            print(f"  {sc} seed {seed} done", flush=True)

    print("\nErrors after deployment, mean over seeds   (deg)")
    print(f"{'scenario':<11}{'filter':<11}{'tilt rms':>9}{'tilt p95':>9}{'tilt max':>9}"
          f"{'head rms':>10}{'head p95':>9}{'head max':>9}")
    for sc in scen:
        for kd in kinds:
            s = np.mean(table[(sc, kd)], axis=0)
            print(f"{sc:<11}{kd:<11}" + "".join(f"{v:9.2f}" for v in s[:3]) + " "
                  + "".join(f"{v:9.2f}" for v in s[3:]))
        print()

    if args.no_plot:
        return
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    col = {"standard": "#d1495b", "adaptive": "#2e86ab", "adapt+GPS": "#3bb273"}
    for sc in scen:
        d = examples[(sc, "standard")][0]
        t = d["t"]
        fig, ax = plt.subplots(4, 1, figsize=(11, 10), sharex=True)
        ax[0].plot(t, d["swing_deg"], color="#888", lw=0.8, label="true swing / tilt of the payload")
        ax[0].plot(t, np.linalg.norm(d["acc"], axis=1) * 10, color="#e0a030", lw=0.6, label="|a| x10 (g)")
        ax[0].set_ylabel("deg"); ax[0].legend(fontsize=8, loc="upper right"); ax[0].set_ylim(0, 45)
        for kd in ["standard", "adaptive", "adapt+GPS"]:
            _, tilt, head, _ = examples[(sc, kd)]
            ax[1].plot(t, tilt, color=col[kd], lw=0.8, label=kd)
            ax[2].plot(t, head, color=col[kd], lw=0.8, label=kd)
        ax[1].set_ylabel("tilt error, deg"); ax[1].legend(fontsize=8, loc="upper right")
        ax[1].set_ylim(0, 30)
        ax[2].set_ylabel("heading error, deg"); ax[2].set_ylim(-30, 30)
        diag = examples[(sc, "adaptive")][3]
        ax[3].semilogy(t, diag[:, 4], color="#2e86ab", lw=0.8, label="adaptive accel gain kp (1/s)")
        ax[3].semilogy(t, np.maximum(diag[:, 3], 0.01) / 10, color="#555", lw=0.8, ls="--",
                       label="tilt uncertainty U /10 (deg)")
        ax[3].axhline(1.0, color="#d1495b", lw=0.8, label="standard kp = 1")
        ax[3].set_ylim(0.01, 10); ax[3].legend(fontsize=8, loc="upper right")
        ax[3].set_xlabel("time, s")
        for a in ax:
            a.axvspan(0, d["tumble_s"], color="#ddd", alpha=0.5)
            a.grid(alpha=0.3)
        fig.suptitle(f"{sc}: standard vs adaptive Mahony (seed 0)")
        fig.tight_layout()
        fig.savefig(f"adaptive_test_{sc}.png", dpi=120)
        plt.close(fig)
        print("saved", f"adaptive_test_{sc}.png")


if __name__ == "__main__":
    main()
