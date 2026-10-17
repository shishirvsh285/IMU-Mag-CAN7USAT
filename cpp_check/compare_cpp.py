"""Run the same simulated flight through adaptive_mahony.py (double) and AdaptiveMahony.h (float)
and report the attitude difference."""
import pickle, subprocess, sys, os
import numpy as np
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from adaptive_mahony import AdaptiveMahony, GpsTurnAccel
from test_adaptive import quat_to_R, errors

here = os.path.dirname(os.path.abspath(__file__))
sims = pickle.load(open(os.path.join(here, "..", "sim_cache.pkl"), "rb"))
for key in [("parachute", 0), ("parafoil", 0), ("bench", 0)]:
    d = sims[key]; dt = d["dt"]; N = len(d["t"])
    gps = GpsTurnAccel(); aw = np.zeros((N, 3))
    for k in range(N):
        if k % 20 == 0:
            gps.fix(d["v_gps"][k][0], d["v_gps"][k][1], d["t"][k])
        aw[k] = gps.a
    f = AdaptiveMahony(mag_field=d["B_mG"], dip_deg=d["dip_deg"])
    Rp = np.zeros((N, 3, 3))
    for k in range(N):
        f.update(*d["gyro"][k], *d["acc"][k], dt, *d["mag"][k], a_world=aw[k]); Rp[k] = quat_to_R(f.q)
    inp = "\n".join(" ".join(f"{x:.9g}" for x in [dt, *d["gyro"][k], *d["acc"][k], *d["mag"][k], *aw[k]])
                    for k in range(N))
    out = subprocess.run([os.path.join(here, "run_filter"), str(d["B_mG"]), str(d["dip_deg"])],
                         input=inp, capture_output=True, text=True).stdout.split("\n")
    qc = np.array([[float(x) for x in l.split()[:4]] for l in out if l.strip()])
    Rc = np.array([quat_to_R(q) for q in qc])
    diff = np.degrees(np.arccos(np.clip((np.einsum("nij,nij->n", Rp, Rc) - 1) / 2, -1, 1)))
    tp, _ = errors(Rp, d["R_true"]); tc, _ = errors(Rc, d["R_true"])
    sel = d["t"] > 10
    print(f"{key[0]:<10} python vs C++ float: max attitude difference {diff.max():.3f} deg, "
          f"tilt rms python {np.sqrt(np.mean(tp[sel]**2)):.3f}, C++ {np.sqrt(np.mean(tc[sel]**2)):.3f}")
