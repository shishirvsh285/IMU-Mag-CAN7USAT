"""tune_sweep.py : grid-search adaptive Mahony settings on simulated descents (ground truth).
  python tune_sweep.py            -> prints tilt/heading rms per setting, best first"""
import itertools, pickle, os, sys
import numpy as np
from descent_sim import simulate
from test_adaptive import run, errors

SEEDS = int(sys.argv[1]) if len(sys.argv) > 1 else 3
cache = "sim_cache.pkl"
if os.path.exists(cache):
    sims = pickle.load(open(cache, "rb"))
else:
    sims = {(sc, s): simulate(sc, T=120, seed=100 + s, tumble_s=0 if sc == "bench" else 3.0)
            for sc in ["bench", "parachute", "parafoil"] for s in range(SEEDS)}
    pickle.dump(sims, open(cache, "wb"))

grid = {"drift_dps": [0.1, 0.3], "drift_scale": [0.003, 0.01], "tau_c": [0.1, 0.2, 0.5], "sigma0_deg": [0.5, 1.0, 2.0]}
res = []
for vals in itertools.product(*grid.values()):
    kw = dict(zip(grid, vals))
    row = {}
    for (sc, s), d in sims.items():
        kind = "adapt+GPS" if sc == "parafoil" else "adaptive"
        R, _ = run(d, kind, **kw)
        tilt, head = errors(R, d["R_true"])
        sel = d["t"] > d["tumble_s"] + 7
        row.setdefault(sc, []).append([np.sqrt(np.mean(tilt[sel] ** 2)), np.sqrt(np.mean(head[sel] ** 2))])
    m = {sc: np.mean(v, 0) for sc, v in row.items()}
    score = sum(m[sc][0] + 0.5 * m[sc][1] for sc in m)
    res.append((score, kw, m))
    print(f"{score:7.2f} {kw} " + " ".join(f"{sc}:{m[sc][0]:.2f}/{m[sc][1]:.2f}" for sc in m), flush=True)
print("\nBEST")
for score, kw, m in sorted(res, key=lambda r: r[0])[:6]:
    print(f"{score:7.2f} {kw} " + " ".join(f"{sc}: tilt {m[sc][0]:.2f} head {m[sc][1]:.2f}" for sc in m))
