"""Truth-graded evaluation of a policy on the capture grid and on the stage 3 range (spec B6).

usage: python scripts/eval_policy.py --policy expert|align|push_only|bc [--model out/models/bc_v1.pt]
                                     [--grid-n 40] [--stage3-n 1200] [--procs 4] [--grid-dr full|nominal] [--tag T]

Grid: header slip {0,1,2,3,4,5} mm x header tilt {0,2,4,6,8} deg (agent A's A6 grid), each with a random sign per
episode. --grid-dr full draws every other randomised quantity from Randomisation.sample (stage 3) and overrides the
pick error; nominal uses the default Randomisation. Stage 3: Randomisation.sample(rng, 3), --stage3-n episodes.
Seat rate = Bench.outcome() == 'S' after the episode (truth, offline). Wilson 95 percent intervals.
Also reported: the seat detector's false-positive rate on this policy's own rollouts (fired, truth not seated).
Writes out/eval_<policy>[_<tag>].json.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time
import warnings
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from cell_twin.env import InsertionEnv, run_controller, run_policy  # noqa: E402
from cell_twin.training_config import Randomisation  # noqa: E402

SLIPS = [0, 1, 2, 3, 4, 5]
TILTS = [0, 2, 4, 6, 8]
_G: dict = {}


def wilson(k, n, z=1.96):
    if n == 0:
        return [None, None, None]
    p = k / n; d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d; h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(p, 4), round(max(0.0, c - h), 4), round(min(1.0, c + h), 4)]


def _init(a):
    warnings.filterwarnings("ignore")
    os.environ["OMP_NUM_THREADS"] = "1"
    sys.path.insert(0, str(ROOT))
    from scripts.collect_expert import seat_detector
    _G["a"] = a
    _G["env"] = InsertionEnv(seat_detector=seat_detector())
    if a.policy == "bc":
        import torch
        from cell_twin import models as M
        torch.set_num_threads(1)
        _G["model"] = M.load(a.model); _G["torch"] = torch


def run_one(job):
    kind, cell, seed = job
    a = _G["a"]; env: InsertionEnv = _G["env"]
    rng = np.random.default_rng(seed)
    if kind == "grid":
        slip, tilt = cell
        r = Randomisation.sample(rng, 3) if a.grid_dr == "full" else Randomisation()
        r.header_slip_mm = float(slip) * float(rng.choice([-1, 1]))
        r.header_tilt_deg = float(tilt) * float(rng.choice([-1, 1]))
        stage = 3
    else:
        r = Randomisation.sample(rng, 3); stage = 3
    t0 = time.time()
    try:
        obs, _ = env.reset(seed=seed, randomisation=r, curriculum_stage=stage)
        if a.policy == "bc":
            torch = _G["torch"]; model = _G["model"]
            done = False
            while not done:
                with torch.no_grad():
                    act = model(torch.from_numpy(obs)[None]).numpy()[0]
                obs, _, term, trunc, _ = env.step(act)
                done = term or trunc
        else:
            from cell_twin import controllers as C
            from scripts.collect_expert import controller_by_name
            name = {"push_only": "push"}.get(a.policy, a.policy)
            fn, takes_rng = controller_by_name(name)
            if takes_rng == "policy":
                run_policy(env, fn(np.random.default_rng(seed + 10_000_000)))
            else:
                kw = {"rng": np.random.default_rng(seed + 10_000_000)} if takes_rng else {}
                run_controller(env, fn, **kw)
        tr = env.truth()
        out = tr["outcome"]
        det_fp = bool(np.asarray(env.log["detector"]).any() and out != "S")
        return dict(kind=kind, cell=cell, seed=seed, outcome=out, end=env.end_reason, ticks=env.t, det_fp=det_fp,
                    depth=tr["depth"], wall=time.time() - t0)
    except Exception as e:  # noqa: BLE001
        return dict(kind=kind, cell=cell, seed=seed, outcome="X", end=f"exception {e!r}"[:200], ticks=0, det_fp=False,
                    depth=None, wall=time.time() - t0)


def summarise(res):
    out = {}
    n = len(res); k = sum(r["outcome"] == "S" for r in res)
    out["n"] = n; out["seat"] = wilson(k, n)
    out["outcomes"] = {o: sum(r["outcome"] == o for r in res) for o in ("S", "F-partial", "F-jam", "F-miss", "X")}
    out["end_reasons"] = {}
    for r in res:
        key = r["end"].split(" ")[0]; out["end_reasons"][key] = out["end_reasons"].get(key, 0) + 1
    fired = [r for r in res if r["det_fp"] or (r["end"] == "seat_detector")]
    out["detector_fp_on_rollouts"] = wilson(sum(r["det_fp"] for r in res), max(1, sum(r["outcome"] != "S" for r in res)))
    out["detector_fired"] = len(fired)
    out["mean_ticks"] = float(np.mean([r["ticks"] for r in res])) if res else None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", required=True, choices=["expert", "align", "push_only", "bc"])
    ap.add_argument("--model", default=str(ROOT / "out" / "models" / "bc_v1.pt"))
    ap.add_argument("--grid-n", type=int, default=40)
    ap.add_argument("--stage3-n", type=int, default=1200)
    ap.add_argument("--procs", type=int, default=4)
    ap.add_argument("--grid-dr", default="full", choices=["full", "nominal"])
    ap.add_argument("--seed0", type=int, default=5_000_000)
    ap.add_argument("--tag", default="")
    a = ap.parse_args()
    jobs = []
    s = a.seed0
    for slip in SLIPS:
        for tilt in TILTS:
            for _ in range(a.grid_n):
                jobs.append(("grid", (slip, tilt), s)); s += 1
    for _ in range(a.stage3_n):
        jobs.append(("stage3", None, s)); s += 1
    t0 = time.time(); res = []
    with mp.get_context("spawn").Pool(a.procs, initializer=_init, initargs=(a,)) as pool:
        for k, r in enumerate(pool.imap_unordered(run_one, jobs, chunksize=1)):
            res.append(r)
            if (k + 1) % 50 == 0 or k + 1 == len(jobs):
                el = time.time() - t0
                print(f"[{k + 1}/{len(jobs)}] {el:.0f}s seat so far {sum(x['outcome'] == 'S' for x in res)}", flush=True)
    grid = [r for r in res if r["kind"] == "grid"]; st3 = [r for r in res if r["kind"] == "stage3"]
    cells = {}
    for slip in SLIPS:
        for tilt in TILTS:
            rr = [r for r in grid if tuple(r["cell"]) == (slip, tilt)]
            cells[f"slip{slip}_tilt{tilt}"] = dict(n=len(rr), seat=wilson(sum(r["outcome"] == "S" for r in rr), len(rr)))
    rep = dict(policy=a.policy, model=a.model if a.policy == "bc" else None, grid_dr=a.grid_dr, grid_n=a.grid_n,
               stage3_n=a.stage3_n, procs=a.procs, wall_s=round(time.time() - t0, 1),
               grid=summarise(grid), grid_cells=cells, stage3=summarise(st3),
               seat_detector=("controllers.SeatDetector" if _has_detector() else "env stand-in (stall rule)"))
    name = f"eval_{a.policy}" + (f"_{a.tag}" if a.tag else "")
    json.dump(rep, open(ROOT / "out" / f"{name}.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "grid_cells"}))


def _has_detector():
    from cell_twin import controllers as C
    return hasattr(C, "SeatDetector")


if __name__ == "__main__":
    main()
