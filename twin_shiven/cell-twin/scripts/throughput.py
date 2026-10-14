"""Bench throughput with full contact: expert episodes over the stage-2 randomisation, per process count.

usage: python scripts/throughput.py [--procs 1,4,8] [--episodes-per-proc 10]
Reports episodes per minute (per process and total) and simulated 30 Hz ticks per second, and model build time
(each episode compiles its own model because the pick error is compiled in). Writes out/throughput.json.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
START_EP = "09710851-e26c-43ef-b7ed-0a684c28e4a4"


def worker(args):
    seed, n, stage = args
    from cell_twin import controllers as C
    from cell_twin.bench import Bench, pick_errors
    from cell_twin.training_config import Randomisation
    start = np.load(EPS / START_EP / "frames.npz")["observation.state"][-1]
    ticks = build = 0.0; t0 = time.time(); outcomes = []
    for k in range(n):
        r = Randomisation.sample(np.random.default_rng(seed + k), stage=stage)
        h, f = pick_errors(r)
        tb = time.time()
        b = Bench(start, h, f, rng=np.random.default_rng(seed + k), rand=r)
        build += time.time() - tb
        acts, _, _ = C.expert(b, policy=C.Expert(np.random.default_rng(seed + k)))
        ticks += len(acts); outcomes.append(b.outcome())
    return dict(secs=time.time() - t0, build_secs=build, ticks=ticks, episodes=n, outcomes=outcomes)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--procs", default="1,4,8")
    ap.add_argument("--episodes-per-proc", type=int, default=10)
    ap.add_argument("--stage", type=int, default=2)
    a = ap.parse_args()
    res = {}
    for p in map(int, a.procs.split(",")):
        jobs = [(900000 + 1000 * i, a.episodes_per_proc, a.stage) for i in range(p)]
        t0 = time.time()
        with mp.get_context("spawn").Pool(p) as pool:
            out = pool.map(worker, jobs)
        wall = time.time() - t0
        eps = sum(o["episodes"] for o in out); ticks = sum(o["ticks"] for o in out)
        res[p] = dict(wall_s=round(wall, 1), episodes=eps, ticks=int(ticks),
                      episodes_per_min_total=round(eps / wall * 60, 2),
                      episodes_per_min_per_proc=round(eps / wall * 60 / p, 2),
                      ticks_per_s_per_proc=round(np.mean([o["ticks"] / o["secs"] for o in out]), 1),
                      realtime_factor_per_proc=round(np.mean([o["ticks"] / o["secs"] for o in out]) / 30, 2),
                      build_s_per_episode=round(sum(o["build_secs"] for o in out) / eps, 2),
                      mean_ticks_per_episode=round(ticks / eps, 1),
                      seat_rate=round(sum(x == "S" for o in out for x in o["outcomes"]) / eps, 3))
        print(p, "procs:", json.dumps(res[p]), flush=True)
    (ROOT / "out").mkdir(exist_ok=True)
    json.dump(res, open(ROOT / "out" / "throughput.json", "w"), indent=1)


if __name__ == "__main__":
    main()
