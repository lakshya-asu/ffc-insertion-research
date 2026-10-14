"""Collect expert episodes on the randomised bench (spec B3).

usage: python scripts/collect_expert.py --n 15000 --procs 8 --stage 2 --noise 0.3 [--controller expert] [--run NAME]
                                         [--frames K] [--seed0 0]

One worker process per --procs, each with its own InsertionEnv. Episode i uses seed seed0 + i, so a run is
reproducible and resumable: episodes whose header json already exists are skipped. The controller is driven through
env.ExpertProxy (bounded ticks, DART noise of --noise action units on every tick, the clean action recorded).
--controller expert uses controllers.expert when it exists; 'align' and 'push' are the existing controllers.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing as mp
import os
import sys
import time
import traceback
import warnings
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

from cell_twin import controllers as C  # noqa: E402
from cell_twin.env import InsertionEnv, run_controller, run_policy  # noqa: E402
from cell_twin.episode_writer import EpisodeWriter, git_hash  # noqa: E402
from cell_twin.training_config import Randomisation  # noqa: E402


def controller_by_name(name: str):
    if name == "expert":
        if not hasattr(C, "Expert"):
            raise SystemExit("controllers.Expert does not exist on this branch; use --controller align")
        return C.Expert, "policy"
    if name == "align":
        return C.align_by_contact, True
    if name == "push":
        return C.push_only, False
    raise SystemExit(f"unknown controller {name}")


def seat_detector():
    return getattr(C, "SeatDetector", False)


_G: dict = {}


def _init(args):
    warnings.filterwarnings("ignore")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    _G["args"] = args
    _G["env"] = InsertionEnv(seat_detector=seat_detector())
    _G["writer"] = EpisodeWriter(ROOT / "out" / "episodes" / args.run, frames_every=args.frames)
    if args.frames:
        _G["env"].on_tick = _G["writer"].on_tick
    _G["git"] = git_hash()


def run_one(i: int):
    a = _G["args"]; env: InsertionEnv = _G["env"]; w: EpisodeWriter = _G["writer"]
    seed = a.seed0 + i
    ep_id = f"ep{seed:07d}"
    if w.exists(ep_id):
        return ep_id, "skip", 0.0, 0
    t0 = time.time()
    fn, takes_rng = controller_by_name(a.controller)
    rng = np.random.default_rng(seed)
    rand = Randomisation.sample(rng, a.stage)
    header = dict(controller=a.controller, noise_sigma=a.noise, bench_git=_G["git"], run=a.run,
                  seat_detector="controllers.SeatDetector" if seat_detector() else "env stand-in (stall rule)")
    try:
        env.reset(seed=seed, randomisation=rand, curriculum_stage=a.stage)
        nrng = np.random.default_rng(seed + 20_000_000)
        if takes_rng == "policy":
            clean = run_policy(env, fn(np.random.default_rng(seed + 10_000_000)), noise_sigma=a.noise, noise_rng=nrng)
        else:
            kw = {"rng": np.random.default_rng(seed + 10_000_000)} if takes_rng else {}
            clean = run_controller(env, fn, noise_sigma=a.noise, noise_rng=nrng, **kw).clean_actions
        w.write(ep_id, env, header, clean_actions=clean)
        out = env.truth()["outcome"]
    except Exception:
        out = "X"
        (w.run_dir / f"{ep_id}.json").write_text(json.dumps(dict(header, id=ep_id, seed=seed, end_reason="exception",
                                                                  randomisation=rand.to_dict(), error=traceback.format_exc()[-2000:])))
        np.savez(w.run_dir / f"{ep_id}.truth.npz", outcome=np.array("X"))
    return ep_id, out, time.time() - t0, env.t


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=15000)
    ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--stage", type=int, default=2)
    ap.add_argument("--noise", type=float, default=0.3)
    ap.add_argument("--controller", default="expert")
    ap.add_argument("--run", default=None)
    ap.add_argument("--frames", type=int, default=0)
    ap.add_argument("--seed0", type=int, default=0)
    a = ap.parse_args()
    controller_by_name(a.controller)
    a.run = a.run or f"{a.controller}_s{a.stage}_n{a.noise}"
    t0 = time.time(); counts: dict = {}; ticks = 0
    with mp.get_context("spawn").Pool(a.procs, initializer=_init, initargs=(a,)) as pool:
        for k, (ep_id, out, dt, t) in enumerate(pool.imap_unordered(run_one, range(a.n), chunksize=1)):
            counts[out] = counts.get(out, 0) + 1; ticks += t
            if (k + 1) % 25 == 0 or k + 1 == a.n:
                el = time.time() - t0
                print(f"[{k + 1}/{a.n}] {el:.0f}s {60 * (k + 1) / el:.1f} ep/min ticks {ticks} {counts}", flush=True)
    summary = dict(run=a.run, n=a.n, procs=a.procs, stage=a.stage, noise=a.noise, controller=a.controller,
                   wall_s=round(time.time() - t0, 1), outcomes=counts, ticks=ticks)
    (ROOT / "out" / "episodes" / a.run / "_collect_summary.json").write_text(json.dumps(summary, indent=1))
    print(json.dumps(summary))


if __name__ == "__main__":
    main()
