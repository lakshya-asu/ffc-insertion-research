"""Capture region of the insertion for rigid pick skews: seat rate vs skew angle, per controller.

usage: python scripts/capture_region.py [--n 6] [--skews 0,2,4,6,8] [--controllers push,align]
writes out/capture_region.json

v1 (spec A4, A6): python scripts/capture_region.py --v1 grid --controllers push,align,expert --n 40 \
        --slips 0,1,2,3,4,5 --tilts 0,2,4,6,8 --procs 8 --out capture_v1.json
    python scripts/capture_region.py --v1 stage --stage 2 --controllers expert --n 400 --procs 2 --out stage2_expert.json
Every episode runs the hand-built seat detector beside the controller (one step per 30 Hz tick) and grades it
against Bench.seated() at the tick it fires (FP) and at the end (FN). Truth is read here only, for grading.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin.bench import Bench, PickError  # noqa: E402
from cell_twin import controllers as C  # noqa: E402

EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
START_EP = "09710851-e26c-43ef-b7ed-0a684c28e4a4"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=6)
    ap.add_argument("--skews", default="0,2,4,6,8")
    ap.add_argument("--controllers", default="push,align")
    ap.add_argument("--laterals", default=None, help="sweep systematic lateral offset (mm) at zero skew instead of skews")
    a = ap.parse_args()
    from scripts.calibrate import episode_offsets
    seated = np.load(EPS / START_EP / "frames.npz")["observation.state"][-1]
    start = seated
    offsets = episode_offsets(seated)
    rng = np.random.default_rng(7)
    results = {}
    for name in a.controllers.split(","):
        fn = C.push_only if name == "push" else C.align_by_contact
        results[name] = {}
        sweep = [float(s) for s in (a.laterals or a.skews).split(",")]
        for skew in sweep:
            seats = 0
            for k in range(a.n):
                if a.laterals:
                    err = PickError(skew_deg=0.0, lateral_mm=(0.0, skew * rng.choice([-1.0, 1.0])))   # slip along the row
                else:
                    err = PickError(skew_deg=skew * rng.choice([-1.0, 1.0]), skew_dir_deg=0.0,          # about the pinch axis
                                    lateral_mm=(0.0, float(rng.normal(0, 1.0))))
                b = Bench(start, err, rng=np.random.default_rng(100 + k), part_offsets=offsets)
                ok = fn(b) if name == "push" else fn(b, rng=np.random.default_rng(200 + k))
                seats += int(ok)
                print(f"{name:6s} skew {skew:4.1f} deg  trial {k}: {'seat' if ok else 'no'}  depth {b.seat_depth()*1000:6.2f} mm", flush=True)
            results[name][skew] = seats / a.n
            print(f"== {name} skew {skew} deg: {seats}/{a.n}", flush=True)
    tag = "lateral" if a.laterals else "skew"
    json.dump(results, open(ROOT / "out" / f"capture_region_{tag}.json", "w"), indent=1)
    print(json.dumps(results, indent=1))


def wilson(k, n, z=1.96):
    if n == 0:
        return [0.0, 0.0, 1.0]
    p = k / n; den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den; h = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return [round(p, 4), round(max(0.0, c - h), 4), round(min(1.0, c + h), 4)]


def run_episode(job):
    """One episode: job = (controller, seed, tilt_deg or None, slip_mm or None, stage). Stage draws the full
    Randomisation; a grid cell keeps the nominal bench and sets only the header pick error."""
    import time
    from cell_twin.bench import Bench, pick_errors
    from cell_twin.training_config import Randomisation
    name, seed, tilt, slip, stage, *extra = job
    weld, logdir, ablate = (extra + [None, None, None])[:3]
    if weld:
        from cell_twin import scene as S
        S.GRASP_SOLREF = (float(weld), 1.0)
    rng = np.random.default_rng(seed)
    r = Randomisation.sample(rng, stage=stage) if tilt is None else Randomisation()
    if ablate:
        r = _ablate(r, ablate)
    if tilt is not None:
        r.header_tilt_deg = tilt * rng.choice([-1.0, 1.0]); r.header_slip_mm = slip * rng.choice([-1.0, 1.0])
    start = np.load(EPS / START_EP / "frames.npz")["observation.state"][-1]
    t0 = time.time()
    try:
        h, f = pick_errors(r)
        b = Bench(start, h, f, rng=np.random.default_rng(seed + 1), rand=r)
        det = C.SeatDetector(b); fire = {"tick": None, "seated_then": None}; ticks = [0]
        cmd = b._command
        log = {k: [] for k in ("tau", "travel", "seated", "depth", "act")}
        def tick():
            cmd(); ticks[0] += 1
            if logdir:
                log["tau"].append(b.torques()); log["travel"].append(b.tool_travel())
                log["seated"].append(b.seated()); log["depth"].append(b.seat_depth())
            if det.step() and fire["tick"] is None:
                fire["tick"] = ticks[0]; fire["seated_then"] = bool(b.seated())
        b._command = tick
        if name == "push":
            C.push_only(b)
        elif name == "align":
            C.align_by_contact(b, rng=np.random.default_rng(seed + 2))
        else:
            pol = C.Expert(np.random.default_rng(seed + 2))
            if logdir:
                act = pol.act
                def logged_act(bb):
                    a_ = act(bb); log["act"].append(a_.copy()); return a_
                pol.act = logged_act
            C.expert(b, policy=pol, detector=_NullDetector())
        out = b.outcome()
        rec = dict(outcome=out, depth_mm=round(b.seat_depth() * 1000, 3), ticks=ticks[0], fired_tick=fire["tick"],
                   fired_seated=fire["seated_then"], seated_end=bool(b.seated()))
    except Exception as e:  # noqa: BLE001  (an exception is outcome X, recorded, never hidden)
        rec = dict(outcome="X", error=repr(e)[:200], ticks=None, fired_tick=None, fired_seated=None, seated_end=False)
    if logdir and rec["outcome"] != "X":
        Path(logdir).mkdir(parents=True, exist_ok=True)
        np.savez_compressed(Path(logdir) / f"{name}_{ablate or 'full'}_{seed}.npz", tau=np.array(log["tau"]), travel=np.array(log["travel"]),
                            seated=np.array(log["seated"]), depth=np.array(log["depth"]), act=np.array(log["act"]),
                            outcome=rec["outcome"])
    rec.update(weld=weld, ablate=ablate, controller=name, seed=seed, tilt=tilt, slip=slip, stage=stage, rand=r.to_dict(), secs=round(time.time() - t0, 2))
    return rec


ABLATE_GROUPS = {   # group -> fields kept from the draw; every other field is set to the nominal bench
    "latency": ("obs_latency_ticks", "act_latency_ticks"),
    "gains": ("kp_scale", "kd_scale"),
    "sensing": ("torque_scale", "torque_bias", "torque_noise_std"),
    "contact": ("contact_stiffness_scale",),
    "female": ("female_tilt_deg",),
    "friction": ("part_friction",),
    "none": (),
}


def _ablate(r, group):
    """Keep the pick error and the named group(s) ('a+b') from the draw, nominal everything else."""
    from cell_twin.training_config import Randomisation
    if group == "all":
        return r
    n = Randomisation()
    n.header_tilt_deg, n.header_slip_mm, n.curriculum_stage = r.header_tilt_deg, r.header_slip_mm, r.curriculum_stage
    for g in group.split("+"):
        for f in ABLATE_GROUPS[g]:
            setattr(n, f, getattr(r, f))
    return n


class _NullDetector:
    def step(self):
        return False


def summarise(recs):
    n = len(recs); seats = sum(r["outcome"] == "S" for r in recs)
    neg = [r for r in recs if not r["seated_end"]]; pos = [r for r in recs if r["seated_end"]]
    fp = sum(r["fired_tick"] is not None and not r["fired_seated"] for r in recs)
    fp_terminal = sum(r["fired_tick"] is not None for r in neg)
    fn = sum(r["fired_tick"] is None for r in pos)
    by = {}
    for r in recs:
        d = by.setdefault(r["outcome"], dict(n=0, fired=0)); d["n"] += 1; d["fired"] += r["fired_tick"] is not None
    return dict(n=n, seat=wilson(seats, n), detector_fp_fired_while_unseated=wilson(fp, n),
                detector_fp_terminal_unseated=wilson(fp_terminal, len(neg)), detector_fn=wilson(fn, len(pos)),
                by_outcome=by, ticks_mean=float(np.mean([r["ticks"] or 0 for r in recs])))


def main_v1():
    import multiprocessing as mp
    ap = argparse.ArgumentParser()
    ap.add_argument("--v1", choices=("grid", "stage"))
    ap.add_argument("--controllers", default="expert")
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--slips", default="0,1,2,3,4,5")
    ap.add_argument("--tilts", default="0,2,4,6,8")
    ap.add_argument("--stage", type=int, default=2)
    ap.add_argument("--procs", type=int, default=2)
    ap.add_argument("--out", default="capture_v1.json")
    ap.add_argument("--weld", default=None, help="grasp weld solref time constant (s), default scene.GRASP_SOLREF")
    ap.add_argument("--log-ticks", action="store_true", help="save per-tick sensor logs to out/ticklogs/<out stem>/")
    ap.add_argument("--ablate", default=None, help="stage mode: comma list of randomisation groups (a+b combines)")
    a = ap.parse_args()
    jobs = []
    logdir = str(ROOT / "out" / "ticklogs" / Path(a.out).stem) if a.log_ticks else None
    for name in a.controllers.split(","):
        if a.v1 == "grid":
            for s in map(float, a.slips.split(",")):
                for t in map(float, a.tilts.split(",")):
                    jobs += [(name, 10000 * int(s * 10 + 1) + 100 * int(t) + k, t, s, 0, a.weld, logdir, None) for k in range(a.n)]
        else:
            for g in (a.ablate.split(",") if a.ablate else [None]):
                jobs += [(name, 500000 + k, None, None, a.stage, a.weld, logdir, g) for k in range(a.n)]
    with mp.get_context("spawn").Pool(min(a.procs, 8)) as pool:
        recs = []
        for i, r in enumerate(pool.imap_unordered(run_episode, jobs, chunksize=1)):
            recs.append(r)
            if (i + 1) % 20 == 0:
                print(f"{i + 1}/{len(jobs)} done", flush=True)
    res = {"args": vars(a), "summary": {}, "cells": {}, "episodes": recs}
    for name in a.controllers.split(","):
        mine = [r for r in recs if r["controller"] == name]
        res["summary"][name] = summarise(mine)
        if a.ablate:
            res.setdefault("by_group", {})[name] = {g: summarise([r for r in mine if r["ablate"] == g]) for g in a.ablate.split(",")}
            for g, v in res["by_group"][name].items():
                print(name, g, "seat", v["seat"], v["by_outcome"])
        if a.v1 == "grid":
            res["cells"][name] = {f"slip{s}_tilt{t}": wilson(sum(r["outcome"] == "S" for r in mine if r["slip"] == s and r["tilt"] == t),
                                                              sum(1 for r in mine if r["slip"] == s and r["tilt"] == t))
                                  for s in map(float, a.slips.split(",")) for t in map(float, a.tilts.split(","))}
        print(name, json.dumps(res["summary"][name]))
        if a.v1 == "grid":
            print(name, json.dumps(res["cells"][name]))
    json.dump(res, open(ROOT / "out" / a.out, "w"), indent=1)
    print("wrote out/" + a.out)


if __name__ == "__main__":
    if "--v1" in sys.argv:
        main_v1()
    else:
        main()
