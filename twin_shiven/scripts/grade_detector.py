"""Grade the observable seating detector against simulator truth on a seeded mix of outcomes.

Episodes: the expert on random L1 starts (successes and failures), plus deliberate partial insertions
(stop early) and forced pushes into jams. Reports false-positive and false-negative rates.
"""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.env import TwinEnv, EstimatorConfig  # noqa: E402
from ffc_twin.expert import Expert  # noqa: E402
from ffc_twin.stats import wilson  # noqa: E402


def run_expert(env, seed, level="L1", max_ticks=150):
    obs, _ = env.reset(seed=seed, level=level)
    ex = Expert(env.spec)
    for _ in range(max_ticks):
        obs, r, term, trunc, info = env.step(ex.act(obs))
        if term or trunc:
            break
    return env.detector_seated(), env.truth()["seated"]


def run_partial(env, seed, stop_depth):
    """Push straight in with no correction and stop at a chosen depth: a partial insertion."""
    obs, _ = env.reset(seed=seed, level="L0", start=dict(lateral=0.0, height=0.0, yaw=0.0, pitch=0.0, roll=0.0))
    for _ in range(150):
        obs, r, term, trunc, info = env.step(np.array([1, 0, 0, 0, 0, 0.0]))
        if obs["tip_estimate"][0] > stop_depth or term or trunc:
            break
    return env.detector_seated(), env.truth()["seated"]


def run_jam(env, seed):
    obs, _ = env.reset(seed=seed, level="L0", start=dict(lateral=0.35e-3, height=0.0, yaw=0.0, pitch=0.0, roll=0.0))
    for _ in range(60):
        obs, r, term, trunc, info = env.step(np.array([1, 0, 0, 0, 0, 0.0]))
        if term or trunc:
            break
    return env.detector_seated(), env.truth()["seated"]


if __name__ == "__main__":
    from ffc_twin.spec import make_spec

    n = int(sys.argv[1]) if len(sys.argv) > 1 else 60
    board = sys.argv[2] if len(sys.argv) > 2 else "pi4"
    presets = sys.argv[3].split(",") if len(sys.argv) > 3 else ["good", "poor"]
    results = []
    for cfg_name in presets:
        env = TwinEnv(spec=make_spec(board), estimator=EstimatorConfig.named(cfg_name), log=False)
        seat = env.spec.connector.seat_depth.value
        rows = []
        for s in range(n):
            rows.append(("expert", *run_expert(env, s)))
        for s in range(n // 3):
            rows.append(("partial", *run_partial(env, 1000 + s, 0.43 * seat)))   # stop well short
            rows.append(("partial", *run_partial(env, 2000 + s, 0.81 * seat)))   # stop just short of the tolerance band
            rows.append(("jam", *run_jam(env, 3000 + s)))
        fp = sum(1 for _, d, t in rows if d and not t); fn = sum(1 for _, d, t in rows if t and not d)
        pos = sum(1 for _, d, t in rows if t); neg = len(rows) - pos
        r = dict(board=board, estimator=cfg_name, episodes=len(rows), truth_positive=pos, truth_negative=neg, false_positive=fp, false_negative=fn,
                 fp_rate=wilson(fp, neg), fn_rate=wilson(fn, pos), by_kind={k: [sum(1 for kk, d, t in rows if kk == k and d), sum(1 for kk, d, t in rows if kk == k and t), sum(1 for kk, *_ in rows if kk == k)] for k in ("expert", "partial", "jam")})
        results.append(r)
        print(json.dumps(r))
    out = Path(__file__).resolve().parents[1] / "out"; out.mkdir(exist_ok=True)
    (out / ("detector_grade.json" if board == "pi4" else f"detector_grade_{board}.json")).write_text(json.dumps(results, indent=2))
