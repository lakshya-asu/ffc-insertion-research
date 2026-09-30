"""Collect the expert's training set: episodes with the observation vector, the expert's action, the phase and
the offline truth, written as float32 shards.

Two kinds of episode:
  clean:      the expert from reset to end.
  corrective: a perturbed controller (expert action plus noise, or a straight push) runs for a random number of
              ticks, then the expert takes over. Only the expert's ticks are marked as training targets, so the
              learner sees recoveries from states the expert alone would not visit.

usage: collect_expert.py --episodes 20000 --shard 1000 --procs 8 --estimator good --level L1 --corrective 0.3
"""
import argparse
import json
import sys
from multiprocessing import Pool
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.env import MAX_TICKS, TwinEnv, EstimatorConfig  # noqa: E402
from ffc_twin.expert import Expert  # noqa: E402

PHASES = {"align": 0, "approach": 1, "advance": 2, "retract": 3}


def _episode(env, ex, rng, corrective: bool):
    obs, info = env.reset(seed=int(rng.integers(2**31)), level=env.level.name)
    ex.reset()
    perturb_ticks = int(rng.integers(5, 40)) if corrective else 0
    mode = int(rng.integers(0, 2)) if corrective else -1  # 0: noisy expert, 1: straight push
    O, A, T, P, M, F = [], [], [], [], [], []
    for k in range(MAX_TICKS):
        a_exp = ex.act(obs)
        if k < perturb_ticks:
            a = np.clip(a_exp + rng.normal(0, 0.6, 6), -1, 1) if mode == 0 else np.array([1, 0, 0, 0, 0, 0.0])
            target = 0.0
        else:
            a = a_exp
            target = 1.0
        O.append(TwinEnv.vector(obs)); A.append(a_exp); P.append(PHASES.get(ex.phase, 4)); M.append(target)
        t = env.truth(); T.append([t["depth_m"], float(t["seated"]), t["force"]] + t["tip_pose"])
        obs, r, term, trunc, info = env.step(a)
        F.append(float(np.linalg.norm(obs["fixture_wrench_n_nm"][:3])))
        if term or trunc:
            break
    t = env.truth()
    return dict(obs=np.asarray(O, np.float32), act=np.asarray(A, np.float32), truth=np.asarray(T, np.float32), phase=np.asarray(P, np.int8),
                target=np.asarray(M, np.float32), force=np.asarray(F, np.float32),
                meta=dict(start=env.start, draws=env.draws, seated=bool(t["seated"]), peak=env.peak_force, ticks=len(O), corrective=corrective, perturb_ticks=perturb_ticks, mode=mode, retries=ex.retries))


def _shard(args):
    idx, n, seed, est_name, level, corrective_frac, out_dir, board, grip, dof = args
    from ffc_twin.spec import LEVELS, make_spec

    env = TwinEnv(spec=make_spec(board, grip, dof), estimator=EstimatorConfig.named(est_name), log=False); env.level = LEVELS[level]
    ex = Expert(env.spec); rng = np.random.default_rng(seed)
    eps = [_episode(env, ex, rng, rng.random() < corrective_frac) for _ in range(n)]
    lens = np.array([e["meta"]["ticks"] for e in eps]); offsets = np.r_[0, np.cumsum(lens)]
    np.savez_compressed(Path(out_dir) / f"shard_{idx:04d}.npz",
                        obs=np.concatenate([e["obs"] for e in eps]), act=np.concatenate([e["act"] for e in eps]), truth=np.concatenate([e["truth"] for e in eps]),
                        phase=np.concatenate([e["phase"] for e in eps]), target=np.concatenate([e["target"] for e in eps]), force=np.concatenate([e["force"] for e in eps]),
                        offsets=offsets)
    (Path(out_dir) / f"shard_{idx:04d}.json").write_text(json.dumps([e["meta"] for e in eps]))
    seated = sum(e["meta"]["seated"] for e in eps)
    return dict(shard=idx, episodes=n, ticks=int(lens.sum()), seated=int(seated))


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=20000); ap.add_argument("--shard", type=int, default=1000); ap.add_argument("--procs", type=int, default=8)
    ap.add_argument("--estimator", default="good"); ap.add_argument("--level", default="L1"); ap.add_argument("--corrective", type=float, default=0.3); ap.add_argument("--out", default="")
    ap.add_argument("--board", default="pi4", choices=["pi4", "zero"]); ap.add_argument("--grip", default="tape", choices=["tape", "body"]); ap.add_argument("--dof", type=int, default=6, choices=[4, 6])
    a = ap.parse_args()
    out = Path(a.out) if a.out else Path(__file__).resolve().parents[1] / "out" / f"dataset_expert_{a.board}_{a.grip}_{a.dof}dof_{a.estimator}_{a.level}"
    out.mkdir(parents=True, exist_ok=True)
    nshards = (a.episodes + a.shard - 1) // a.shard
    jobs = [(i, min(a.shard, a.episodes - i * a.shard), 7000 + i, a.estimator, a.level, a.corrective, str(out), a.board, a.grip, a.dof) for i in range(nshards)]
    with Pool(a.procs) as pool:
        stats = pool.map(_shard, jobs)
    manifest = dict(episodes=a.episodes, board=a.board, grip=a.grip, dof=a.dof, estimator=a.estimator, level=a.level, corrective_fraction=a.corrective, shards=stats,
                    obs_layout="tip_estimate(6) tip_visible(1) tip_sigma_scale(1) fixture_wrench_n_nm(6) tactile(4) robot_q_rad(6) last_action(6) t(1)",
                    truth_layout="depth_m seated force tip_pose(6)", target="1 where the recorded action is the expert's own move (training target), 0 during the perturbation")
    (out / "manifest.json").write_text(json.dumps(manifest, indent=1))
    print(json.dumps(dict(episodes=a.episodes, ticks=sum(s["ticks"] for s in stats), seated=sum(s["seated"] for s in stats), out=str(out))))
