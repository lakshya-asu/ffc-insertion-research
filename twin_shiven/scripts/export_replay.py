"""Replay a recorded twin episode as a USD timeline for Isaac Sim's renderer: every geom becomes a cube with
time-sampled world poses at the control rate, physics disabled. Isaac opens it and renders with RTX; nothing is
simulated there. Mirrors Lakshya's recorded-state replay packages (experiments 039 and 040).

usage: export_replay.py [--board zero] [--grip tape] [--dof 6] [--seed 3] [--level L1] [--estimator good] --out out/replay_zero_seed3.usda
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ffc_twin.env import EstimatorConfig, TwinEnv  # noqa: E402
from ffc_twin.expert import Expert  # noqa: E402
from ffc_twin.spec import make_spec  # noqa: E402


def run_episode(board, grip, dof, seed, level, estimator):
    env = TwinEnv(spec=make_spec(board, grip, dof), estimator=EstimatorConfig.named(estimator), log=True, log_geoms=True)
    obs, _ = env.reset(seed=seed, level=level)
    ex = Expert(env.spec)
    for _ in range(150):
        obs, r, term, trunc, info = env.step(ex.act(obs))
        if term or trunc:
            break
    return env.episode_record(dict(controller="expert", seed=seed))


def _quat_from_mat(R):
    # w, x, y, z from a row-major 3x3
    R = np.asarray(R).reshape(3, 3)
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return (0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return ((R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s)


def write_usd(episode: dict, path: Path) -> Path:
    from pxr import Gf, Sdf, Usd, UsdGeom

    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    fps = episode["control_hz"]
    n = len(episode["records"])
    stage.SetStartTimeCode(0); stage.SetEndTimeCode(n - 1); stage.SetTimeCodesPerSecond(fps)
    root = UsdGeom.Xform.Define(stage, "/Replay")
    stage.SetDefaultPrim(root.GetPrim())
    meta = dict(board=episode["board"], level=episode["level"], seed=episode.get("seed"), controller=episode.get("controller"),
                seated=bool(episode["truth_final"]["seated"]), peak_force_n=float(episode["peak_force"]), physics="none: recorded MuJoCo poses",
                note="Frame C: origin at the slot mouth, +x into the socket, +y across the cable, +z the cable normal")
    stage.SetMetadata("customLayerData", {"ffc_twin_replay": meta})
    cubes = []
    for gi, g in enumerate(episode["geom_catalogue"]):
        if g["type"] != 6:      # 6 = box; the twin uses boxes only
            continue
        name = g["name"] or f"geom_{gi:03d}"
        cube = UsdGeom.Cube.Define(stage, f"/Replay/{name}")
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        t_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
        o_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionFloat)
        xf.AddScaleOp().Set(Gf.Vec3f(*(2 * s for s in g["size"])))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*g["rgba"][:3])])
        if g["rgba"][3] < 1:
            cube.CreateDisplayOpacityAttr([float(g["rgba"][3])])
        cubes.append((gi, t_op, o_op))
    for k, rec in enumerate(episode["records"]):
        poses = rec["geoms"]
        for gi, t_op, o_op in cubes:
            p = poses[gi]
            t_op.Set(Gf.Vec3d(*p[:3]), k)
            w, x, y, z = _quat_from_mat(p[3:])
            o_op.Set(Gf.Quatf(w, x, y, z), k)
    stage.GetRootLayer().Save()
    return path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--board", default="zero"); ap.add_argument("--grip", default="tape"); ap.add_argument("--dof", type=int, default=6)
    ap.add_argument("--seed", type=int, default=3); ap.add_argument("--level", default="L1"); ap.add_argument("--estimator", default="good")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    ep = run_episode(a.board, a.grip, a.dof, a.seed, a.level, a.estimator)
    out = Path(a.out) if a.out else Path(__file__).resolve().parents[1] / "out" / f"replay_{a.board}_{a.grip}_{a.dof}dof_seed{a.seed}.usda"
    write_usd(ep, out)
    print("wrote", out, f"{out.stat().st_size/1e6:.2f} MB", "ticks", ep["ticks"], "seated", ep["truth_final"]["seated"], "peak N", round(ep["peak_force"], 2))
