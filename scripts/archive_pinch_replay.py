"""Archive experiment 038 bench traces as portable, physics-disabled USD playback."""

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.learning_episode import export_episode  # noqa: E402


def archive(run, output):
    report = json.loads((run / "report.json").read_text())
    if "no robot" not in report["scope"]:
        raise ValueError("Exporter supports experiment 038 bench only")
    trace = json.loads((run / "sensor-trace.json").read_text())
    poses = json.loads((run / "offline-cable-trace.json").read_text())
    output.mkdir(parents=True, exist_ok=False)
    layer = Sdf.Layer.CreateAnonymous()
    layer.ImportFromString((run / "bench.usda").read_text().replace("/workspace/", str(ROOT) + "/"))
    stage = Usd.Stage.Open(layer)
    stage.SetTimeCodesPerSecond(24)
    stage.SetFramesPerSecond(24)
    stage.SetStartTimeCode(trace[0]["time_s"] * 24)
    stage.SetEndTimeCode(trace[-1]["time_s"] * 24)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        prim.CreateAttribute("physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool).Set(False)
    ops = {}

    def key(path, xyz, quat, time):
        if path not in ops:
            prim = stage.GetPrimAtPath(path)
            if not prim:
                raise ValueError(f"Missing replay body: {path}")
            xf = UsdGeom.Xformable(prim)
            xf.ClearXformOpOrder()
            ops[path] = xf.AddTransformOp(opSuffix="recorded")
        quat = np.asarray(quat, dtype=float)
        quat /= np.linalg.norm(quat)
        matrix = Gf.Matrix4d(1)
        matrix.SetRotate(Gf.Quatd(float(quat[0]), Gf.Vec3d(*quat[1:])))
        matrix.SetTranslateOnly(Gf.Vec3d(*map(float, xyz)))
        ops[path].Set(matrix, time * 24)

    for frame in poses:
        for i, (xyz, quat) in enumerate(zip(frame["positions_m"], frame["quaternions_wxyz"], strict=True)):
            key(f"/World/Cable/segment_{i:03}", xyz, quat, frame["time_s"])
    for row in trace:
        obs = row["observation"]
        for name, offset in [
            ("fixed", 0),
            ("lower", obs["lower_travel_m"]),
            ("upper", -obs["upper_travel_m"]),
        ]:
            key("/World/Tool/" + name, [0, 0, 0.055 + obs["lift_m"] + offset], [1, 0, 0, 0], row["time_s"])
    stage.GetRootLayer().customLayerData = {
        "purpose": "Recorded state playback; physics disabled; not a rerun"
    }
    stage.Flatten().Export(str(output / "replay.usdc"))
    if not UsdUtils.CreateNewUsdzPackage(
        Sdf.AssetPath(str(output / "replay.usdc")), str(output / "replay.usdz")
    ):
        raise RuntimeError("Could not package replay")
    replay = Usd.Stage.Open(str(output / "replay.usdz"))
    for frame in poses:
        for i, xyz in enumerate(frame["positions_m"]):
            actual = (
                UsdGeom.Xformable(replay.GetPrimAtPath(f"/World/Cable/segment_{i:03}"))
                .GetLocalTransformation(frame["time_s"] * 24)
                .ExtractTranslation()
            )
            if not np.allclose(actual, xyz, atol=1e-9, rtol=0):
                raise RuntimeError("Replay differs from recorded cable trace")
    for prim in replay.Traverse():
        if (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        ):
            raise RuntimeError("Replay physics still enabled")
    (output / "episode.json").write_text(json.dumps(export_episode(trace, run.name)))
    shutil.copy2(run / "pinch.mp4", output / "video.mp4")
    shutil.copy2(run / "report.json", output / "report.json")
    manifest = {
        "run_id": run.name,
        "type": "recorded_state_playback",
        "physics_disabled": True,
        "cable_frames_checked": len(poses),
        "final_state": report["final"]["command"]["state"],
        "scope": report["scope"],
        "source_hashes": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in run.glob("*.py")},
        "rerun_bundle": None,
        "isaac_gui_playback_verified": False,
        "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir()},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps({"run": run.name, "frames_checked": len(poses), "output": str(output)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    archive(args.run, args.output)
