"""Package mounted commissioning motion from encoder FK and offline recorded body poses.

No physics is rerun. Tool/cable poses are offline playback data, never actor inputs.
"""

import argparse
import hashlib
import json
import shutil
import sys
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.isaac_kinematics import from_stage  # noqa: E402
from ffc.learning_episode import export_mounted_episode  # noqa: E402


def archive(run, output):
    trace = json.loads((run / "sensor-trace.json").read_text())
    frames = json.loads((run / "offline-cable-trace.json").read_text())
    if not trace or not frames or any("tool_bodies" not in f for f in frames):
        raise ValueError("Need complete measured tool-body capture from runner revision 004 onward")
    output.mkdir(parents=True, exist_ok=False)
    layer = Sdf.Layer.CreateAnonymous()
    layer.ImportFromString((run / "last-scene.usda").read_text().replace("/workspace/", str(ROOT) + "/"))
    stage = Usd.Stage.Open(layer)
    chain = from_stage(stage)
    joint_bodies = [
        str(
            UsdPhysics.RevoluteJoint.Get(stage, f"/World/FR3/Physics/fr3v2_1_joint{i}")
            .GetBody1Rel()
            .GetTargets()[0]
        )
        for i in range(1, 8)
    ]
    # This vendor asset nests each link below its predecessor. Verify rather than
    # silently writing a world matrix into an unexpected hierarchy.
    for i in range(1, 7):
        if str(stage.GetPrimAtPath(joint_bodies[i]).GetParent().GetPath()) != joint_bodies[i - 1]:
            raise ValueError("Unsupported robot link hierarchy")
    base_parent = np.asarray(
        UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(joint_bodies[0]).GetParent())
    ).T
    ops = {}

    def key(path, matrix, t):
        if path not in ops:
            xf = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            if not xf:
                raise ValueError(f"Missing body: {path}")
            xf.ClearXformOpOrder()
            ops[path] = xf.AddTransformOp(opSuffix="recorded")
        ops[path].Set(Gf.Matrix4d(*matrix.T.flatten().tolist()), t * 24)

    for row in trace:
        world = chain.base.copy()
        parent_world = base_parent
        for i, q in enumerate(row["arm_q_rad"]):
            rotation = np.eye(4)
            rotation[:3, :3] = Rotation.from_rotvec(q * chain.axes[i]).as_matrix()
            world = world @ chain.parent_frames[i] @ rotation @ np.linalg.inv(chain.child_frames[i])
            key(joint_bodies[i], np.linalg.inv(parent_world) @ world, row["time_s"])
            parent_world = world.copy()
    for frame in frames:
        bodies = {
            f"/World/Cable/segment_{i:03}": {"position_m": xyz, "quaternion_wxyz": q}
            for i, (xyz, q) in enumerate(zip(frame["positions_m"], frame["quaternions_wxyz"], strict=True))
        }
        bodies.update({"/World/Tool/" + name: value for name, value in frame["tool_bodies"].items()})
        for path, body in bodies.items():
            q = body["quaternion_wxyz"]
            matrix = np.eye(4)
            matrix[:3, :3] = Rotation.from_quat([*q[1:], q[0]]).as_matrix()
            matrix[:3, 3] = body["position_m"]
            key(path, matrix, frame["time_s"])
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
        if prim.IsA(UsdPhysics.Joint):
            UsdPhysics.Joint(prim).CreateJointEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            prim.CreateAttribute("physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool).Set(False)
    stage.SetTimeCodesPerSecond(24)
    stage.SetFramesPerSecond(24)
    stage.SetStartTimeCode(trace[0]["time_s"] * 24)
    stage.SetEndTimeCode(frames[-1]["time_s"] * 24)
    stage.GetRootLayer().customLayerData = {"purpose": "Recorded state playback; physics disabled"}
    stage.Flatten().Export(str(output / "replay.usdc"))
    if not UsdUtils.CreateNewUsdzPackage(
        Sdf.AssetPath(str(output / "replay.usdc")), str(output / "replay.usdz")
    ):
        raise RuntimeError("Package failed")
    replay = Usd.Stage.Open(str(output / "replay.usdz"))
    max_error = 0.0
    for row in trace:
        actual = np.asarray(
            UsdGeom.XformCache(row["time_s"] * 24).GetLocalToWorldTransform(
                replay.GetPrimAtPath(joint_bodies[-1])
            )
        ).T
        max_error = max(max_error, float(np.max(abs(actual - chain.forward(np.asarray(row["arm_q_rad"]))))))
    for frame in frames:
        cache = UsdGeom.XformCache(frame["time_s"] * 24)
        for name, body in frame["tool_bodies"].items():
            actual = np.asarray(cache.GetLocalToWorldTransform(replay.GetPrimAtPath("/World/Tool/" + name))).T
            max_error = max(max_error, float(np.max(abs(actual[:3, 3] - body["position_m"]))))
        for i, xyz in enumerate(frame["positions_m"]):
            actual = np.asarray(
                cache.GetLocalToWorldTransform(replay.GetPrimAtPath(f"/World/Cable/segment_{i:03}"))
            ).T
            max_error = max(max_error, float(np.max(abs(actual[:3, 3] - xyz))))
    if max_error > 1e-8:
        raise RuntimeError(f"Replay disagrees with recorded motion: {max_error}")
    for prim in replay.Traverse():
        if (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        ):
            raise RuntimeError("Replay dynamics enabled")
    shutil.copy2(run / "lift.mp4", output / "video.mp4")
    for name in ["report.json", "failure.txt", "source-hashes.json"]:
        if (run / name).exists():
            shutil.copy2(run / name, output / name)
    (output / "episode.json").write_text(json.dumps(export_mounted_episode(trace, run.name)))
    manifest = {
        "run_id": run.name,
        "type": "recorded_state_playback",
        "physics_disabled": True,
        "robot_source": "Measured arm encoders and vendor joint-frame FK",
        "tool_cable_source": "Offline sampled physics body poses",
        "actor_scope": "Arm/jaw encoders, ideal pads and joint commands; no cameras",
        "max_replay_numeric_error": max_error,
        "cable_frames_checked": len(frames),
        "robot_samples_checked": len(trace),
        "final_command": trace[-1]["command"],
        "rerun_bundle": None,
        "isaac_playback_render_verified": False,
        "sha256": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in output.iterdir()},
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    print(json.dumps({"run": run.name, "replay_numeric_error": max_error, "frames": len(frames)}))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    a = parser.parse_args()
    archive(a.run, a.output)
