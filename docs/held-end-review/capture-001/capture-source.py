"""Prepared Isaac capture: raw fixed-camera RGB from a physics-disabled motion replay.

No task geometry is read by the perception path. Intrinsics and camera extrinsics
are sensor metadata. This is sampled recorded-state capture, not new physics.
"""

import argparse
import hashlib
import json
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--config", type=Path, default=ROOT / "config/held-end-inspection-v1.json")
    a = p.parse_args()
    cfg = json.loads(a.config.read_text())
    optics = json.loads((ROOT / cfg["optics_profile"]).read_text())
    a.output.mkdir(parents=True, exist_ok=False)
    (a.output / "capture-config.json").write_text(json.dumps(cfg, indent=2))
    (a.output / "optics-profile.json").write_text(json.dumps(optics, indent=2))
    (a.output / "capture-source.py").write_bytes(Path(__file__).read_bytes())
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "renderer": "RaytracedLighting", "extra_args": ["--allow-root"]})
    rows = []
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom, UsdPhysics

        from ffc.camera_optics import apply_reference_optics
        from ffc.macro_camera import optical_budget

        omni.usd.get_context().open_stage(str(a.stage))
        for _ in range(8):
            app.update()
        stage = omni.usd.get_context().get_stage()
        for prim in stage.Traverse():
            if (
                prim.HasAPI(UsdPhysics.RigidBodyAPI)
                and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
            ):
                raise ValueError("Capture requires a physics-disabled replay")
        budget = optical_budget(optics)
        yaw, elevation = np.deg2rad([cfg["yaw_degrees"], cfg["elevation_degrees"]])
        axis = np.array([np.cos(elevation) * np.cos(yaw), np.cos(elevation) * np.sin(yaw), np.sin(elevation)])
        target = np.asarray(cfg["target_world_m"])
        eye = target + axis * budget["object_distance_from_inferred_principal_plane_mm"] / 1000
        camera = UsdGeom.Camera.Define(stage, "/World/Cameras/HeldEndInspection")
        camera.AddTransformOp().Set(
            Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
        )
        camera.CreateClippingRangeAttr(Gf.Vec2f(0.0001, 20))
        projection = dict(optics, focal_length_mm=budget["projection_focal_length_mm"])
        intrinsics = apply_reference_optics(
            stage, camera, projection, float(np.linalg.norm(eye - target)), cfg["resolution"]
        )
        camera.CreateFStopAttr(budget["equivalent_renderer_f_number"])
        world_optical = np.asarray(
            UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim())
        ).T @ np.diag([1, -1, -1, 1])
        calibration = {
            "camera_id": cfg["camera_id"],
            "frame_id": cfg["frame_id"],
            "K": intrinsics["K"],
            "resolution": cfg["resolution"],
            "world_from_optical": world_optical.tolist(),
            "distortion": {"model": "ideal_pinhole", "coefficients": [0, 0, 0, 0, 0]},
            "focus_distance_m": intrinsics["focus_distance_m"],
            "renderer_f_number": budget["equivalent_renderer_f_number"],
            "calibration_kind": "synthetic camera model; physical calibration pending",
        }
        calibration_id = hashlib.sha256(json.dumps(calibration, sort_keys=True).encode()).hexdigest()
        (a.output / "calibration.json").write_text(
            json.dumps(calibration | {"calibration_id": calibration_id}, indent=2)
        )
        product = rep.create.render_product(str(camera.GetPath()), tuple(cfg["resolution"]))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        for sequence, stamp in enumerate(cfg["sample_times_s"]):
            frame = stamp * stage.GetTimeCodesPerSecond()
            if not stage.GetStartTimeCode() <= frame <= stage.GetEndTimeCode():
                raise ValueError("Requested time outside recorded replay")
            timeline.set_current_time(stamp)
            for _ in range(8):
                rep.orchestrator.step(delta_time=0, pause_timeline=True)
            pixels = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
            if pixels.shape != (cfg["resolution"][1], cfg["resolution"][0], 3):
                raise ValueError("Unexpected camera buffer")
            name = f"rgb-{sequence:04}.png"
            Image.fromarray(pixels).save(a.output / name)
            rows.append(
                {
                    "sequence": sequence,
                    "timestamp_s": stamp,
                    "clock": "recorded_simulation_seconds",
                    "capture_wall_time_s": time.time(),
                    "camera_id": cfg["camera_id"],
                    "calibration_id": calibration_id,
                    "file": name,
                    "sha256": hashlib.sha256((a.output / name).read_bytes()).hexdigest(),
                }
            )
        stage.GetRootLayer().Export(str(a.output / "inspection-scene.usda"))
    except Exception:
        import traceback

        (a.output / "failure.txt").write_text(traceback.format_exc())
        raise
    finally:
        (a.output / "manifest.json").write_text(
            json.dumps(
                {
                    "frames": rows,
                    "expected_frames": len(cfg["sample_times_s"]),
                    "complete": len(rows) == len(cfg["sample_times_s"]),
                    "source_replay_sha256": hashlib.sha256(a.stage.read_bytes()).hexdigest(),
                    "mode": "Recorded state RGB capture; no physics rerun",
                    "motion_permitted": False,
                },
                indent=2,
            )
        )
        app.close()


if __name__ == "__main__":
    main()
