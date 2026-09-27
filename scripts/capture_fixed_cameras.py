"""M1 static sensor fixture: fixed RGB cameras, labels isolated for offline evaluation.

No physics stepping, robot actions, scripted episode controller or cable-state
feedback. Cases are authored static scene perturbations, not dynamic rollouts.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--final-evaluation", action="store_true")
    parser.add_argument("--evaluation-seed", type=int)
    args = parser.parse_args()
    if (args.output / "sensor" / "frames.json").exists():
        parser.error("Use a fresh output directory")
    sensor = args.output / "sensor"
    evaluator = args.output / "offline"
    sensor.mkdir(parents=True, exist_ok=True)
    evaluator.mkdir(parents=True, exist_ok=True)
    output_owner = args.output.stat()
    source = args.output / "source"
    source.mkdir(exist_ok=True)
    for path in [Path(__file__), ROOT / "src/ffc/isaac_scene.py", ROOT / "config/isaac-workcell.json"]:
        shutil.copy2(path, source / path.name)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    products, annotators = [], []
    exit_code = 0
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom, UsdLux, UsdSemantics

        from ffc.isaac_scene import box, build, camera, pose

        cfg = json.loads((ROOT / "config/isaac-workcell.json").read_text())
        cfg["tip_pose_alignment"] = False
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        stage.GetRootLayer().Export(str(args.output / "static-workcell.usda"))
        context.open_stage(str(args.output / "static-workcell.usda"))
        for _ in range(5):
            app.update()
        stage = context.get_stage()
        paths = build(stage, ROOT, cfg)
        for path in paths["cable_paths"]:
            UsdSemantics.LabelsAPI.Apply(stage.GetPrimAtPath(path), "class").CreateLabelsAttr(["cable"])
        width, height = 960, 640
        camera_designs = {
            "global": ((0.375, -0.25, 0.60), (0.375, -0.18, 0), 45.0),
            "local": ((0.42, -0.34, 0.23), (0.42, -0.18, 0), 45.0),
        }
        cameras = {}
        streams = {}
        for name, (eye, target, focal) in camera_designs.items():
            cam = camera(stage, f"/World/Cameras/Sensor_{name}", eye, target, focal)
            cam.CreateVerticalApertureAttr(24.0)
            calibration = {
                "camera_id": name,
                "width": width,
                "height": height,
                "projection": "ideal_pinhole",
                "distortion": "none; not hardware calibrated",
                "focal_length_mm": focal,
                "horizontal_aperture_mm": 36.0,
                "vertical_aperture_mm": 24.0,
                "K": [[focal / 36 * width, 0, width / 2], [0, focal / 24 * height, height / 2], [0, 0, 1]],
                "world_from_camera_usd_row_matrix": np.asarray(
                    UsdGeom.XformCache().GetLocalToWorldTransform(cam.GetPrim())
                ).tolist(),
                "camera_axes": "USD: +X right, +Y up, -Z forward",
                "mount": "fixed in world; no target tracking",
            }
            calibration["calibration_id"] = hashlib.sha256(
                json.dumps(calibration, sort_keys=True).encode()
            ).hexdigest()
            cameras[name] = calibration
            product = rep.create.render_product(str(cam.GetPath()), (width, height))
            products.append(product)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(product)
            semantic = rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"colorize": False, "semanticTypes": ["class"]}
            )
            semantic.attach(product)
            annotators.extend([(rgb, product), (semantic, product)])
            streams[name] = (rgb, semantic, cam)
        (sensor / "calibration.json").write_text(json.dumps(cameras, indent=2))
        obstacle = box(
            stage,
            "/World/PerceptionOccluder",
            (0.375, -0.18, 0.012),
            (0.025, 0.03, 0.024),
            (0.45, 0.47, 0.49),
            collision=False,
        )
        distractor = box(
            stage,
            "/World/PerceptionDistractor",
            (0.35, -0.145, 0.001),
            (0.11, 0.008, 0.0003),
            (0.08, 0.3, 0.85),
            collision=False,
        )
        cases = []
        for i, (dx, dy, angle) in enumerate(
            [
                (0, 0, 0),
                (-0.025, -0.018, -20),
                (0.02, 0.02, 18),
                (-0.012, 0.024, 30),
                (0.025, -0.02, -30),
                (0, -0.03, 10),
                (0.018, -0.012, 25),
                (-0.02, 0.015, -25),
                (0.008, 0.028, 12),
                (-0.03, -0.015, -12),
                (0.03, 0.005, 35),
                (-0.008, -0.025, -35),
            ]
        ):
            cases.append(
                dict(
                    id=f"pose-{i:02d}",
                    dx=dx,
                    dy=dy,
                    angle=angle,
                    split="development" if i < 6 else "held_out",
                    condition="pose",
                )
            )
        cases.extend(
            [
                dict(id=name, dx=0, dy=0, angle=0, split="challenge", condition=name)
                for name in ["absent", "occluded", "distractor", "low_contrast"]
            ]
        )
        if args.final_evaluation:
            cases = [
                dict(
                    id=f"final-{i:02d}", dx=dx, dy=dy, angle=angle, split="final_evaluation", condition="pose"
                )
                for i, (dx, dy, angle) in enumerate(
                    [
                        (-0.022, -0.022, 41),
                        (0.026, 0.017, -41),
                        (0.011, -0.024, 7),
                        (-0.017, 0.027, -7),
                        (0.032, -0.009, 22),
                        (-0.031, 0.008, -22),
                        (0.004, 0.031, 33),
                        (-0.005, -0.032, -33),
                    ]
                )
            ]
            cases.extend(
                dict(id="final-" + name, dx=0, dy=0, angle=0, split="final_challenge", condition=name)
                for name in ["absent", "occluded", "distractor", "low_contrast"]
            )
        if args.evaluation_seed is not None:
            if not args.final_evaluation:
                raise ValueError("An evaluation seed requires --final-evaluation")
            generator = np.random.default_rng(args.evaluation_seed)
            for case in cases:
                if case["condition"] == "pose":
                    case.update(
                        dx=float(generator.uniform(-0.032, 0.032)),
                        dy=float(generator.uniform(-0.032, 0.032)),
                        angle=float(generator.uniform(-42.0, 42.0)),
                    )
                case["evaluation_seed"] = args.evaluation_seed
        timeline = omni.timeline.get_timeline_interface()
        initial_simulation_time = timeline.get_current_time()
        start = time.monotonic()
        frames = []
        truth = []
        length = cfg["cable"]["length_m"]
        center = np.array([0.375, -0.18, 0.00015])
        for sequence, case in enumerate(cases):
            UsdGeom.Imageable(obstacle).MakeVisible() if case[
                "condition"
            ] == "occluded" else UsdGeom.Imageable(obstacle).MakeInvisible()
            UsdGeom.Imageable(distractor).MakeVisible() if case[
                "condition"
            ] == "distractor" else UsdGeom.Imageable(distractor).MakeInvisible()
            rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), case["angle"])
            for i, path in enumerate(paths["cable_paths"]):
                prim = stage.GetPrimAtPath(path)
                local = Gf.Vec3d((i + 0.5) * length / len(paths["cable_paths"]) - length / 2, 0, 0)
                point = np.array(rotation.TransformDir(local)) + center + [case["dx"], case["dy"], 0]
                pose(prim, point, Gf.Quatf(rotation.GetQuat()))
                visible = UsdGeom.Imageable(prim)
                visible.MakeInvisible() if case["condition"] == "absent" else visible.MakeVisible()
                normal = (0.08, 0.3, 0.85) if i >= 28 else (0.87, 0.80, 0.58)
                UsdGeom.Gprim(stage.GetPrimAtPath(path + "/Shape")).GetDisplayColorAttr().Set(
                    [Gf.Vec3f(*((0.47, 0.49, 0.51) if case["condition"] == "low_contrast" else normal))]
                )
            UsdLux.DomeLight(stage.GetPrimAtPath("/World/Lighting/Dome")).GetIntensityAttr().Set(
                (650 if sequence % 2 == 0 else 500)
                if args.final_evaluation
                else (700 if sequence % 2 == 0 else 450)
            )
            for _ in range(4):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=4, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial_simulation_time:
                raise RuntimeError("Static camera capture advanced the simulation clock")
            timestamp = time.monotonic() - start
            for name, (rgb, semantic, cam) in streams.items():
                raw = np.asarray(rgb.get_data())
                if raw.shape[:2] != (height, width):
                    raise RuntimeError(f"Bad RGB shape {raw.shape}")
                pixels = raw[..., :3].astype(np.uint8)
                label = semantic.get_data()
                ids = [
                    int(k)
                    for k, v in label["info"]["idToLabels"].items()
                    if "cable" in str(v.get("class", ""))
                ]
                mask = np.isin(np.asarray(label["data"]).reshape(height, width), ids)
                if case["condition"] != "absent" and not mask.any():
                    raise RuntimeError(f"Missing semantic labels: {label['info']}")
                stem = f"{sequence:03d}-{name}"
                Image.fromarray(pixels).save(sensor / f"{stem}.png")
                Image.fromarray((mask * 255).astype(np.uint8)).save(evaluator / f"{stem}-mask.png")
                current = np.asarray(UsdGeom.XformCache().GetLocalToWorldTransform(cam.GetPrim())).tolist()
                if current != cameras[name]["world_from_camera_usd_row_matrix"]:
                    raise RuntimeError("Camera moved")
                frames.append(
                    dict(
                        file=f"{stem}.png",
                        camera_id=name,
                        sequence=sequence,
                        timestamp_s=timestamp,
                        calibration_id=cameras[name]["calibration_id"],
                        width=width,
                        height=height,
                        sha256=hashlib.sha256((sensor / f"{stem}.png").read_bytes()).hexdigest(),
                    )
                )
                truth.append(
                    dict(
                        file=f"{stem}.png", mask=f"{stem}-mask.png", case=case, visible_pixels=int(mask.sum())
                    )
                )
            print(f"CAPTURE {sequence + 1}/{len(cases)} {case['id']}", flush=True)
        (sensor / "frames.json").write_text(
            json.dumps(
                {
                    "timestamp_domain": (
                        "host monotonic elapsed at render completion; not exposure timestamp or physics time"
                    ),
                    "frames": frames,
                },
                indent=2,
            )
        )
        (evaluator / "labels.json").write_text(json.dumps(truth, indent=2))
        (args.output / "capture-report.json").write_text(
            json.dumps(
                {
                    "status": "captured",
                    "images": len(frames),
                    "fixed_cameras": 2,
                    "robot_actions": 0,
                    "physics_steps_requested": 0,
                    "timeline_elapsed_s": timeline.get_current_time() - initial_simulation_time,
                    "scene_type": "authored static perturbations, not dynamic episodes",
                    "truth_location": "offline/ only; not mounted into detector",
                    "limitations": [
                        "ideal optics",
                        "uncalibrated appearances",
                        "no actual camera noise or exposure model",
                        "straight rigidly repositioned ribbon",
                        "no physical perception qualification",
                    ],
                },
                indent=2,
            )
        )
        print("CAPTURE_COMPLETE", flush=True)
    except Exception:
        exit_code = 1
        import traceback

        traceback.print_exc()
        raise
    finally:
        for a, p in annotators:
            a.detach(p)
        for p in products:
            p.destroy()
        # The host-created output directory declares artifact ownership.
        if os.geteuid() == 0:
            for path in args.output.rglob("*"):
                os.chown(path, output_owner.st_uid, output_owner.st_gid, follow_symlinks=False)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
