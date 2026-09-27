"""Load real STEP-derived assemblies into the FR3 workcell and render static CAD review.

No robot actions, insertion simulation, success oracle, or training observations.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--orbit-frames", type=int, default=72)
    parser.add_argument("--refined", action="store_true")
    args = parser.parse_args()
    if (args.output / "review.json").exists():
        parser.error("Use a fresh output directory")
    args.output.mkdir(parents=True, exist_ok=True)
    owner = args.output.stat()
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
        from pxr import Gf, UsdGeom, UsdLux

        from ffc.isaac_scene import box, build, camera
        from ffc.raspberry_pi_scene import add_assets, apply_review_materials, make_camera_cable, refine_pi4

        cfg = json.loads((ROOT / "config/isaac-workcell.json").read_text())
        cfg["tip_pose_alignment"] = False
        context = omni.usd.get_context()
        context.new_stage()
        context.get_stage().GetRootLayer().Export(str(args.output / "raspberry-pi-workcell.usda"))
        context.open_stage(str(args.output / "raspberry-pi-workcell.usda"))
        for _ in range(5):
            app.update()
        stage = context.get_stage()
        build(stage, ROOT, cfg)
        for path in ["/World/Cable", "/World/PCB", "/World/PCBStand", "/World/Connector", "/World/Regrasp"]:
            stage.RemovePrim(path)
        report = add_assets(stage, ROOT, detailed=args.refined)
        task = json.loads((ROOT / "config/raspberry-pi-task.json").read_text())
        report["task"] = task
        report["cable"] = make_camera_cable(stage, task)
        report["appearance"] = apply_review_materials(stage)
        if args.refined:
            report["refinement"] = refine_pi4(stage, ROOT)
        for i, (x, y) in enumerate(
            [(0.6035, -0.1365), (0.6615, -0.1365), (0.6035, -0.0875), (0.6615, -0.0875)]
        ):
            cyl = UsdGeom.Cylinder.Define(stage, f"/World/Hardware/BoardSupports/Post{i}")
            cyl.CreateRadiusAttr(0.0025)
            cyl.CreateHeightAttr(0.03)
            cyl.AddTranslateOp().Set(Gf.Vec3d(x, y, 0.015))
            cyl.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.13, 0.14)])
        box(
            stage,
            "/World/Hardware/CameraSupport",
            (0.49, -0.08, 0.00065),
            (0.019, 0.019, 0.0013),
            (0.10, 0.12, 0.14),
            collision=False,
        )
        UsdLux.DomeLight(stage.GetPrimAtPath("/World/Lighting/Dome")).GetIntensityAttr().Set(450)
        fill = UsdLux.RectLight.Define(stage, "/World/Lighting/HardwareFill")
        fill.AddTranslateOp().Set(Gf.Vec3d(0.63, -0.30, 0.40))
        fill.CreateWidthAttr(0.25)
        fill.CreateHeightAttr(0.25)
        fill.CreateIntensityAttr(650)
        views = {
            "workcell": ((1.1, -0.95, 0.78), (0.35, -0.04, 0.20), 35),
            "hardware": ((0.75, -0.40, 0.34), (0.525, -0.13, 0.018), 45),
            "pi4": ((0.73, -0.235, 0.15), (0.642, -0.112, 0.038), 55),
            "pi4-top": ((0.6425, -0.1121, 0.235), (0.6425, -0.112, 0.031), 55),
            "csi-macro": ((0.619, -0.15, 0.061), (0.647, -0.1285, 0.0358), 65),
            "camera3": ((0.522, -0.120, 0.063), (0.49, -0.08, 0.006), 60),
            "cable-tip": ((0.317, -0.216, 0.035), (0.34, -0.19, 0.0006), 55),
        }
        cam = camera(stage, "/World/Cameras/HardwareReview", *views["hardware"])
        cam.CreateVerticalApertureAttr(24)
        product = rep.create.render_product(str(cam.GetPath()), (1440, 960))
        products.append(product)
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(product)
        annotators.append(rgb)
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial_time = timeline.get_current_time()
        frames = []

        def capture(filename, eye, target, focal, subframes=8):
            UsdGeom.Xformable(cam).ClearXformOpOrder()
            camera(stage, str(cam.GetPath()), eye, target, focal)
            cam.CreateVerticalApertureAttr(24)
            for _ in range(2):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=subframes, pause_timeline=True)
            arr = np.asarray(rgb.get_data())
            if arr.shape[:2] != (960, 1440) or arr[..., :3].std() < 5:
                raise RuntimeError(f"Bad render: {filename} {arr.shape}")
            Image.fromarray(arr[..., :3].astype(np.uint8)).save(args.output / filename)
            if timeline.is_playing() or timeline.get_current_time() != initial_time:
                raise RuntimeError("Static review unexpectedly advanced simulation time")

        for name, (eye, target, focal) in views.items():
            capture(name + ".png", eye, target, focal)
            frames.append(
                {"id": name, "file": name + ".png", "eye_m": eye, "target_m": target, "focal_mm": focal}
            )
            print("RENDERED", name, flush=True)
        (args.output / "orbit").mkdir(exist_ok=True)
        for i in range(args.orbit_frames):
            angle = math.radians(-135 + i * 140 / max(args.orbit_frames - 1, 1))
            eye = (0.6425 + 0.17 * math.cos(angle), -0.112 + 0.17 * math.sin(angle), 0.16)
            capture(f"orbit/{i:04d}.png", eye, (0.6425, -0.112, 0.037), 55, 4)
        report.update(
            {
                "views": frames,
                "renderer": "Isaac Sim 6.1 RTX RaytracedLighting",
                "resolution": [1440, 960],
                "physics_steps_requested": 0,
                "timeline_elapsed_s": timeline.get_current_time() - initial_time,
                "orbit_frames": args.orbit_frames,
                "training_ready": False,
                "purpose": "CAD and appearance review, not a manipulation demonstration",
            }
        )
        UsdGeom.Xformable(cam).ClearXformOpOrder()
        camera(stage, str(cam.GetPath()), *views["hardware"])
        stage.GetRootLayer().customLayerData = {
            "task": "Pi4 Camera3 static CAD review",
            "training_ready": False,
        }
        stage.GetRootLayer().Save()
        (args.output / "review.json").write_text(json.dumps(report, indent=2) + "\n")
    except Exception:
        exit_code = 1
        import traceback

        traceback.print_exc()
        raise
    finally:
        for annotator in annotators:
            annotator.detach()
        for product in products:
            product.destroy()
        for path in args.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
