"""Static Pi Zero 2 W side-entry review; no physics, labels or action policy."""

import argparse
import hashlib
import json
import os
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error("Use a fresh output directory to preserve evidence")
    args.output.mkdir(parents=True)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    products, exit_code = [], 0
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdLux

        from ffc.camera_optics import apply_reference_optics
        from ffc.isaac_scene import camera
        from ffc.pi_zero_scene import add_zero, make_zero_cable, set_probe

        cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        profile = json.loads((ROOT / cfg["hardware_profile"]).read_text())
        context = omni.usd.get_context()
        context.new_stage()
        stage = context.get_stage()
        stage.GetRootLayer().subLayerPaths = [
            str(ROOT / "outputs/pi4-refined-002/raspberry-pi-workcell.usda")
        ]
        stage.GetRootLayer().Export(str(args.output / "pi-zero-workcell.usda"))
        context.open_stage(str(args.output / "pi-zero-workcell.usda"))
        for _ in range(8):
            app.update()
        stage = context.get_stage()
        for path in ["/World/Hardware/Pi4", "/World/Hardware/BoardSupports", "/World/Hardware/CameraCable"]:
            stage.GetPrimAtPath(path).SetActive(False)
        cad = add_zero(stage, ROOT, cfg)
        cable = make_zero_cable(stage, cfg)
        light = UsdLux.RectLight.Define(stage, "/World/Lighting/ZeroFill")
        light.AddTransformOp().Set(
            Gf.Matrix4d()
            .SetLookAt(Gf.Vec3d(0.86, -0.22, 0.22), Gf.Vec3d(0.633, -0.14, 0.032), Gf.Vec3d(0, 0, 1))
            .GetInverse()
        )
        light.CreateWidthAttr(0.15)
        light.CreateHeightAttr(0.15)
        light.CreateIntensityAttr(650)
        metrics = []
        for view in cfg["cameras"]:
            cam = camera(stage, "/World/Cameras/Zero_" + view["id"], view["eye_m"], view["target_m"], 16)
            distance = float(np.linalg.norm(np.array(view["eye_m"]) - view["target_m"]))
            optics = apply_reference_optics(stage, cam, profile, distance)
            if view["id"] == "macro":
                optics["model"] = "Virtual inspection camera; not a deployable Arducam view"
            metrics.append(dict(view, optics=optics))
            product = rep.create.render_product(str(cam.GetPath()), (3840, 2160))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(product)
            products.append((view["id"], product, rgb))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        frames = []
        for case in ["loose", "approach", "near"]:
            pose = set_probe(stage, cfg, case)
            for _ in range(2):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=8, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            for name, _, rgb in products:
                arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                if arr.shape != (2160, 3840, 3) or arr.std() < 5:
                    raise RuntimeError(f"Bad render {case} {name}")
                filename = f"{case}-{name}.png"
                Image.fromarray(arr).save(args.output / filename)
                frames.append(
                    {
                        "file": filename,
                        "case": case,
                        "camera": name,
                        "pose": pose,
                        "sha256": hashlib.sha256((args.output / filename).read_bytes()).hexdigest(),
                    }
                )
                print("RENDERED", filename, flush=True)
        set_probe(stage, cfg, "loose")
        stage.GetRootLayer().Save()
        report = {
            "configuration": cfg,
            "camera_metrics": metrics,
            "frames": frames,
            "cad": {k: v for k, v in cad.items() if k != "components"},
            "cable": cable,
            "geometry_audit": json.loads((ROOT / "third_party/raspberry_pi/zero/audit.json").read_text()),
            "image_size": [3840, 2160],
            "renderer": "Isaac Sim 6.1 RTX RaytracedLighting",
            "motion_permitted": False,
            "training_ready": False,
            "timeline_elapsed_s": timeline.get_current_time() - initial,
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "sources": json.loads((ROOT / "config/pi-zero-sources.json").read_text()),
        }
        (args.output / "review.json").write_text(json.dumps(report, indent=2) + "\n")
    except Exception as exc:
        exit_code = 1
        traceback.print_exc()
        (args.output / "invalid.json").write_text(json.dumps({"error": str(exc)}))
    finally:
        for _, product, rgb in products:
            rgb.detach()
            product.destroy()
        owner = ROOT.stat()
        for path in args.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        os.chown(args.output, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
