"""Native RGB and separate offline visible-region labels for the Zero side-entry study."""

import argparse
import hashlib
import json
import os
import sys
import time
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--split", choices=["development", "test"], required=True)
    p.add_argument("--count", type=int)
    a = p.parse_args()
    if a.output.exists():
        p.error("Use a fresh output directory")
    sensor = a.output / "sensor"
    offline = a.output / "offline"
    sensor.mkdir(parents=True)
    offline.mkdir()
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    streams = []
    exit_code = 0
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom, UsdSemantics, UsdShade

        from ffc.camera_optics import apply_reference_optics
        from ffc.isaac_scene import camera
        from ffc.lab_feed import publish_snapshot
        from ffc.pi_zero_scene import box, make_zero_cable

        cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        contract = json.loads((ROOT / "config/zero-perception-v1.json").read_text())
        classes = contract["classes"]
        profile = json.loads((ROOT / cfg["hardware_profile"]).read_text())
        ctx = omni.usd.get_context()
        ctx.open_stage(str(ROOT / "outputs/pi-zero-review-002/pi-zero-workcell.usda"))
        for _ in range(8):
            app.update()
        stage = ctx.get_stage()
        for name in ["Zero2W", "ZeroCable", "ZeroJawProbe", "ZeroSupports"]:
            stage.GetPrimAtPath("/World/Hardware/" + name).SetActive(False)
        specs = {}
        for view in cfg["cameras"]:
            if view["id"] not in contract["cameras"]:
                continue
            cam = camera(stage, "/World/Cameras/Training_" + view["id"], view["eye_m"], view["target_m"], 16)
            optical = apply_reference_optics(
                stage, cam, profile, float(np.linalg.norm(np.array(view["eye_m"]) - view["target_m"]))
            )
            specs[view["id"]] = {
                "width": 3840,
                "height": 2160,
                "K": optical["K"],
                "crop_xyxy": contract["fixed_crop_xyxy"],
                "projection": "ideal reference, uncalibrated hardware",
            }
            specs[view["id"]]["calibration_id"] = hashlib.sha256(
                json.dumps(specs[view["id"]], sort_keys=True).encode()
            ).hexdigest()
            rp = rep.create.render_product(str(cam.GetPath()), (3840, 2160))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            seg = rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"colorize": False, "semanticTypes": ["class"]}
            )
            seg.attach(rp)
            streams.append((view["id"], rp, rgb, seg))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        rng = np.random.default_rng(927401 if a.split == "development" else 927503)
        count = a.count or (80 if a.split == "development" else 36)
        frames = []
        labels = []
        previous = None
        start = time.monotonic()
        for i in range(count):
            if previous:
                stage.RemovePrim(previous)
            root = f"/World/ZeroDataset/Scene_{i:04d}"
            previous = root
            group = UsdGeom.Xform.Define(stage, root)
            condition = "ordinary"
            if a.split == "test" and i >= 20:
                condition = [
                    "absent_board",
                    "absent_cable",
                    "empty",
                    "dim",
                    "bright",
                    "large_offset",
                    "large_yaw",
                    "tool_occlusion",
                ][(i - 20) % 8]
            elif i % 8 == 7:
                condition = ["absent_board", "absent_cable", "empty", "tool_occlusion"][(i // 8) % 4]
            shift = np.array(
                [rng.uniform(-0.006, 0.006), rng.uniform(-0.008, 0.008), rng.uniform(-0.001, 0.001)]
            )
            yaw = float(rng.uniform(-8, 8))
            if condition == "large_offset":
                shift[1] = 0.016 * (-1 if i % 2 else 1)
            if condition == "large_yaw":
                yaw = 20 * (-1 if i % 2 else 1)
            group.AddTranslateOp().Set(Gf.Vec3d(*(np.array(cfg["board_translation_m"]) + shift)))
            group.AddRotateZOp().Set(yaw)
            if condition not in ["absent_board", "empty"]:
                board = UsdGeom.Xform.Define(stage, root + "/Board")
                board.GetPrim().GetReferences().AddReference(str(ROOT / cfg["asset"]))
                height = cfg["board_translation_m"][2] + float(shift[2])
                for post_i, (px, py) in enumerate(
                    [(-0.029, -0.0123), (0.029, -0.0123), (-0.029, 0.0107), (0.029, 0.0107)]
                ):
                    post = UsdGeom.Cylinder.Define(stage, root + f"/Supports/Post{post_i}")
                    post.CreateRadiusAttr(0.0024)
                    post.CreateHeightAttr(height)
                    post.AddTranslateOp().Set(Gf.Vec3d(px, py, -height / 2))
                    post.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.13, 0.14)])
                for prim in stage.Traverse():
                    if str(prim.GetPath()).startswith(root + "/Board") and prim.IsA(UsdGeom.Mesh):
                        if "22 Pin FPC Connector" in (prim.GetCustomDataByKey("cad_component") or ""):
                            UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr(["connector"])
            gap = float(rng.uniform(0.003, 0.025))
            roll = float(rng.uniform(-12, 12) + (180 if i % 5 == 0 else 0))
            cyaw = float(rng.uniform(-10, 10))
            dy = float(rng.uniform(-0.004, 0.004))
            dz = float(rng.uniform(-0.0008, 0.0008))
            jaw_visible = (i % 3 != 0) or condition == "tool_occlusion"
            if condition not in ["absent_cable", "empty"]:
                cable_root = root + "/Cable"
                make_zero_cable(stage, cfg, root=cable_root)
                cable = UsdGeom.Xformable(stage.GetPrimAtPath(cable_root))
                tip = np.array([0.0333 + gap, -0.0008 + dy, 0.0022 + dz])
                cable.AddTranslateOp().Set(Gf.Vec3d(*tip))
                cable.AddRotateZOp().Set(cyaw)
                cable.AddRotateXOp().Set(roll)
                for prim in stage.Traverse():
                    name = str(prim.GetPath())
                    if name.startswith(cable_root) and (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
                        kind = "mini_end" if "/Mini" in name else "cable"
                        UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([kind])
                shader = UsdShade.Shader(stage.GetPrimAtPath(cable_root + "/Materials/Film/Surface"))
                grey = float(rng.uniform(0.015, 0.065))
                shader.GetInput("diffuseColor").Set(Gf.Vec3f(grey, grey * 1.05, grey * 1.1))
                if jaw_visible:
                    jaws = UsdGeom.Xform.Define(stage, root + "/Jaws")
                    jaws.AddTranslateOp().Set(Gf.Vec3d(*tip))
                    jaws.AddRotateZOp().Set(cyaw)
                    jaws.AddRotateXOp().Set(roll)
                    x = 0.003 if condition == "tool_occlusion" else 0.0075
                    for j, sign in enumerate([-1, 1]):
                        prim = box(
                            stage,
                            root + f"/Jaws/Jaw{j}",
                            (x, 0, sign * 0.00175),
                            (0.004, 0.012, 0.003),
                            (0.16, 0.18, 0.2),
                        )
                        UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr(["gripper"])
            scale = (
                0.15 if condition == "dim" else 2.5 if condition == "bright" else float(rng.uniform(0.6, 1.4))
            )
            for name, nominal in [("Dome", 450), ("Key", 1400), ("ZeroFill", 650), ("HardwareFill", 650)]:
                prim = stage.GetPrimAtPath("/World/Lighting/" + name)
                if prim:
                    prim.GetAttribute("inputs:intensity").Set(nominal * scale)
            for _ in range(4):
                app.update()
            for _ in range(3):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=4, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            for name, _, rgb, seg in streams:
                raw = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                data = seg.get_data()
                ids = np.asarray(data["data"]).reshape(2160, 3840)
                mask = np.zeros((2160, 3840), np.uint8)
                for key, value in data["info"]["idToLabels"].items():
                    text = value.get("class", "")
                    if text in classes[1:]:
                        mask[ids == int(key)] = classes.index(text)
                x0, y0, x1, y1 = contract["fixed_crop_xyxy"]
                cropped = mask[y0:y1, x0:x1]
                counts = np.bincount(cropped.ravel(), minlength=len(classes)).tolist()
                if condition in ["absent_board", "absent_cable", "empty"]:
                    (offline / f"debug-{i:04d}-{name}.json").write_text(
                        json.dumps(
                            {"condition": condition, "counts": counts, "labels": data["info"]["idToLabels"]},
                            indent=2,
                        )
                    )
                    Image.fromarray(cropped).save(offline / f"debug-{i:04d}-{name}-mask.png")
                    Image.fromarray(raw).save(offline / f"debug-{i:04d}-{name}-rgb.png")
                if condition in ["absent_board", "empty"] and counts[1]:
                    raise RuntimeError("Stale board labels")
                if condition in ["absent_cable", "empty"] and sum(counts[2:]):
                    raise RuntimeError("Stale cable labels")
                if condition == "ordinary" and i % 3 == 0 and counts[1] < 20:
                    raise RuntimeError("Missing unobstructed connector label")
                if condition == "ordinary" and counts[2] < 50:
                    raise RuntimeError("Missing visible cable label")
                file = f"{i:04d}-{name}.png"
                Image.fromarray(raw).save(sensor / file)
                Image.fromarray(cropped).save(offline / file)
                frames.append(
                    {
                        "file": file,
                        "camera_id": name,
                        "sequence": i,
                        "timestamp_s": time.monotonic() - start,
                        "width": 3840,
                        "height": 2160,
                        "calibration_id": specs[name]["calibration_id"],
                        "sha256": hashlib.sha256((sensor / file).read_bytes()).hexdigest(),
                    }
                )
                split = (
                    ("train" if i < 64 else "validation")
                    if a.split == "development"
                    else ("test" if i < 20 else "challenge")
                )
                labels.append(
                    {
                        "file": file,
                        "scene": i,
                        "split": split,
                        "condition": condition,
                        "camera_id": name,
                        "class_pixels": counts,
                        "offline_pose": {
                            "shift_m": shift.tolist(),
                            "yaw_deg": yaw,
                            "gap_m": gap,
                            "cable_roll_deg": roll,
                            "cable_yaw_deg": cyaw,
                            "tip_dy_m": dy,
                            "tip_dz_m": dz,
                        },
                    }
                )
                publish_snapshot(
                    raw[::4, ::4],
                    ROOT / "outputs/lab-live/feed",
                    "board" if name == "entrance" else "desk",
                    i,
                    (
                        f"Zero perception capture: {a.split} scene {i + 1}/{count}, {name}. "
                        "Raw RGB; static authored pose."
                    ),
                    "Isaac Sim dataset capture",
                )
            print("CAPTURE", i + 1, count, condition, flush=True)
        (sensor / "frames.json").write_text(json.dumps({"frames": frames}, indent=2))
        (sensor / "calibration.json").write_text(json.dumps(specs, indent=2))
        (offline / "labels.json").write_text(
            json.dumps({"classes": classes, "frames": labels, "contract": contract}, indent=2)
        )
        report = {
            "scenes": count,
            "images": len(frames),
            "split": a.split,
            "timeline_elapsed_s": timeline.get_current_time() - initial,
            "motion_permitted": False,
            "contract": contract,
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "elapsed_s": time.monotonic() - start,
            "scope": "offline visible CAD component masks, no aperture or latch labels",
        }
        (a.output / "capture-report.json").write_text(json.dumps(report, indent=2))
    except Exception as exc:
        exit_code = 1
        traceback.print_exc()
        (a.output / "invalid.json").write_text(json.dumps({"error": str(exc)}))
    finally:
        for _, rp, rgb, seg in streams:
            rgb.detach()
            seg.detach()
            rp.destroy()
        owner = ROOT.stat()
        for path in a.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        os.chown(a.output, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
