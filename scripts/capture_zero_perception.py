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
    p.add_argument("--split", choices=["development", "calibration", "test"], required=True)
    p.add_argument("--count", type=int)
    p.add_argument("--profile", choices=["v1", "reliability"], default="v1")
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
        if a.profile == "reliability":
            protocol = json.loads((ROOT / "config/zero-reliability-v2.json").read_text())
            contract = {
                k: contract[k]
                for k in ["classes", "fixed_crop_xyxy", "native_resolution", "cameras", "limitations"]
            }
            contract["reliability_protocol"] = protocol
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
                "instance_id_segmentation", init_params={"colorize": False}
            )
            seg.attach(rp)
            streams.append((view["id"], rp, rgb, seg))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        seed = (
            {"development": 928401, "calibration": 928451, "test": 928503}[a.split]
            if a.profile == "reliability"
            else (927401 if a.split == "development" else 927503)
        )
        rng = np.random.default_rng(seed)
        count = a.count or (
            {"development": 200, "calibration": 80, "test": 120}[a.split]
            if a.profile == "reliability"
            else (80 if a.split == "development" else 36)
        )
        frames = []
        labels = []
        # Keep semantic prim identities stable throughout the run. Delete/recreate
        # caused labels from removed cable meshes to attach to unrelated geometry.
        root = "/World/ZeroDataset/Scene"
        group = UsdGeom.Xform.Define(stage, root)
        group.AddTranslateOp()
        group.AddRotateZOp()
        board = UsdGeom.Xform.Define(stage, root + "/Board")
        board.GetPrim().GetReferences().AddReference(str(ROOT / cfg["asset"]))
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith(root + "/Board") and prim.IsA(UsdGeom.Mesh):
                if "22 Pin FPC Connector" in (prim.GetCustomDataByKey("cad_component") or ""):
                    UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr(["connector"])
        UsdGeom.Xform.Define(stage, root + "/Supports")
        posts = []
        for post_i, (px, py) in enumerate(
            [(-0.029, -0.0123), (0.029, -0.0123), (-0.029, 0.0107), (0.029, 0.0107)]
        ):
            post = UsdGeom.Cylinder.Define(stage, root + f"/Supports/Post{post_i}")
            post.CreateRadiusAttr(0.0024)
            post.CreateHeightAttr(0.03)
            post.AddTranslateOp().Set(Gf.Vec3d(px, py, -0.015))
            post.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.13, 0.14)])
            posts.append((post, px, py))
        cable_root = root + "/Cable"
        make_zero_cable(stage, cfg, root=cable_root)
        cable = UsdGeom.Xformable(stage.GetPrimAtPath(cable_root))
        cable.AddTranslateOp()
        cable.AddRotateZOp()
        cable.AddRotateXOp()
        for prim in stage.Traverse():
            name = str(prim.GetPath())
            if name.startswith(cable_root) and (prim.IsA(UsdGeom.Mesh) or prim.IsA(UsdGeom.Cube)):
                kind = "mini_end" if "/Mini" in name else "cable"
                UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([kind])
        shader = UsdShade.Shader(stage.GetPrimAtPath(cable_root + "/Materials/Film/Surface"))
        jaws = UsdGeom.Xform.Define(stage, root + "/Jaws")
        jaws.AddTranslateOp()
        jaws.AddRotateZOp()
        jaws.AddRotateXOp()
        for j, sign in enumerate([-1, 1]):
            prim = box(
                stage,
                root + f"/Jaws/Jaw{j}",
                (0.0075, 0, sign * 0.00175),
                (0.004, 0.012, 0.003),
                (0.16, 0.18, 0.2),
            )
            UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr(["gripper"])

        def visible(path, value):
            obj = UsdGeom.Imageable(stage.GetPrimAtPath(path))
            obj.MakeVisible() if value else obj.MakeInvisible()

        start = time.monotonic()
        for i in range(count):
            condition = "ordinary"
            if a.profile == "reliability":
                condition = str(
                    rng.choice(
                        [
                            "ordinary",
                            "absent_board",
                            "absent_cable",
                            "empty",
                            "dim",
                            "bright",
                            "large_offset",
                            "large_yaw",
                            "tool_occlusion",
                            "blur",
                        ],
                        p=[0.45, 0.06, 0.06, 0.04, 0.06, 0.06, 0.07, 0.07, 0.08, 0.05],
                    )
                )
            elif a.split == "test" and i >= 20:
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
                shift[1] = (
                    0.016 * int(rng.choice([-1, 1]))
                    if a.profile == "reliability"
                    else 0.016 * (-1 if i % 2 else 1)
                )
            if condition == "large_yaw":
                yaw = (
                    20 * int(rng.choice([-1, 1])) if a.profile == "reliability" else 20 * (-1 if i % 2 else 1)
                )
            group.GetPrim().GetAttribute("xformOp:translate").Set(
                Gf.Vec3d(*(np.array(cfg["board_translation_m"]) + shift))
            )
            group.GetPrim().GetAttribute("xformOp:rotateZ").Set(yaw)
            board_present = condition not in ["absent_board", "empty"]
            cable_present = condition not in ["absent_cable", "empty"]
            visible(root + "/Board", board_present)
            visible(root + "/Supports", board_present)
            visible(cable_root, cable_present)
            height = cfg["board_translation_m"][2] + float(shift[2])
            for post, px, py in posts:
                post.GetHeightAttr().Set(height)
                post.GetPrim().GetAttribute("xformOp:translate").Set(Gf.Vec3d(px, py, -height / 2))
            gap = float(rng.uniform(0.003, 0.025))
            flip = rng.random() < 0.2 if a.profile == "reliability" else i % 5 == 0
            roll = float(rng.uniform(-12, 12) + (180 if flip else 0))
            cyaw = float(rng.uniform(-10, 10))
            dy = float(rng.uniform(-0.004, 0.004))
            dz = float(rng.uniform(-0.0008, 0.0008))
            jaw_visible = (
                (rng.random() < 2 / 3) if a.profile == "reliability" else (i % 3 != 0)
            ) or condition == "tool_occlusion"
            visible(root + "/Jaws", cable_present and jaw_visible)
            if cable_present:
                tip = np.array([0.0333 + gap, -0.0008 + dy, 0.0022 + dz])
                for obj in [cable.GetPrim(), jaws.GetPrim()]:
                    obj.GetAttribute("xformOp:translate").Set(Gf.Vec3d(*tip))
                    obj.GetAttribute("xformOp:rotateZ").Set(cyaw)
                    obj.GetAttribute("xformOp:rotateX").Set(roll)
                grey = float(rng.uniform(0.015, 0.065))
                shader.GetInput("diffuseColor").Set(Gf.Vec3f(grey, grey * 1.05, grey * 1.1))
                x = 0.003 if condition == "tool_occlusion" else 0.0075
                for j, sign in enumerate([-1, 1]):
                    stage.GetPrimAtPath(root + f"/Jaws/Jaw{j}").GetAttribute("xformOp:translate").Set(
                        Gf.Vec3d(x, 0, sign * 0.00175)
                    )
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
                    path = str(value)
                    cls = 0
                    if path.startswith(root + "/Cable/"):
                        cls = 3 if path.startswith(root + "/Cable/Mini") else 2
                    elif path.startswith(root + "/Jaws/"):
                        cls = 4
                    elif path.startswith(root + "/Board/"):
                        prim = stage.GetPrimAtPath(path)
                        while prim and str(prim.GetPath()).startswith(root + "/Board/"):
                            if "22 Pin FPC Connector" in (prim.GetCustomDataByKey("cad_component") or ""):
                                cls = 1
                                break
                            prim = prim.GetParent()
                    if cls:
                        mask[ids == int(key)] = cls
                if i < 2:
                    (offline / f"instance-map-{i:04d}-{name}.json").write_text(
                        json.dumps(data["info"], default=str, indent=2)
                    )
                x0, y0, x1, y1 = contract["fixed_crop_xyxy"]
                cropped = mask[y0:y1, x0:x1]
                counts = np.bincount(cropped.ravel(), minlength=len(classes)).tolist()
                if condition in ["absent_board", "absent_cable", "empty"] or (
                    condition == "ordinary" and (counts[1] < 20 or counts[2] < 50)
                ):
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
                if condition == "ordinary" and not jaw_visible and counts[1] < 20:
                    raise RuntimeError("Missing unobstructed connector label")
                if condition == "ordinary" and counts[2] < 50:
                    raise RuntimeError("Missing visible cable label")
                if condition == "blur":
                    from PIL import ImageFilter

                    raw = np.array(Image.fromarray(raw).filter(ImageFilter.GaussianBlur(2.0)))
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
                if a.profile == "reliability":
                    split = ("train" if i < 160 else "validation") if a.split == "development" else a.split
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
            "profile": a.profile,
            "semantic_identity": "persistent prims; visibility and transforms only",
            "seed": seed,
            "images": len(frames),
            "split": a.split,
            "timeline_elapsed_s": timeline.get_current_time() - initial,
            "motion_permitted": False,
            "contract": contract,
            "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "elapsed_s": time.monotonic() - start,
            "scope": "offline visible CAD component masks, no aperture or latch labels",
            "label_source": "renderer instance IDs mapped to explicit USD paths, not dynamic semantic IDs",
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
