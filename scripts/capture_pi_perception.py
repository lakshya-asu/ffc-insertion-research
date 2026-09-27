"""Static, fixed-camera Pi4 dataset. Privileged labels remain offline only."""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
CLASSES = ["background", "cable", "contacts", "stiffener", "csi", "dsi"]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--split", choices=["development", "test"], required=True)
    p.add_argument("--count", type=int)
    p.add_argument("--live-feed", type=Path)
    p.add_argument("--stage", type=Path, default=ROOT / "outputs/pi4-refined-002/raspberry-pi-workcell.usda")
    a = p.parse_args()
    if (a.output / "sensor/frames.json").exists():
        p.error("Use a fresh output directory")
    sensor, offline = a.output / "sensor", a.output / "offline"
    sensor.mkdir(parents=True, exist_ok=True)
    offline.mkdir(exist_ok=True)
    owner = a.output.stat()
    capture_started_unix_s = time.time()
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    streams, products, exit_code = {}, [], 0
    live_rgb = None
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom, UsdLux, UsdSemantics, UsdShade

        from ffc.isaac_scene import box, camera
        from ffc.lab_feed import publish_snapshot
        from ffc.raspberry_pi_scene import cable_centerline, make_camera_cable

        context = omni.usd.get_context()
        context.open_stage(str(a.stage))
        for _ in range(8):
            app.update()
        stage = context.get_stage()
        width, height = 768, 512
        cameras = {}
        designs = {
            "desk": ((0.50, -0.20, 0.64), (0.50, -0.14, 0.005), 42),
            "board": ((0.64, -0.245, 0.22), (0.6425, -0.112, 0.035), 50),
        }
        for name, design in designs.items():
            cam = camera(stage, "/World/Cameras/Perception_" + name, *design)
            product = rep.create.render_product(str(cam.GetPath()), (width, height))
            products.append(product)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            seg = rep.AnnotatorRegistry.get_annotator(
                "semantic_segmentation", init_params={"colorize": False, "semanticTypes": ["class"]}
            )
            rgb.attach(product)
            seg.attach(product)
            streams[name] = (rgb, seg, cam)
            calibration = {
                "camera_id": name,
                "width": width,
                "height": height,
                "mount": "fixed",
                "K": [
                    [design[2] / 36 * width, 0, width / 2],
                    [0, design[2] / 24 * height, height / 2],
                    [0, 0, 1],
                ],
                "world_from_camera": np.asarray(
                    UsdGeom.XformCache().GetLocalToWorldTransform(cam.GetPrim())
                ).tolist(),
                "projection": "ideal pinhole; no hardware calibration or measured sensor noise",
            }
            calibration["calibration_id"] = hashlib.sha256(
                json.dumps(calibration, sort_keys=True).encode()
            ).hexdigest()
            cameras[name] = calibration
        if a.live_feed:
            cam = camera(stage, "/World/Cameras/LiveCapture", (1.4, -1.4, 1.25), (0.30, -0.02, 0.42), 28)
            product = rep.create.render_product(str(cam.GetPath()), (960, 640))
            products.append(product)
            live_rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            live_rgb.attach(product)
        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            component = prim.GetCustomDataByKey("cad_component") or ""
            kind = (
                "csi"
                if "Camera Connector" in component
                else "dsi"
                if "Display Connector" in component
                else None
            )
            if str(prim.GetPath()).startswith("/World/Hardware/CameraCable/"):
                path = str(prim.GetPath())
                kind = "contacts" if "/Contacts" in path else "stiffener" if "Support" in path else "cable"
            if kind:
                UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([kind])
        obstacle = box(
            stage,
            "/World/PerceptionOccluder",
            (0.44, -0.19, 0.012),
            (0.04, 0.055, 0.024),
            (0.10, 0.11, 0.12),
            collision=False,
        )
        distractor = box(
            stage,
            "/World/PerceptionDistractor",
            (0.44, -0.245, 0.001),
            (0.08, 0.016, 0.002),
            (0.03, 0.08, 0.35),
            collision=False,
        )
        baseline = json.loads((ROOT / "config/raspberry-pi-task.json").read_text())
        pi = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Hardware/Pi4"))
        supports = UsdGeom.Xform.Define(stage, "/World/Hardware/BoardSupports")
        pi_translation = Gf.Vec3d(0.6425, -0.11164793078422418, 0.031152069215775825)
        cable_prim_path = "/World/Hardware/CameraCable"
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial_time = timeline.get_current_time()
        rng = np.random.default_rng(927101 if a.split == "development" else 927203)
        count = a.count or (120 if a.split == "development" else 56)
        frames, labels = [], []
        start = time.monotonic()
        for i in range(count):
            split = (
                ("train" if i < 96 else "validation")
                if a.split == "development"
                else ("test" if i < 40 else "challenge")
            )
            condition = "ordinary"
            if a.split == "test" and i >= 40:
                condition = [
                    "absent_cable",
                    "occlusion",
                    "distractor",
                    "dim_light",
                    "no_markings",
                    "absent_board",
                    "glare",
                    "large_bow",
                ][(i - 40) % 8]
            elif a.split == "development" and i % 12 == 11:
                condition = ["absent_cable", "occlusion", "distractor", "no_markings"][i // 12 % 4]
            cfg = copy.deepcopy(baseline)
            cfg["cable"].update(
                {
                    "face_a_up": bool(rng.integers(2)),
                    "curve_height_m": float(rng.uniform(0.001, 0.008)),
                    "lateral_bow_m": float(rng.uniform(-0.012, 0.012)),
                }
            )
            if condition == "large_bow":
                cfg["cable"]["lateral_bow_m"] = 0.025
                cfg["cable"]["curve_height_m"] = 0.014
            # Unique prim identities prevent stale RTX semantic mappings when visibility or faces change.
            stage.RemovePrim(cable_prim_path)
            cable_prim_path = f"/World/Hardware/CameraCable_case_{i:04d}"
            make_camera_cable(stage, cfg, root=cable_prim_path)
            cable = UsdGeom.Xformable(stage.GetPrimAtPath(cable_prim_path))
            cable.ClearXformOpOrder()
            yaw = float(rng.uniform(-32, 32))
            dx, dy = rng.uniform(-0.018, 0.018, 2)
            pivot = Gf.Vec3d(0.435, -0.19, 0)
            cable.AddTranslateOp().Set(pivot + Gf.Vec3d(float(dx), float(dy), 0))
            cable.AddRotateZOp().Set(yaw)
            cable.AddTranslateOp(opSuffix="pivot").Set(-pivot)
            UsdGeom.Imageable(cable).MakeInvisible() if condition == "absent_cable" else UsdGeom.Imageable(
                cable
            ).MakeVisible()
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if prim.IsA(UsdGeom.Mesh) and path.startswith(cable_prim_path):
                    kind = (
                        "contacts" if "/Contacts" in path else "stiffener" if "Support" in path else "cable"
                    )
                    UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr([kind])
            board_yaw = float(rng.uniform(-180, 180))
            board_dx, board_dy = rng.uniform(-0.01, 0.01, 2)
            pi.ClearXformOpOrder()
            pi.AddTranslateOp().Set(pi_translation + Gf.Vec3d(float(board_dx), float(board_dy), 0))
            pi.AddRotateZOp().Set(board_yaw)
            supports.ClearXformOpOrder()
            supports.AddTranslateOp().Set(pi_translation + Gf.Vec3d(float(board_dx), float(board_dy), 0))
            supports.AddRotateZOp().Set(board_yaw)
            supports.AddTranslateOp(opSuffix="pivot").Set(-pi_translation)
            UsdGeom.Imageable(pi).MakeInvisible() if condition == "absent_board" else UsdGeom.Imageable(
                pi
            ).MakeVisible()
            for name in ["Silkscreen", "DrawingDetails"]:
                prim = stage.GetPrimAtPath("/World/Hardware/Pi4/" + name)
                UsdGeom.Imageable(prim).MakeInvisible() if (
                    condition == "no_markings" and name == "Silkscreen"
                ) else UsdGeom.Imageable(prim).MakeVisible()
            # Making a child visible can unhide ancestors; apply final parent visibility last.
            if condition == "absent_board":
                UsdGeom.Imageable(pi).MakeInvisible()
            UsdGeom.Imageable(obstacle).MakeVisible() if condition == "occlusion" else UsdGeom.Imageable(
                obstacle
            ).MakeInvisible()
            UsdGeom.Imageable(distractor).MakeVisible() if condition == "distractor" else UsdGeom.Imageable(
                distractor
            ).MakeInvisible()
            level = float(rng.uniform(220, 650))
            if condition == "dim_light":
                level = 60.0
            if condition == "glare":
                level = 1400.0
            UsdLux.DomeLight(stage.GetPrimAtPath("/World/Lighting/Dome")).GetIntensityAttr().Set(level)
            UsdLux.RectLight(stage.GetPrimAtPath("/World/Lighting/Key")).GetIntensityAttr().Set(
                float(rng.uniform(800, 2000))
            )
            colour = float(rng.uniform(0.15, 0.55))
            UsdGeom.Gprim(stage.GetPrimAtPath("/World/Desk/Shape")).GetDisplayColorAttr().Set(
                [Gf.Vec3f(colour * 0.98, colour, colour * 1.02)]
            )
            shader = UsdShade.Shader(stage.GetPrimAtPath(cable_prim_path + "/Materials/Film/Surface"))
            film_colour = float(rng.uniform(0.57, 0.87))
            shader.GetInput("diffuseColor").Set(Gf.Vec3f(film_colour, film_colour, film_colour * 0.96))
            for _ in range(3):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=4, pause_timeline=True)
            if timeline.get_current_time() != initial_time or timeline.is_playing():
                raise RuntimeError("Dataset capture advanced physics")
            if a.live_feed:
                publish_snapshot(
                    np.asarray(live_rgb.get_data())[..., :3].astype(np.uint8),
                    a.live_feed,
                    "workcell",
                    i,
                    (
                        f"Capturing {a.split} scene {i + 1}/{count}: {condition}. "
                        "Static randomized pose; no robot action."
                    ),
                    "Isaac Sim dataset capture",
                )
            transforms = UsdGeom.XformCache()
            center = cable_centerline(
                height_m=cfg["cable"]["curve_height_m"], lateral_m=cfg["cable"]["lateral_bow_m"]
            )
            cable_matrix = transforms.GetLocalToWorldTransform(cable.GetPrim())
            endpoint_world = [cable_matrix.Transform(Gf.Vec3d(*center[j])) for j in [0, -1]]
            pi_matrix = transforms.GetLocalToWorldTransform(pi.GetPrim())
            # Nominal visual socket center from imported CAD; not a calibrated insertion target.
            csi_world = pi_matrix.Transform(Gf.Vec3d(0.0041, -0.01685, 0.0058))
            for name, (rgb, semantic, cam) in streams.items():
                raw = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                if a.live_feed:
                    publish_snapshot(
                        raw,
                        a.live_feed,
                        name,
                        i,
                        f"Capturing {a.split} scene {i + 1}/{count}: {condition}. Raw RGB, no annotation.",
                        "Isaac Sim dataset capture",
                    )
                data = semantic.get_data()
                seg = np.asarray(data["data"]).reshape(height, width)
                mask = np.zeros((height, width), np.uint8)
                for key, value in data["info"]["idToLabels"].items():
                    label = str(value.get("class", ""))
                    for cls, text in enumerate(CLASSES[1:], 1):
                        if label == text:
                            mask[seg == int(key)] = cls
                if name == "desk" and condition == "ordinary":
                    counts = np.bincount(mask.ravel(), minlength=6)
                    if min(counts[1:4]) == 0:
                        raise RuntimeError(
                            f"Missing visible cable annotations in scene {i}: {counts.tolist()}"
                        )
                if raw.shape != (height, width, 3):
                    raise RuntimeError("Bad RGB buffer")
                if i == 0 and name == "board" and not np.isin(mask, [4, 5]).any():
                    raise RuntimeError(str(data["info"]))
                filename = f"{i:04d}-{name}.png"
                Image.fromarray(raw).save(sensor / filename)
                Image.fromarray(mask).save(offline / filename)
                timestamp = time.monotonic() - start
                frames.append(
                    {
                        "file": filename,
                        "camera_id": name,
                        "sequence": i,
                        "timestamp_s": timestamp,
                        "calibration_id": cameras[name]["calibration_id"],
                        "width": width,
                        "height": height,
                        "sha256": hashlib.sha256((sensor / filename).read_bytes()).hexdigest(),
                    }
                )
                matrix = transforms.GetLocalToWorldTransform(cam.GetPrim())
                if np.asarray(matrix).tolist() != cameras[name]["world_from_camera"]:
                    raise RuntimeError("Camera moved")

                def project(point, matrix=matrix, name=name):
                    q = matrix.GetInverse().Transform(point)
                    K = cameras[name]["K"]
                    return [
                        float(K[0][0] * q[0] / -q[2] + width / 2),
                        float(-K[1][1] * q[1] / -q[2] + height / 2),
                    ]

                physical = [project(point) for point in endpoint_world]
                if not cfg["cable"]["face_a_up"]:
                    physical.reverse()
                keypoints = []
                for point, cls in zip(physical + [project(csi_world)], [2, 3, 4], strict=True):
                    x, y = point
                    visible = 2 <= x < width - 2 and 2 <= y < height - 2 and int((mask == cls).sum()) >= 8
                    if visible:
                        xi, yi = int(x), int(y)
                        radius = 3 if cls in [2, 3] else 8
                        visible = bool(
                            (
                                mask[
                                    max(0, yi - radius) : yi + radius + 1,
                                    max(0, xi - radius) : xi + radius + 1,
                                ]
                                == cls
                            ).any()
                        )
                    keypoints.append({"xy": point, "visible": visible})
                labels.append(
                    {
                        "file": filename,
                        "split": split,
                        "condition": condition,
                        "camera_id": name,
                        "class_pixels": np.bincount(mask.ravel(), minlength=6).tolist(),
                        "keypoints": keypoints,
                        "case": {"board_yaw_deg": board_yaw, "cable_yaw_deg": yaw, **cfg["cable"]},
                    }
                )
            if i % 8 == 0:
                print("CAPTURE", i + 1, "/", count, flush=True)
        (sensor / "frames.json").write_text(
            json.dumps({"timestamp_domain": "host monotonic render completion", "frames": frames}, indent=2)
        )
        (sensor / "calibration.json").write_text(json.dumps(cameras, indent=2))
        (offline / "labels.json").write_text(json.dumps({"classes": CLASSES, "frames": labels}, indent=2))
        (a.output / "capture-report.json").write_text(
            json.dumps(
                {
                    "split": a.split,
                    "capture_started_unix_s": capture_started_unix_s,
                    "capture_completed_unix_s": time.time(),
                    "seed": 927101 if a.split == "development" else 927203,
                    "cases": count,
                    "images": len(frames),
                    "capture_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    "timeline_elapsed_s": timeline.get_current_time() - initial_time,
                    "motion_permitted": False,
                    "source_stage_sha256": hashlib.sha256(a.stage.read_bytes()).hexdigest(),
                    "real_camera_validation": False,
                    "camera_fixed": True,
                    "classes": CLASSES,
                    "label_scope": "offline supervised annotations only",
                },
                indent=2,
            )
        )
    except Exception:
        exit_code = 1
        import traceback

        traceback.print_exc()
        raise
    finally:
        for rgb, seg, _ in streams.values():
            rgb.detach()
            seg.detach()
        if live_rgb is not None:
            live_rgb.detach()
        for product in products:
            product.destroy()
        for path in a.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
