"""Feature-level simulated RGB supervision with a declared engineered socket substitute."""

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
    p.add_argument("--split", choices=["development", "test"], required=True)
    p.add_argument("--count", type=int)
    p.add_argument("--seed", type=int, help="Explicit independent capture seed; recorded in report")
    a = p.parse_args()
    if a.output.exists():
        p.error("Fresh output required")
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
        from PIL import Image, ImageFilter
        from pxr import Gf, UsdGeom, UsdLux

        from ffc.camera_optics import apply_reference_optics
        from ffc.entrance_features import label_leading_band, make_socket, set_slider
        from ffc.isaac_scene import camera
        from ffc.lab_feed import publish_snapshot
        from ffc.pi_zero_scene import box, make_zero_cable

        cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        spec = json.loads((ROOT / "config/zero-feature-v1.json").read_text())
        profile = json.loads((ROOT / cfg["hardware_profile"]).read_text())
        seed = spec[a.split + "_seed"] if a.seed is None else a.seed
        rng = np.random.default_rng(seed)
        count = a.count or spec[a.split + "_scenes"]
        ctx = omni.usd.get_context()
        ctx.open_stage(str(ROOT / "outputs/pi-zero-review-002/pi-zero-workcell.usda"))
        for _ in range(8):
            app.update()
        stage = ctx.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        for name in ["Zero2W", "ZeroCable", "ZeroJawProbe"]:
            stage.GetPrimAtPath("/World/Hardware/" + name).SetActive(False)
        root = "/World/FeatureTask"
        task = UsdGeom.Xform.Define(stage, root)
        board = UsdGeom.Xform.Define(stage, root + "/Board")
        board.GetPrim().GetReferences().AddReference(str(ROOT / cfg["asset"]))
        stage.GetPrimAtPath(root + "/Board/Components/Part_0010").SetActive(False)
        make_socket(stage, root + "/Socket", spec)
        socket = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/Socket"))
        socket.AddTranslateOp().Set(Gf.Vec3d(0.0333, -0.0008, 0.0022))
        make_zero_cable(stage, cfg, root + "/Cable")
        label_leading_band(stage, root + "/Cable", cfg)
        jaws = UsdGeom.Xform.Define(stage, root + "/Jaws")
        for j, sign in enumerate([-1, 1]):
            box(
                stage,
                root + f"/Jaws/J{j}",
                (0, 0, sign),
                (0.004, 0.008, 0.0015),
                (0.15, 0.16, 0.18),
                collision=False,
            )
        light = UsdLux.RectLight.Define(stage, root + "/SpecularProbe")
        light.AddTranslateOp().Set(Gf.Vec3d(0.08, -0.04, 0.10))
        light.CreateWidthAttr(0.07)
        light.CreateHeightAttr(0.02)
        light.CreateIntensityAttr(500)
        for view in cfg["cameras"]:
            if view["id"] not in ["entrance", "offset"]:
                continue
            cam = camera(stage, "/World/Cameras/Feature_" + view["id"], view["eye_m"], view["target_m"], 16)
            optics = apply_reference_optics(
                stage, cam, profile, float(np.linalg.norm(np.array(view["eye_m"]) - view["target_m"]))
            )
            product = rep.create.render_product(str(cam.GetPath()), (3840, 2160))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(product)
            seg = rep.AnnotatorRegistry.get_annotator(
                "instance_id_segmentation", init_params={"colorize": False}
            )
            seg.attach(product)
            streams.append((view["id"], product, rgb, seg, optics))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        frames = []
        conditions = [
            "ordinary",
            "glare",
            "dim",
            "rotation",
            "occlusion",
            "blur",
            "absent_cable",
            "absent_socket",
        ]
        for i in range(count):
            condition = conditions[i % len(conditions)]
            opened = bool(rng.integers(2))
            yaw = float(rng.uniform(-8, 8) if condition == "rotation" else rng.uniform(-3, 3))
            delta = rng.uniform([-0.002, -0.004, 0], [0.002, 0.004, 0.001])
            task.ClearXformOpOrder()
            task.AddTranslateOp().Set(Gf.Vec3d(*(np.array(cfg["board_translation_m"]) + delta)))
            task.AddRotateZOp().Set(yaw)
            gap = float(rng.uniform(0.001, 0.02))
            tip = np.array(
                [0.0333 + gap, -0.0008 + rng.uniform(-0.003, 0.003), 0.0022 + rng.uniform(-0.0004, 0.001)]
            )
            cable = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/Cable"))
            cable.ClearXformOpOrder()
            cable.AddTranslateOp().Set(Gf.Vec3d(*tip))
            cable.AddRotateZOp().Set(float(rng.uniform(-10, 10)))
            set_slider(stage, root + "/Socket", spec, opened)
            for path, visible in [
                (root + "/Cable", condition != "absent_cable"),
                (root + "/Socket", condition != "absent_socket"),
            ]:
                (
                    UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeVisible
                    if visible
                    else UsdGeom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible
                )()
            show_jaws = condition == "occlusion" or bool(rng.random() < 0.35)
            (UsdGeom.Imageable(jaws).MakeVisible if show_jaws else UsdGeom.Imageable(jaws).MakeInvisible)()
            setback = float(rng.choice([0.006, 0.010, 0.015]))
            thick = float(rng.choice([0.0015, 0.003]))
            for j, sign in enumerate([-1, 1]):
                jaw = UsdGeom.Xformable(stage.GetPrimAtPath(root + f"/Jaws/J{j}"))
                jaw.ClearXformOpOrder()
                jaw.AddTranslateOp().Set(Gf.Vec3d(*(tip + [setback, 0, sign * (thick / 2 + 0.00025)])))
                shape = UsdGeom.Xformable(stage.GetPrimAtPath(root + f"/Jaws/J{j}/Shape"))
                shape.ClearXformOpOrder()
                shape.AddScaleOp().Set(Gf.Vec3f(0.004, 0.012 if condition == "occlusion" else 0.008, thick))
            light.GetIntensityAttr().Set(15000 if condition == "glare" else 500)
            dome = stage.GetPrimAtPath("/World/Lighting/Dome")
            if dome and dome.HasAttribute("inputs:intensity"):
                dome.GetAttribute("inputs:intensity").Set(100 if condition == "dim" else 1000)
            for _ in range(4):
                app.update()
            for _ in range(3):
                rep.orchestrator.step(delta_time=0, rt_subframes=4, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            for name, _, rgb, seg, _optics in streams:
                arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                image = Image.fromarray(arr)
                if condition == "blur":
                    image = image.filter(ImageFilter.GaussianBlur(1.2))
                if condition == "dim":
                    image = Image.fromarray((np.asarray(image) * 0.35).astype(np.uint8))
                data = seg.get_data()
                ids = np.asarray(data["data"]).reshape(2160, 3840)
                mask = np.zeros(ids.shape, np.uint8)
                for key, val in data["info"]["idToLabels"].items():
                    path = str(val)
                    cls = 0
                    if path == root + "/Socket/Upper/Front":
                        cls = 1
                    elif path == root + "/Socket/Lower/Front":
                        cls = 2
                    elif path.startswith(root + "/Cable/LeadingBand"):
                        cls = 3
                    elif path.startswith(root + "/Socket/Slider/"):
                        cls = 4 if opened else 5
                    if cls:
                        mask[ids == int(key)] = cls
                crop = spec["crop_xyxy"]
                mask = mask[crop[1] : crop[3], crop[0] : crop[2]]
                if condition == "absent_socket" and np.isin(mask, [1, 2, 4, 5]).any():
                    raise RuntimeError("Stale socket label")
                if condition == "absent_cable" and (mask == 3).any():
                    raise RuntimeError("Stale cable label")
                filename = f"{i:04d}-{name}.png"
                image.save(sensor / filename)
                Image.fromarray(mask).save(offline / filename)
                split = (
                    ("validation" if i >= count - spec["validation_scenes"] else "train")
                    if a.split == "development"
                    else "test"
                )
                row = dict(
                    file=filename,
                    scene=i,
                    split=split,
                    camera=name,
                    condition=condition,
                    slider_open=opened,
                    visible_pixels=[int((mask == c).sum()) for c in range(6)],
                    authored_pose=dict(board_delta_m=delta.tolist(), yaw_deg=yaw, tip_local_m=tip.tolist()),
                    rgb_sha256=hashlib.sha256((sensor / filename).read_bytes()).hexdigest(),
                )
                frames.append(row)
                publish_snapshot(
                    np.asarray(image),
                    ROOT / "outputs/lab-live/feed",
                    "board" if name == "entrance" else "desk",
                    i,
                    "Engineered socket feature capture; static simulation only",
                    "entrance-features-v1",
                )
                print("CAPTURED", filename, row["visible_pixels"], flush=True)
        (offline / "labels.json").write_text(json.dumps(dict(frames=frames, spec=spec), indent=2) + "\n")
        (sensor / "camera.json").write_text(
            json.dumps(
                dict(cameras={n: o for n, _, _, _, o in streams}, crop_xyxy=spec["crop_xyxy"]), indent=2
            )
        )
        (a.output / "capture-report.json").write_text(
            json.dumps(
                dict(
                    frames=len(frames),
                    seed=seed,
                    physics_elapsed_s=timeline.get_current_time() - initial,
                    spec=spec,
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                ),
                indent=2,
            )
        )
    except Exception as exc:
        exit_code = 1
        traceback.print_exc()
        (a.output / "invalid.json").write_text(json.dumps({"error": str(exc)}))
    finally:
        for _, product, rgb, seg, _ in streams:
            rgb.detach()
            seg.detach()
            product.destroy()
        owner = ROOT.stat()
        for path in a.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        os.chown(a.output, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
