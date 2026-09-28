"""Native mounted-camera dataset with full-tool offline IK; never runtime state input."""

import argparse
import copy
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
    p.add_argument("--split", choices=["audit", "development", "test"], required=True)
    p.add_argument("--count", type=int)
    p.add_argument("--frozen-model", type=Path)
    p.add_argument("--paired-setback", action="store_true", help="Audit-only matched 6/10 mm tool pairs")
    a = p.parse_args()
    if a.paired_setback and a.split != "audit":
        p.error("Matched pairs are audit only")
    if a.output.exists():
        p.error("Fresh output directory required")
    frozen_hash = None
    if a.split == "test":
        if not a.frozen_model:
            p.error("Final test capture requires --frozen-model")
        frozen = json.loads((a.frozen_model / "frozen.json").read_text())
        frozen_hash = hashlib.sha256((a.frozen_model / "member0.pt").read_bytes()).hexdigest()
        if frozen_hash != frozen["model_sha256"]:
            p.error("Frozen checkpoint hash mismatch")
    sensor, offline = a.output / "sensor", a.output / "offline"
    sensor.mkdir(parents=True)
    offline.mkdir()
    snapshots = offline / "source"
    snapshots.mkdir()
    for source in [
        Path(__file__),
        ROOT / "config/macro-dataset-v1.json",
        ROOT / "config/macro-mounted-camera.json",
        ROOT / "config/pi-zero-task.json",
        ROOT / "config/zero-feature-v1.json",
        ROOT / "src/ffc/entrance_features.py",
        ROOT / "src/ffc/isaac_kinematics.py",
        ROOT / "src/ffc/mount_clearance.py",
    ]:
        (snapshots / source.name).write_bytes(source.read_bytes())
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
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom
        from scipy.spatial.transform import Rotation

        from ffc.entrance_features import label_leading_band, set_slider
        from ffc.isaac_kinematics import from_stage
        from ffc.lab_feed import publish_snapshot
        from ffc.mount_clearance import Box, link_poses, separation

        cfg = json.loads((ROOT / "config/macro-dataset-v1.json").read_text())
        cable_cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        spec = json.loads((ROOT / "config/zero-feature-v1.json").read_text())
        mount = json.loads((ROOT / "outputs/mount-clearance-004/report.json").read_text())
        seed = cfg[a.split + "_seed"]
        rng = np.random.default_rng(seed)
        count = a.count or cfg[a.split + "_scenes"]
        stage_path = ROOT / "outputs/mount-review-003/mounted-workcell.usda"
        ctx = omni.usd.get_context()
        ctx.open_stage(str(stage_path))
        for _ in range(8):
            app.update()
        stage = ctx.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        root = "/World/MacroTask"
        label_leading_band(stage, root + "/Cable", cable_cfg)
        cache = UsdGeom.XformCache()

        def world(path):
            cache.Clear()
            return np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path))).T

        def set_world(path, transform):
            prim = stage.GetPrimAtPath(path)
            cache.Clear()
            parent = np.array(cache.GetLocalToWorldTransform(prim.GetParent())).T
            x = UsdGeom.Xformable(prim)
            x.ClearXformOpOrder()
            x.AddTransformOp().Set(Gf.Matrix4d((np.linalg.inv(parent) @ transform).T.tolist()))

        def set_local(path, transform):
            x = UsdGeom.Xformable(stage.GetPrimAtPath(path))
            x.ClearXformOpOrder()
            x.AddTransformOp().Set(Gf.Matrix4d(transform.T.tolist()))

        def visible(path, value):
            im = UsdGeom.Imageable(stage.GetPrimAtPath(path))
            (im.MakeVisible if value else im.MakeInvisible)()

        base_task = world(root)
        tool_bind = np.linalg.inv(world(mount["link_paths"][-1])) @ world("/World/Tool")
        chain = from_stage(stage)
        near = next(row for row in mount["paths"] if row["segment"] == "near_entry")
        q = np.array(near["q"])
        r0 = np.array(mount["tool_rotation"])
        point = np.array(mount["reference_point_link7_m"])
        obstacles = [
            Box(x["name"], np.array(x["center"]), np.array(x["half"]), np.array(x["axes"]))
            for x in mount["candidates"][1]["obstacles"]
        ]
        bounds = [
            Box(x["name"], np.array(x["center"]), np.array(x["half"]), np.array(x["axes"]))
            for x in mount["local_tool_boxes"]
        ]
        cam = UsdGeom.Camera.Get(stage, "/World/Cameras/MountMacro")
        focus = cam.GetFocusDistanceAttr().Get()
        k = [
            2448 * cam.GetFocalLengthAttr().Get() / cam.GetHorizontalApertureAttr().Get(),
            0,
            1224,
            0,
            2048 * cam.GetFocalLengthAttr().Get() / cam.GetVerticalApertureAttr().Get(),
            1024,
            0,
            0,
            1,
        ]
        (sensor / "camera.json").write_text(
            json.dumps(
                dict(
                    width=2448,
                    height=2048,
                    k=k,
                    d=[0] * 5,
                    distortion_model="plumb_bob",
                    frame_id="macro_optical_frame",
                    profile=cfg["camera_profile"],
                    world_optical=(world("/World/Cameras/MountMacro") @ np.diag([1, -1, -1, 1])).tolist(),
                    preprocessing_revision=cfg["preprocessing_revision"],
                ),
                indent=2,
            )
        )
        visible("/World/Hardware/MountMacroEnvelope", False)
        rp = rep.create.render_product("/World/Cameras/MountMacro", (2448, 2048))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        seg = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False})
        seg.attach(rp)
        streams.append((rp, rgb, seg))
        dome = stage.GetPrimAtPath("/World/Lighting/Dome").GetAttribute("inputs:intensity")
        dome_initial = dome.Get()
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        rows = []
        # Independent latch sampling avoids learning the condition index as a latch label.
        for i in range(count):
            condition = cfg["conditions"][i % len(cfg["conditions"])]
            if a.paired_setback:
                if i % 2 == 0:
                    pair_state = copy.deepcopy(rng.bit_generator.state)
                else:
                    rng.bit_generator.state = copy.deepcopy(pair_state)
                condition = "short_grasp" if i % 2 == 0 else "paired_10mm"
            opened = bool(rng.integers(2))
            t = base_task.copy()
            yaw = float(rng.uniform(-3, 3) if condition == "board_rotation" else rng.uniform(-0.6, 0.6))
            t[:3, :3] = Rotation.from_euler("z", yaw, degrees=True).as_matrix() @ base_task[:3, :3]
            t[:3, 3] += rng.uniform([-0.00025, -0.0004, 0], [0.00025, 0.0004, 0.00015])
            set_world(root, t)
            gap = float(
                rng.uniform(0.0005, 0.001) if condition == "near_entry" else rng.uniform(0.001, 0.006)
            )
            cable_yaw = float(rng.uniform(-2, 2))
            cable = np.eye(4)
            cable[:3, :3] = Rotation.from_euler("z", cable_yaw, degrees=True).as_matrix()
            cable[:3, 3] = [
                0.0333 + gap,
                -0.0008 + rng.uniform(-0.0004, 0.0004),
                0.0022 + rng.uniform(0, 0.0002),
            ]
            set_local(root + "/Cable", cable)
            # A conservative leading-corner gap check; does not certify full-cell collisions.
            corner_gap = gap - 0.00575 * abs(np.sin(np.deg2rad(cable_yaw)))
            if corner_gap < 0.0001:
                raise RuntimeError("Leading corner crosses precontact boundary")
            setback = 0.006 if condition == "short_grasp" else 0.010
            target = (t @ cable @ np.array([setback, 0, 0, 1]))[:3]
            desired_r = t[:3, :3] @ cable[:3, :3] @ r0
            q, metric = chain.solve(target, desired_r, q, point, rng)
            poses = link_poses(chain, q)
            for path, pose in zip(mount["link_paths"], poses, strict=True):
                set_world(path, pose)
            set_world("/World/Tool", poses[-1] @ tool_bind)
            bound = min(separation(b.transformed(poses[-1]), ob) for b in bounds for ob in obstacles)
            if bound < 0.005:
                raise RuntimeError(f"Tool / mount bound below 5 mm: {bound}")
            cable_present = condition not in ["absent_cable", "empty_targets"]
            socket_present = condition not in ["absent_socket", "empty_targets"]
            visible(root + "/Cable", cable_present)
            visible(root + "/Socket", socket_present)
            set_slider(stage, root + "/Socket", spec, opened)
            scale = 0.12 if condition == "dim" else (4.0 if condition == "bright" else 1.0)
            dome.Set(dome_initial * scale)
            focus_delta = float(rng.choice([-1, 1]) * 0.0008) if condition == "focus_drift" else 0.0
            cam.GetFocusDistanceAttr().Set(focus + focus_delta)
            for _ in range(3):
                rep.orchestrator.step(delta_time=0, rt_subframes=8, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            if rep.orchestrator.get_status() == rep.orchestrator.Status.STEPPING:
                raise RuntimeError("Capture stalled")
            arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
            data = seg.get_data()
            ids = np.asarray(data["data"]).reshape(2048, 2448)
            mask = np.zeros(ids.shape, np.uint8)
            for key, val in data["info"]["idToLabels"].items():
                path = str(val)
                cls = 0
                if path == root + "/Socket/Upper/Front":
                    cls = 1
                elif path == root + "/Socket/Lower/Front":
                    cls = 2
                elif path.startswith(root + "/Cable/LeadingBand/"):
                    cls = 3
                elif path.startswith(root + "/Socket/Slider/"):
                    cls = 4 if opened else 5
                if cls:
                    mask[ids == int(key)] = cls
            if not cable_present and (mask == 3).any():
                raise RuntimeError("Absent cable has labels")
            if not socket_present and np.isin(mask, [1, 2, 4, 5]).any():
                raise RuntimeError("Absent socket has labels")
            filename = f"{i:04d}-macro.png"
            Image.fromarray(arr).save(sensor / filename)
            Image.fromarray(mask).save(offline / filename)
            split = (
                "audit"
                if a.split == "audit"
                else ("validation" if i >= int(count * (1 - cfg["validation_fraction"])) else "train")
            )
            if a.split == "test":
                split = "test"
            rows.append(
                dict(
                    file=filename,
                    scene=i,
                    split=split,
                    condition=condition,
                    slider_open=opened,
                    visible_pixels=np.bincount(mask.ravel(), minlength=6).tolist(),
                    rgb_sha256=hashlib.sha256((sensor / filename).read_bytes()).hexdigest(),
                    mask_sha256=hashlib.sha256((offline / filename).read_bytes()).hexdigest(),
                    authored=dict(
                        board_world=t.tolist(),
                        cable_local=cable.tolist(),
                        gap_m=gap,
                        focus_delta_m=focus_delta,
                        dome_scale=scale,
                        setback_m=setback,
                        tool_mount_gap_m=bound,
                        ik=metric,
                    ),
                )
            )
            (offline / "labels.json").write_text(json.dumps(dict(frames=rows, config=cfg), indent=2))
            publish_snapshot(
                arr,
                ROOT / "outputs/lab-live/feed",
                "board",
                i,
                "Mounted macro dataset; full tool; static offline supervision",
                cfg["revision"],
            )
            print("CAPTURED", filename, condition, rows[-1]["visible_pixels"], flush=True)
        (a.output / "capture-report.json").write_text(
            json.dumps(
                dict(
                    frames=count,
                    seed=seed,
                    split=a.split,
                    physics_elapsed_s=0,
                    motion_permitted=False,
                    stage_sha256=hashlib.sha256(stage_path.read_bytes()).hexdigest(),
                    script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                    scope=cfg["scope"],
                    paired_setback=a.paired_setback,
                    frozen_model_sha256=frozen_hash,
                ),
                indent=2,
            )
        )
    except Exception as exc:
        traceback.print_exc()
        (a.output / "invalid.json").write_text(json.dumps({"error": str(exc)}))
    finally:
        for product, rgb, seg in streams:
            rgb.detach()
            seg.detach()
            product.destroy()
        owner = ROOT.stat()
        for path in a.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        os.chown(a.output, owner.st_uid, owner.st_gid)
        app.close()


if __name__ == "__main__":
    main()
