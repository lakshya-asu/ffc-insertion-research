"""Static CAD review of mounting hardware and offline IK poses; no physics rollout."""

import argparse
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
    p.add_argument("--report", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error("Fresh output required")
    a.output.mkdir(parents=True)
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
        from pxr import Gf, Usd, UsdGeom

        from ffc.isaac_scene import camera
        from ffc.macro_camera import add_macro_camera
        from ffc.pi_zero_scene import box

        report = json.loads(a.report.read_text())
        selected = report["candidates"][1]
        ctx = omni.usd.get_context()
        ctx.open_stage(str(ROOT / "outputs/macro-preview-scene-002/macro-workcell.usda"))
        for _ in range(8):
            app.update()
        stage = ctx.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        stage.GetPrimAtPath("/World/Hardware/MacroEnvelope").SetActive(False)
        _, macro_envelope, _ = add_macro_camera(
            stage,
            report["profile"],
            report["mouth_m"],
            "/World/Cameras/MountMacro",
            "/World/Hardware/MountMacroEnvelope",
        )
        macro_cam = UsdGeom.Camera.Get(stage, "/World/Cameras/MountMacro")
        macro_cam.GetFStopAttr().Set(
            report["profile"]["starting_settings"]["physical_f_number"]
            * (1 + report["profile"]["magnification"])
        )
        cache = UsdGeom.XformCache()
        UsdGeom.Xform.Define(stage, "/World/Tool")
        original = np.asarray(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(report["link_paths"][-1]))).T
        for i in range(2):
            stage.GetPrimAtPath("/World/MacroTask/Jaw" + str(i)).SetActive(False)
        box(stage, "/World/Mount/Baseplate", (0.71, -0.14, -0.005), (0.5, 0.46, 0.01), (0.23, 0.25, 0.28))
        for item in selected["obstacles"]:
            if item["name"] in ["lens_keepout", "camera_body"]:
                continue
            axes = np.array(item["axes"])
            center = np.array(item["center"])
            size = np.array(item["half"]) * 2
            prim = box(
                stage,
                "/World/Mount/" + item["name"],
                (0, 0, 0),
                tuple(size),
                (0.09, 0.10, 0.12) if "usb" in item["name"] else (0.28, 0.32, 0.36),
            )
            t = np.eye(4)
            t[:3, :3] = axes
            t[:3, 3] = center
            x = UsdGeom.Xformable(prim)
            x.ClearXformOpOrder()
            x.AddTransformOp().Set(Gf.Matrix4d(t.T.tolist()))
        camera(stage, "/World/Cameras/MountReview", (0.98, -0.63, 0.43), (0.69, -0.13, 0.105), 45)
        camera(stage, "/World/Cameras/MountCell", (1.25, -1.20, 1.03), (0.36, -0.04, 0.4), 32)
        for name, path, size in [
            ("placement", "MountReview", (1280, 960)),
            ("cell", "MountCell", (1280, 960)),
            ("macro", "MountMacro", (2448, 2048)),
        ]:
            rp = rep.create.render_product("/World/Cameras/" + path, size)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            streams.append((name, rp, rgb))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        # Endpoints plus the worst passing sample; no apparent interpolated robot motion.
        indices = [0]
        for i, row in enumerate(report["paths"][:-1]):
            if row["segment"] != report["paths"][i + 1]["segment"]:
                indices.append(i)
        indices.extend([len(report["paths"]) - 1, selected["worst_sample"]])
        indices = sorted(set(indices))
        frames = []
        for frame, idx in enumerate(indices):
            row = report["paths"][idx]
            for path, world in zip(report["link_paths"], row["link_poses"], strict=True):
                prim = stage.GetPrimAtPath(path)
                cache.Clear()
                parent = np.asarray(cache.GetLocalToWorldTransform(prim.GetParent())).T
                local = np.linalg.inv(parent) @ np.array(world)
                x = UsdGeom.Xformable(prim)
                x.ClearXformOpOrder()
                x.AddTransformOp().Set(Gf.Matrix4d(local.T.tolist()))
            delta = np.array(row["tool_pose"]) @ np.linalg.inv(original)
            x = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Tool"))
            x.ClearXformOpOrder()
            x.AddTransformOp().Set(Gf.Matrix4d(delta.T.tolist()))
            offset = np.array(row["offset_m"]) if row["group"] == "cable" else np.array([0.0003, 0, 0])
            cable = UsdGeom.Xformable(stage.GetPrimAtPath("/World/MacroTask/Cable"))
            cable.ClearXformOpOrder()
            cable.AddTranslateOp().Set(Gf.Vec3d(*(np.array([0.0333, -0.0008, 0.0022]) + offset)))
            envelope = UsdGeom.Imageable(macro_envelope)
            for macro in [False, True]:
                (envelope.MakeInvisible if macro else envelope.MakeVisible)()
                for _ in range(3):
                    rep.orchestrator.step(delta_time=0, rt_subframes=8, pause_timeline=True)
                for name, _, rgb in streams:
                    if (name == "macro") != macro:
                        continue
                    arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                    filename = f"{frame:02d}-{name}.png"
                    Image.fromarray(arr).save(a.output / filename)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            frames.append(
                dict(
                    frame=frame,
                    sample=idx,
                    segment=row["segment"],
                    group=row["group"],
                    offset_m=row["offset_m"],
                )
            )
            print("RENDERED", frame, row["segment"], flush=True)
            if row["segment"] == "near_entry":
                envelope.MakeVisible()
                export = Usd.Stage.Open(stage.Flatten())
                for path in ["/Render", "/Replicator", "/Orchestrator"]:
                    export.RemovePrim(path)
                export.GetRootLayer().Export(str(a.output / "mounted-workcell.usda"))
        (a.output / "review.json").write_text(
            json.dumps(dict(frames=frames, physics_elapsed_s=0, motion_permitted=False), indent=2) + "\n"
        )
    except Exception as exc:
        traceback.print_exc()
        (a.output / "invalid.json").write_text(json.dumps({"error": str(exc)}))
    finally:
        for _, rp, rgb in streams:
            rgb.detach()
            rp.destroy()
        owner = ROOT.stat()
        for path in a.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        os.chown(a.output, owner.st_uid, owner.st_gid)
        app.close()


if __name__ == "__main__":
    main()
