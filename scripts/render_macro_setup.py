"""Render selected macro setup and a static near-entry visibility/defocus study."""

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
    a = p.parse_args()
    if a.output.exists():
        p.error("Use a fresh output directory")
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

        from ffc.entrance_features import make_socket, set_slider
        from ffc.isaac_scene import camera
        from ffc.macro_camera import add_macro_camera
        from ffc.pi_zero_scene import box, make_zero_cable

        cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        spec = json.loads((ROOT / "config/zero-feature-v1.json").read_text())
        profile = json.loads((ROOT / "config/macro-basler-kowa.json").read_text())
        context = omni.usd.get_context()
        context.open_stage(str(ROOT / "outputs/pi-zero-review-002/pi-zero-workcell.usda"))
        for _ in range(8):
            app.update()
        stage = context.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        for name in ["Zero2W", "ZeroCable", "ZeroJawProbe"]:
            stage.GetPrimAtPath("/World/Hardware/" + name).SetActive(False)
        root = "/World/MacroTask"
        task = UsdGeom.Xform.Define(stage, root)
        task.AddTranslateOp().Set(Gf.Vec3d(*cfg["board_translation_m"]))
        board = UsdGeom.Xform.Define(stage, root + "/Board")
        board.GetPrim().GetReferences().AddReference(str(ROOT / cfg["asset"]))
        stage.GetPrimAtPath(root + "/Board/Components/Part_0010").SetActive(False)
        make_socket(stage, root + "/Socket", spec)
        UsdGeom.Xformable(stage.GetPrimAtPath(root + "/Socket")).AddTranslateOp().Set(
            Gf.Vec3d(0.0333, -0.0008, 0.0022)
        )
        set_slider(stage, root + "/Socket", spec, True)
        make_zero_cable(stage, cfg, root + "/Cable")
        for j in range(2):
            box(stage, root + f"/Jaw{j}", (0, 0, 0), (0.004, 0.008, 0.0015), (0.16, 0.18, 0.20))
        macro, envelope, optics = add_macro_camera(stage, profile, spec["mouth_m"])
        overview = camera(stage, "/World/Cameras/MacroOverview", [0.93, -0.42, 0.34], [0.67, -0.12, 0.06], 40)
        for name, cam, size in [
            ("macro", macro, tuple(profile["native_resolution"])),
            ("placement", overview, (1280, 960)),
        ]:
            rp = rep.create.render_product(str(cam.GetPath()), size)
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb.attach(rp)
            streams.append((name, rp, rgb))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        frames = []
        for gap in [6, 3, 1, 0.3]:
            tip = np.array([0.0333 + gap / 1000, -0.0008, 0.0022])
            cable = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/Cable"))
            cable.ClearXformOpOrder()
            cable.AddTranslateOp().Set(Gf.Vec3d(*tip))
            for j, sign in enumerate([-1, 1]):
                jaw = UsdGeom.Xformable(stage.GetPrimAtPath(root + f"/Jaw{j}"))
                jaw.ClearXformOpOrder()
                jaw.AddTranslateOp().Set(Gf.Vec3d(*(tip + [0.010, 0, sign * 0.001])))
            for mode in ["ideal", "finite-aperture"]:
                macro.GetFStopAttr().Set(0 if mode == "ideal" else optics["equivalent_renderer_f_number"])
                UsdGeom.Imageable(envelope).MakeInvisible()
                for _ in range(3):
                    rep.orchestrator.step(delta_time=0, rt_subframes=16, pause_timeline=True)
                arr = np.asarray(streams[0][2].get_data())[..., :3].astype(np.uint8)
                if arr.shape != (2048, 2448, 3) or arr.std() < 5:
                    raise RuntimeError("Invalid macro RGB")
                name = f"gap-{gap:g}-{mode}.png"
                Image.fromarray(arr).save(a.output / name)
                frames.append(dict(file=name, gap_mm=gap, optics=mode))
                print("RENDERED", name, flush=True)
            UsdGeom.Imageable(envelope).MakeVisible()
            for _ in range(2):
                rep.orchestrator.step(delta_time=0, rt_subframes=8, pause_timeline=True)
            Image.fromarray(np.asarray(streams[1][2].get_data())[..., :3].astype(np.uint8)).save(
                a.output / f"placement-{gap:g}.png"
            )
        if timeline.get_current_time() != initial or timeline.is_playing():
            raise RuntimeError("Physics advanced during static capture")
        # Persist all edits to a new, self-contained composed scene. Source assets remain untouched.
        export_stage = Usd.Stage.Open(stage.Flatten())
        # Replicator graphs are process-local and must be rebuilt by the next renderer.
        for transient in ["/Render", "/Replicator"]:
            export_stage.RemovePrim(transient)
        export_stage.GetRootLayer().Export(str(a.output / "macro-workcell.usda"))
        report = dict(
            profile=profile,
            optics=optics,
            frames=frames,
            physics_elapsed_s=0,
            motion_permitted=False,
            limitations=[
                "Static rigid cable and finger proxies; no contact or grasp qualification",
                "Engineered connector opening uses assumed dimensions",
                "Finite-aperture image is an uncalibrated sensitivity test; no measured MTF",
                "Macro envelope hidden during its own exposure: lens geometry is a solid placeholder",
                "No detector inference: macro optics require a separately evaluated model",
            ],
        )
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
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
    if (a.output / "invalid.json").exists():
        raise SystemExit(1)


if __name__ == "__main__":
    main()
