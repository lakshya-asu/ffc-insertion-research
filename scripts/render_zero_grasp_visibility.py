"""Offline static finger-envelope sweep; no grasp mechanics or runtime observations."""

import argparse
import hashlib
import itertools
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
        p.error("A fresh output directory is required")
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
    streams, exit_code = [], 0
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import Gf, UsdGeom

        from ffc.pi_zero_scene import box

        cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        context = omni.usd.get_context()
        context.open_stage(str(ROOT / "outputs/pi-zero-review-002/pi-zero-workcell.usda"))
        for _ in range(8):
            app.update()
        stage = context.get_stage()
        # All edits live in an anonymous session layer; the source stage is never saved.
        stage.SetEditTarget(stage.GetSessionLayer())
        UsdGeom.Imageable(stage.GetPrimAtPath("/World/Hardware/ZeroJawProbe")).MakeInvisible()
        jaws = UsdGeom.Xform.Define(stage, "/World/VisibilityJaws")
        for i, sign in enumerate([-1, 1]):
            box(
                stage,
                f"/World/VisibilityJaws/Jaw{i}",
                (0, 0, sign),
                (0.004, 0.012, 0.003),
                (0.16, 0.18, 0.2),
                collision=False,
            )
        for name in ["entrance", "offset"]:
            product = rep.create.render_product("/World/Cameras/Zero_" + name, (3840, 2160))
            rgb = rep.AnnotatorRegistry.get_annotator("rgb")
            seg = rep.AnnotatorRegistry.get_annotator(
                "instance_id_segmentation", init_params={"colorize": False}
            )
            rgb.attach(product)
            seg.attach(product)
            streams.append((name, product, rgb, seg))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        cases = []
        for gap in [15, 3, 1]:
            cases.append(dict(gap_mm=gap, jaws=False, setback_mm=0, width_mm=0, thickness_mm=0))
            for setback, width, thickness in itertools.product([6, 10, 15], [8, 12], [1.5, 3]):
                cases.append(
                    dict(gap_mm=gap, jaws=True, setback_mm=setback, width_mm=width, thickness_mm=thickness)
                )
        frames, baselines = [], {}
        for idx, case in enumerate(cases):
            tip = np.array(cfg["nominal_mouth_review_point_m"]) + [case["gap_mm"] / 1000, 0, 0]
            cable = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Hardware/ZeroCable"))
            cable.ClearXformOpOrder()
            cable.AddTranslateOp().Set(Gf.Vec3d(*tip))
            (UsdGeom.Imageable(jaws).MakeVisible if case["jaws"] else UsdGeom.Imageable(jaws).MakeInvisible)()
            for i, sign in enumerate([-1, 1]):
                jaw = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/VisibilityJaws/Jaw{i}"))
                jaw.ClearXformOpOrder()
                jaw.AddTranslateOp().Set(
                    Gf.Vec3d(
                        *(
                            tip
                            + [case["setback_mm"] / 1000, 0, sign * (case["thickness_mm"] / 2000 + 0.00025)]
                        )
                    )
                )
                shape = UsdGeom.Xformable(stage.GetPrimAtPath(f"/World/VisibilityJaws/Jaw{i}/Shape"))
                shape.ClearXformOpOrder()
                shape.AddScaleOp().Set(
                    Gf.Vec3f(0.004, max(case["width_mm"], 1) / 1000, max(case["thickness_mm"], 1) / 1000)
                )
            for _ in range(4):
                app.update()
            for _ in range(3):
                rep.orchestrator.step(delta_time=0.0, rt_subframes=4, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            for camera, _, rgb, seg in streams:
                arr = np.asarray(rgb.get_data())[..., :3].astype(np.uint8)
                data = seg.get_data()
                ids = np.asarray(data["data"]).reshape(2160, 3840)
                mask = np.zeros(ids.shape, np.uint8)
                for key, value in data["info"]["idToLabels"].items():
                    path, cls = str(value), 0
                    if path.startswith("/World/Hardware/ZeroCable/Mini"):
                        cls = 2
                    elif path.startswith("/World/Hardware/Zero2W/"):
                        prim = stage.GetPrimAtPath(path)
                        while prim and str(prim.GetPath()).startswith("/World/Hardware/Zero2W/"):
                            if "22 Pin FPC Connector" in (prim.GetCustomDataByKey("cad_component") or ""):
                                cls = 1
                                break
                            prim = prim.GetParent()
                    if cls:
                        mask[ids == int(key)] = cls
                counts = {"connector": int((mask == 1).sum()), "mini_end": int((mask == 2).sum())}
                key = (case["gap_mm"], camera)
                if not case["jaws"]:
                    if min(counts.values()) == 0:
                        raise RuntimeError(f"Empty baseline: {key}: {counts}")
                    baselines[key] = counts
                retention = {k: counts[k] / baselines[key][k] for k in counts}
                filename = f"{idx:03d}-{camera}.png"
                if arr.shape != (2160, 3840, 3) or arr.std() < 5:
                    raise RuntimeError("Invalid RGB")
                Image.fromarray(arr).save(args.output / filename)
                Image.fromarray(mask).save(args.output / (filename[:-4] + "-offline.png"))
                frames.append(
                    dict(
                        file=filename,
                        camera=camera,
                        case=case,
                        pixels=counts,
                        retention_vs_no_jaws=retention,
                        sha256=hashlib.sha256((args.output / filename).read_bytes()).hexdigest(),
                    )
                )
                print("RENDERED", filename, counts, retention, flush=True)
        report = dict(
            frames=frames,
            cases=cases,
            configuration=cfg,
            motion_permitted=False,
            physics_elapsed_s=timeline.get_current_time() - initial,
            scope="Offline component visibility, not aperture observability or stable grasp",
            jaw_length_mm=4,
            inner_gap_mm=0.5,
            limitations=[
                "Rigid flat cable",
                "Unverified mouth review point and latch",
                "Whole-component pixel retention does not measure entrance rim or leading-edge visibility",
                "Ideal reference optics; real lens focus unverified",
                "No contact, grip force, slip or deformation",
            ],
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        )
        (args.output / "review.json").write_text(json.dumps(report, indent=2) + "\n")
    except Exception as exc:
        exit_code = 1
        traceback.print_exc()
        (args.output / "invalid.json").write_text(json.dumps({"error": str(exc)}))
    finally:
        for _, product, rgb, seg in streams:
            rgb.detach()
            seg.detach()
            product.destroy()
        owner = ROOT.stat()
        for path in args.output.rglob("*"):
            os.chown(path, owner.st_uid, owner.st_gid)
        os.chown(args.output, owner.st_uid, owner.st_gid)
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
