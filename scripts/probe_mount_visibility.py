"""Offline visible-feature counts with and without the complete mounted tool."""

import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stage", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    if a.output.exists():
        p.error("Fresh output directory required")
    a.output.mkdir(parents=True)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root"],
        }
    )
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image
        from pxr import UsdGeom

        from ffc.entrance_features import label_leading_band

        ctx = omni.usd.get_context()
        ctx.open_stage(str(a.stage))
        for _ in range(8):
            app.update()
        stage = ctx.get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        cfg = json.loads((ROOT / "config/pi-zero-task.json").read_text())
        label_leading_band(stage, "/World/MacroTask/Cable", cfg)
        for name in ["MacroEnvelope", "MountMacroEnvelope"]:
            prim = stage.GetPrimAtPath("/World/Hardware/" + name)
            if prim and prim.IsActive():
                UsdGeom.Imageable(prim).MakeInvisible()
        cam = (
            "/World/Cameras/MountMacro"
            if stage.GetPrimAtPath("/World/Cameras/MountMacro")
            else "/World/Cameras/Macro"
        )
        rp = rep.create.render_product(cam, (2448, 2048))
        rgb = rep.AnnotatorRegistry.get_annotator("rgb")
        rgb.attach(rp)
        seg = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False})
        seg.attach(rp)
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        counts = {}
        for visible in [False, True]:
            tool = UsdGeom.Imageable(stage.GetPrimAtPath("/World/Tool"))
            (tool.MakeVisible if visible else tool.MakeInvisible)()
            for _ in range(3):
                rep.orchestrator.step(delta_time=0, rt_subframes=8, pause_timeline=True)
            data = seg.get_data()
            ids = np.asarray(data["data"]).reshape(2048, 2448)
            mask = np.zeros(ids.shape, np.uint8)
            for key, value in data["info"]["idToLabels"].items():
                path = str(value)
                cls = 0
                if "/Socket/Upper/Front" in path:
                    cls = 1
                elif "/Socket/Lower/Front" in path:
                    cls = 2
                elif "/Cable/LeadingBand/" in path:
                    cls = 3
                if cls:
                    mask[ids == int(key)] = cls
            name = "full-tool" if visible else "no-tool"
            counts[name] = {
                c: int((mask == i).sum())
                for i, c in enumerate(["background", "upper_rim", "lower_rim", "leading_band"])
                if i
            }
            Image.fromarray(np.asarray(rgb.get_data())[..., :3].astype(np.uint8)).save(
                a.output / (name + ".png")
            )
            Image.fromarray(mask).save(a.output / (name + "-offline.png"))
        assert timeline.get_current_time() == initial and not timeline.is_playing()
        result = dict(
            counts=counts,
            retention={c: counts["full-tool"][c] / v if v else None for c, v in counts["no-tool"].items()},
            scope="Offline pinhole label visibility, not optical resolution or learned localization",
            physics_elapsed_s=0,
        )
        (a.output / "report.json").write_text(json.dumps(result, indent=2) + "\n")
        print(result, flush=True)
        rgb.detach()
        seg.detach()
        rp.destroy()
    finally:
        owner = ROOT.stat()
        for f in a.output.rglob("*"):
            os.chown(f, owner.st_uid, owner.st_gid)
        os.chown(a.output, owner.st_uid, owner.st_gid)
        app.close()


if __name__ == "__main__":
    main()
