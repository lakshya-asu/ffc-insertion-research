"""Render a recorded insertion bench from alternative human-review cameras."""

import argparse
import json
import sys
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--stage", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--socket-macro", action="store_true", help="Add a frontal entrance review camera")
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "extra_args": ["--allow-root"]})
try:
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from PIL import Image
    from pxr import UsdPhysics

    from ffc.isaac_scene import camera

    omni.usd.get_context().open_stage(str(a.stage))
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    for prim in stage.Traverse():
        if (
            prim.HasAPI(UsdPhysics.RigidBodyAPI)
            and UsdPhysics.RigidBodyAPI(prim).GetRigidBodyEnabledAttr().Get()
        ):
            raise ValueError("Expected disabled playback physics")
    poses = [(0.045, 0.012, 0.068), (-0.045, 0.012, 0.068), (0.02, 0.045, 0.075)]
    if a.socket_macro:
        poses.append((0, 0.030, 0.0575))
    annotators = []
    for i, eye in enumerate(poses):
        path = f"/World/Cameras/Review{i}"
        camera(stage, path, eye, (0, -0.001, 0.055), 48)
        product = rep.create.render_product(path, (960, 720))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach(product)
        annotators.append(ann)
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    reviews = []
    for label, stamp in [("start", stage.GetStartTimeCode()), ("end", stage.GetEndTimeCode())]:
        timeline.set_current_time(stamp / stage.GetTimeCodesPerSecond())
        for _ in range(8):
            rep.orchestrator.step(delta_time=0, pause_timeline=True)
        for i, ann in enumerate(annotators):
            pixels = np.asarray(ann.get_data())[..., :3]
            if pixels.size == 0:
                raise ValueError("Empty rendering")
            Image.fromarray(pixels).save(a.output / f"{label}-{i}.png")
            reviews.append(
                {
                    "label": label,
                    "camera": i,
                    "pixel_std": float(pixels.std()),
                    "informative_pixels": bool(pixels.std() >= 1),
                }
            )
    (a.output / "review.json").write_text(
        json.dumps(
            {
                "mode": "Recorded-state start/end review; no physics rerun",
                "eyes_m": poses,
                "target_m": [0, -0.001, 0.055],
                "geometry_hidden": False,
                "views": reviews,
            },
            indent=2,
        )
    )
except Exception:
    import traceback

    (a.output / "failure.txt").write_text(traceback.format_exc())
    raise
finally:
    app.close()
