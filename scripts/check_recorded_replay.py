"""Open a physics-disabled replay in Isaac and render its start/end timeline states."""

import argparse
import json
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--stage", type=Path, required=True)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=False)
from isaacsim import SimulationApp  # noqa: E402

app = SimulationApp({"headless": True, "extra_args": ["--allow-root"]})
try:
    import numpy as np
    import omni.replicator.core as rep
    import omni.timeline
    import omni.usd
    from PIL import Image
    from pxr import UsdGeom

    omni.usd.get_context().open_stage(str(a.stage))
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    cameras = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdGeom.Camera)]
    camera = next((p for p in cameras if p.endswith("/Action")), cameras[0])
    product = rep.create.render_product(camera, (640, 480))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(product)
    timeline = omni.timeline.get_timeline_interface()
    timeline.pause()
    records = []
    for label, frame in [("start", stage.GetStartTimeCode()), ("end", stage.GetEndTimeCode())]:
        timeline.set_current_time(frame / stage.GetTimeCodesPerSecond())
        for _ in range(8):
            rep.orchestrator.step(delta_time=0.0, pause_timeline=True)
        pixels = np.asarray(rgb.get_data())[..., :3]
        if pixels.size == 0 or pixels.std() < 1:
            raise RuntimeError("Empty replay rendering")
        Image.fromarray(pixels).save(a.output / (label + ".png"))
        records.append({"label": label, "frame": frame, "pixel_std": float(pixels.std())})
    (a.output / "verification.json").write_text(
        json.dumps(
            {
                "stage": str(a.stage),
                "camera": camera,
                "mode": "Isaac paused timeline start/end render; not physics rerun",
                "frames": records,
            },
            indent=2,
        )
    )
finally:
    app.close()
