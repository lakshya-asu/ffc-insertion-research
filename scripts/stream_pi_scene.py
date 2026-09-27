"""Render a live, stationary Pi cell preview. No physics or robot actions."""

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT / "outputs/lab-live/feed")
    p.add_argument("--stage", type=Path, default=ROOT / "outputs/pi4-refined-002/raspberry-pi-workcell.usda")
    p.add_argument("--seconds", type=float, default=600)
    p.add_argument("--layout", type=Path, default=ROOT / "config/arducam-b0498-cell-v1.json")
    p.add_argument("--board-camera", default="insertion-a")
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=True)
    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "renderer": "RaytracedLighting",
            "anti_aliasing": 2,
            "extra_args": ["--allow-root", "--/telemetry/enableAnonymousData=false"],
        }
    )
    streams, exit_code, running = [], 0, True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        import numpy as np
        import omni.replicator.core as rep
        import omni.timeline
        import omni.usd
        from PIL import Image

        from ffc.camera_optics import apply_reference_optics
        from ffc.isaac_scene import camera

        context = omni.usd.get_context()
        context.open_stage(str(a.stage))
        for _ in range(8):
            app.update()
        stage = context.get_stage()
        profile = json.loads((ROOT / "config/arducam-b0498.json").read_text())
        layout = json.loads(a.layout.read_text())
        poses = {v["id"]: v for v in layout["cameras"]}
        views = {"workcell": ((1.4, -1.4, 1.25), (0.30, -0.02, 0.42), 28)}
        for name, pose in [("desk", poses["overview"]), ("board", poses[a.board_camera])]:
            views[name] = (pose["eye_m"], pose["target_m"], profile["focal_length_mm"])
        optics = {}
        for name, design in views.items():
            cam = camera(stage, "/World/Cameras/Live_" + name, *design)
            resolution = (960, 640) if name == "workcell" else (960, 540)
            if name != "workcell":
                optics[name] = apply_reference_optics(
                    stage,
                    cam,
                    profile,
                    float(np.linalg.norm(np.array(design[0]) - np.array(design[1]))),
                    resolution,
                )
            product = rep.create.render_product(str(cam.GetPath()), resolution)
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach(product)
            streams.append((name, product, annotator))
        timeline = omni.timeline.get_timeline_interface()
        timeline.pause()
        initial = timeline.get_current_time()
        start, sequence = time.monotonic(), 0
        while (
            running
            and time.monotonic() - start < a.seconds
            and not (a.output.parent / "stop-preview").exists()
        ):
            rep.orchestrator.step(delta_time=0.0, rt_subframes=2, pause_timeline=True)
            if timeline.is_playing() or timeline.get_current_time() != initial:
                raise RuntimeError("Physics advanced")
            for name, _, annotator in streams:
                raw = np.asarray(annotator.get_data())[..., :3].astype(np.uint8)
                tmp = a.output / (name + ".jpg.tmp")
                Image.fromarray(raw).save(tmp, format="JPEG", quality=88)
                os.replace(tmp, a.output / (name + ".jpg"))
                metadata = {
                    "captured_at": time.time(),
                    "sequence": sequence,
                    "source": "Isaac Sim live static preview",
                    "description": (
                        "Stationary scene, freshly rendered; zero physics steps. Not a manipulation rollout."
                    ),
                    "camera": name,
                    "width": int(raw.shape[1]),
                    "height": int(raw.shape[0]),
                    "reference_optics": optics.get(name),
                    "motion_permitted": False,
                }
                metadata["workpiece"] = layout.get("board", "Raspberry Pi 4 Model B")
                metadata["description"] = metadata["workpiece"] + ". " + metadata["description"]
                if name in optics:
                    metadata["description"] += (
                        " Arducam B0498 reference, manual 16 mm lens; 960 × 540 preview of a"
                        " 3840 × 2160 field. Ideal projection; blur and rolling readout uncalibrated."
                    )
                tmp = a.output / (name + ".json.tmp")
                tmp.write_text(json.dumps(metadata))
                os.replace(tmp, a.output / (name + ".json"))
            sequence += 1
            time.sleep(0.5)
    except Exception:
        exit_code = 1
        import traceback

        traceback.print_exc()
        raise
    finally:
        for _, product, annotator in streams:
            annotator.detach()
            product.destroy()
        app.close(exit_code=exit_code)


if __name__ == "__main__":
    main()
