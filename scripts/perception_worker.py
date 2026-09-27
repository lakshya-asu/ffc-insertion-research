"""Image-only worker. Run in a container with /sensor, /code and /results only.

Simulation labels/scene/configuration are not mounted. Pixel masks written here
are predictions, never renderer annotations. Replay time validates serialization;
this worker does not establish live acquisition latency.
"""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
from cable_perception import detect_cable
from camera_observations import CameraGate, CameraSpec
from PIL import Image


def main():
    source, destination = Path("/sensor"), Path("/results")
    calibration = json.loads((source / "calibration.json").read_text())
    gate = CameraGate(
        {name: CameraSpec(c["width"], c["height"], c["calibration_id"]) for name, c in calibration.items()}
    )
    manifest = json.loads((source / "frames.json").read_text())
    model = (
        json.loads((source / "pixel-model.json").read_text())
        if (source / "pixel-model.json").exists()
        else None
    )
    predictions = []
    for entry in manifest["frames"]:
        name = entry["file"]
        if Path(name).name != name:
            raise ValueError("Nested or escaping image path")
        path = source / name
        if hashlib.sha256(path.read_bytes()).hexdigest() != entry["sha256"]:
            raise ValueError("Image hash mismatch")
        with Image.open(path) as img:
            if img.mode != "RGB" or img.size != (entry["width"], entry["height"]):
                raise ValueError("Invalid RGB file")
            packet = {
                key: entry[key]
                for key in ["camera_id", "sequence", "timestamp_s", "width", "height", "calibration_id"]
            }
            packet["rgb"] = img.tobytes()
        frame = gate.accept(packet, entry["timestamp_s"])
        pixels = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3)
        start = time.perf_counter()
        prediction, mask = detect_cable(pixels, model)
        prediction["inference_ms"] = (time.perf_counter() - start) * 1000
        prediction["file"] = name
        prediction["mask"] = Path(name).stem + "-prediction.png"
        Image.fromarray((mask * 255).astype(np.uint8)).save(destination / prediction["mask"])
        predictions.append(prediction)
    result = {
        "algorithm": "quadratic RGB classifier + candidate rejection v3"
        if model
        else "frozen color/shape baseline v1",
        "input": "RGB8 only",
        "clock_mode": "offline replay",
        "robot_actions": 0,
        "trained": model is not None,
        "model_sha256": hashlib.sha256((source / "pixel-model.json").read_bytes()).hexdigest()
        if model
        else None,
        "predictions": predictions,
    }
    (destination / "predictions.json").write_text(json.dumps(result, indent=2))
    print(json.dumps({"images": len(predictions), "detections": sum(p["detected"] for p in predictions)}))


if __name__ == "__main__":
    main()
