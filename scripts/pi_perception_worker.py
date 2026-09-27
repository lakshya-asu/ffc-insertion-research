"""Isolated replay inference: mounted RGB, camera contract and frozen weights only."""

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from camera_observations import CameraGate, CameraSpec
from pi_perception_model import CLASSES, PiPerception
from PIL import Image
from scipy import ndimage


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sensor", type=Path, default=Path("/sensor"))
    p.add_argument("--model", type=Path, default=Path("/model/model.pt"))
    p.add_argument("--output", type=Path, default=Path("/results"))
    a = p.parse_args()
    torch.set_num_threads(4)
    model = PiPerception()
    checkpoint = torch.load(a.model, map_location="cpu", weights_only=True)
    if tuple(checkpoint["classes"]) != CLASSES:
        raise ValueError("Checkpoint class order mismatch")
    model.load_state_dict(checkpoint["state_dict"])
    model.eval()
    specs = json.loads((a.sensor / "calibration.json").read_text())
    gate = CameraGate({k: CameraSpec(v["width"], v["height"], v["calibration_id"]) for k, v in specs.items()})
    frames = json.loads((a.sensor / "frames.json").read_text())["frames"]
    a.output.mkdir(parents=True, exist_ok=True)
    predictions = []
    for item in frames:
        file = a.sensor / item["file"]
        if file.parent != a.sensor or file.suffix != ".png":
            raise ValueError("Invalid sensor filename")
        if hashlib.sha256(file.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Corrupt frame")
        rgb = np.asarray(Image.open(file).convert("RGB"))
        packet = {
            k: item[k] for k in ["camera_id", "sequence", "timestamp_s", "width", "height", "calibration_id"]
        }
        packet["rgb"] = rgb.tobytes()
        # Offline replay uses the recorded acquisition clock, not wall-clock freshness claims.
        frame = gate.accept(packet, now_s=item["timestamp_s"])
        x = (
            torch.from_numpy(
                np.frombuffer(frame.rgb, dtype=np.uint8).copy().reshape(frame.height, frame.width, 3)
            )
            .permute(2, 0, 1)[None]
            .float()
            / 255
        )
        start = time.monotonic()
        with torch.no_grad():
            probability = model(x).softmax(1)[0]
        mask = probability.argmax(0).numpy().astype(np.uint8)
        confidence = probability.numpy()
        latency = time.monotonic() - start
        Image.fromarray(mask).save(a.output / item["file"])
        objects = []
        for cls in range(1, 6):
            components, _ = ndimage.label(mask == cls)
            sizes = np.bincount(components.ravel())
            for label in np.flatnonzero(sizes >= 8):
                if label == 0:
                    continue
                yy, xx = np.where(components == label)
                if len(xx) < 8:
                    continue
                objects.append(
                    {
                        "class": CLASSES[cls],
                        "pixels": len(xx),
                        "centroid_xy": [float(xx.mean()), float(yy.mean())],
                        "mean_probability": float(confidence[cls, yy, xx].mean()),
                    }
                )
        predictions.append(
            {
                "file": item["file"],
                "camera_id": frame.camera_id,
                "sequence": frame.sequence,
                "objects": objects,
                "latency_s": latency,
                "motion_permitted": False,
            }
        )
        if len(predictions) % 16 == 0:
            print("INFER", len(predictions), "/", len(frames), flush=True)
    (a.output / "predictions.json").write_text(
        json.dumps(
            {
                "model_sha256": hashlib.sha256(a.model.read_bytes()).hexdigest(),
                "classes": CLASSES,
                "frames": predictions,
                "input_scope": "RGB only; camera metadata used for ingress validation",
                "timestamp_mode": "offline recorded-clock replay",
                "motion_permitted": False,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
