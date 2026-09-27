"""Isolated RGB replay. Mount only sensor frames, frozen models and executable code."""

import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from camera_observations import CameraGate, CameraSpec
from PIL import Image
from scipy import ndimage
from zero_region_model import CLASSES, CROP, RegionHead, encode, load_backbone


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--sensor", type=Path, default=Path("/sensor"))
    p.add_argument("--models", type=Path, default=Path("/models"))
    p.add_argument("--backbone", type=Path, default=Path("/backbone"))
    p.add_argument("--output", type=Path, default=Path("/results"))
    a = p.parse_args()
    torch.set_num_threads(4)
    if (a.output / "predictions.json").exists():
        raise ValueError("Use fresh inference output")
    device = "cuda" if torch.cuda.is_available() else "cpu"
    specs = json.loads((a.sensor / "calibration.json").read_text())
    gate = CameraGate({k: CameraSpec(v["width"], v["height"], v["calibration_id"]) for k, v in specs.items()})
    rows = json.loads((a.sensor / "frames.json").read_text())["frames"]
    backbone = load_backbone(a.backbone / "source", a.backbone / "dinov2_vitb14_pretrain.pth", device)
    heads = {}
    hashes = {}
    for name in ["dino", "rgb_ablation"]:
        ckpt = torch.load(a.models / (name + ".pt"), map_location="cpu", weights_only=True)
        if tuple(ckpt["classes"]) != CLASSES or tuple(ckpt["crop_xyxy"]) != CROP:
            raise ValueError("Model contract mismatch")
        model = RegionHead(ckpt["use_dino"])
        model.load_state_dict(ckpt["state_dict"])
        heads[name] = model.to(device).eval()
        hashes[name] = hashlib.sha256((a.models / (name + ".pt")).read_bytes()).hexdigest()
        (a.output / name).mkdir(parents=True, exist_ok=True)
    predictions = []
    for item in rows:
        file = a.sensor / item["file"]
        if file.parent != a.sensor or file.suffix != ".png":
            raise ValueError("Invalid frame path")
        if hashlib.sha256(file.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Frame hash mismatch")
        raw = np.array(Image.open(file).convert("RGB"))
        packet = {
            k: item[k] for k in ["camera_id", "sequence", "timestamp_s", "width", "height", "calibration_id"]
        }
        packet["rgb"] = raw.tobytes()
        gate.accept(packet, now_s=item["timestamp_s"])
        if tuple(specs[item["camera_id"]]["crop_xyxy"]) != CROP:
            raise ValueError("Fixed camera crop mismatch")
        x0, y0, x1, y1 = CROP
        crop = raw[y0:y1, x0:x1].copy()
        x = torch.from_numpy(crop).permute(2, 0, 1)[None].to(device).float() / 255
        for name, head in heads.items():
            if device == "cuda":
                torch.cuda.synchronize()
            start = time.perf_counter()
            with torch.inference_mode():
                if name == "dino":
                    with torch.autocast(device_type=device, dtype=torch.bfloat16, enabled=device == "cuda"):
                        z = encode(backbone, x)
                    z = z.float()
                else:
                    z = None
                prob = head(x, z).softmax(1)[0]
                mask = prob.argmax(0).cpu().numpy().astype(np.uint8)
            if device == "cuda":
                torch.cuda.synchronize()
            elapsed = time.perf_counter() - start
            Image.fromarray(mask).save(a.output / name / item["file"])
            objects = []
            for cls in range(1, len(CLASSES)):
                cc, _ = ndimage.label(mask == cls)
                counts = np.bincount(cc.ravel())
                counts[0] = 0
                if counts.max(initial=0) < 16:
                    continue
                largest = int(counts.argmax())
                yy, xx = np.where(cc == largest)
                objects.append(
                    {
                        "class": CLASSES[cls],
                        "visible_pixels": len(xx),
                        "bbox_xyxy_full_frame": [
                            int(xx.min() + x0),
                            int(yy.min() + y0),
                            int(xx.max() + x0 + 1),
                            int(yy.max() + y0 + 1),
                        ],
                        "status": "unverified visible-region candidate, not a mating or grasp target",
                    }
                )
            predictions.append(
                {
                    "file": item["file"],
                    "camera_id": item["camera_id"],
                    "model": name,
                    "objects": objects,
                    "compute_s": elapsed,
                    "motion_permitted": False,
                    "aperture": "unknown",
                    "latch": "unknown",
                    "leading_edge": "not estimated",
                    "insertion_pose": "not estimated",
                }
            )
        if len(predictions) % 16 == 0:
            print("REPLAY", len(predictions) // 2, len(rows), flush=True)
    report = {
        "model_sha256": hashes,
        "frames": predictions,
        "classes": CLASSES,
        "device": device,
        "input_scope": (
            "RGB with fixed optical-center window; calibration/timestamps used only for ingress validation"
        ),
        "crop_xyxy": CROP,
        "motion_permitted": False,
        "timing_scope": "model compute on predecoded crop; excludes capture and image decode",
        "timestamp_mode": "recorded-clock offline replay",
        "component_min_pixels": 16,
        "backbone_sha256": hashlib.sha256(
            (a.backbone / "dinov2_vitb14_pretrain.pth").read_bytes()
        ).hexdigest(),
    }
    (a.output / "predictions.json").write_text(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
