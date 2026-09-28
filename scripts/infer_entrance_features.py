"""Isolated RGB-only DINOv3 inference; no offline geometry, labels or poses mounted."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from dinov3_features import encode_dinov3, load_dinov3
from entrance_feature_model import CLASSES, CROP, FeatureHead
from PIL import Image

sensor, output = Path("/sensor"), Path("/results")
if any(output.iterdir()):
    raise RuntimeError("Fresh output directory required")
weights = Path("/model/member0.pt")
checkpoint = torch.load(weights, map_location="cpu", weights_only=True)
assert tuple(checkpoint["classes"]) == CLASSES and tuple(checkpoint["crop_xyxy"]) == CROP
head = FeatureHead().cuda().eval()
head.load_state_dict(checkpoint["state_dict"])
backbone = load_dinov3("/backbone", "cuda")
rows = []
with torch.inference_mode():
    for source in sorted(sensor.glob("*.png")):
        im = Image.open(source).convert("RGB")
        if im.size != (3840, 2160):
            raise ValueError("Unexpected camera dimensions")
        x = torch.from_numpy(np.array(im.crop(CROP))).permute(2, 0, 1)[None].cuda().float() / 255
        torch.cuda.synchronize()
        start = time.perf_counter()
        with torch.autocast("cuda", dtype=torch.bfloat16):
            features = encode_dinov3(backbone, x)
            probabilities = head(x, features).softmax(1)
        torch.cuda.synchronize()
        elapsed = time.perf_counter() - start
        mask = probabilities.argmax(1)[0].cpu().numpy().astype(np.uint8)
        Image.fromarray(mask).save(output / source.name)
        rows.append(
            dict(
                file=source.name,
                seconds=elapsed,
                rgb_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                predicted_pixels=[int((mask == c).sum()) for c in range(6)],
            )
        )
report = dict(
    frames=rows,
    classes=CLASSES,
    crop_xyxy=CROP,
    model_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
    backbone_sha256=hashlib.file_digest(open("/backbone/model.safetensors", "rb"), "sha256").hexdigest(),
    motion_permitted=False,
    insertion_pose=None,
    latch_state_verified=False,
    runtime_inputs="RGB only; fixed crop; offline files unavailable",
    runtime_code_sha256={
        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path("/code").glob("*.py")
    },
    peak_cuda_allocated_bytes=torch.cuda.max_memory_allocated(),
)
(output / "predictions.json").write_text(json.dumps(report, indent=2) + "\n")
