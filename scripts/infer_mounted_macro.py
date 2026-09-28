"""Isolated RGB-only DINOv3 inference; no offline geometry, labels or poses mounted."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from dinov3_features import encode_dinov3, load_dinov3
from entrance_feature_model import CLASSES, FeatureHead
from ffc_cell.cv_frontend import REVISION, Frontend
from PIL import Image

sensor, output = Path("/sensor"), Path("/results")
if any(output.iterdir()):
    raise RuntimeError("Fresh output directory required")
weights = Path("/model/member0.pt")
checkpoint = torch.load(weights, map_location="cpu", weights_only=True)
assert tuple(checkpoint["classes"]) == CLASSES
assert checkpoint["preprocessing_revision"] == REVISION
assert checkpoint["input_size"] == [1232, 1024]
calibration = json.loads((sensor / "camera.json").read_text())
reference = json.loads(Path("/model/sensor-contract.json").read_text())
assert calibration == reference, "Camera calibration or profile changed"
assert calibration["profile"] == checkpoint["camera_profile"]
assert (
    hashlib.sha256(weights.read_bytes()).hexdigest()
    == json.loads(Path("/model/frozen.json").read_text())["model_sha256"]
)
frontend = Frontend()
torch.set_num_threads(4)
head = FeatureHead().cuda().eval()
head.load_state_dict(checkpoint["state_dict"])
backbone = load_dinov3("/backbone", "cuda")
rows = []
with torch.inference_mode():
    for source in sorted(sensor.glob("*.png")):
        im = Image.open(source).convert("RGB")
        if im.size != (2448, 2048):
            raise ValueError("Unexpected camera dimensions")
        window, _, _, quality = frontend.process(
            np.array(im), calibration["k"], calibration["d"], calibration["distortion_model"]
        )
        x = torch.from_numpy(window).permute(2, 0, 1)[None].cuda().float() / 255
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
    preprocessing_revision=REVISION,
    input_size=[1232, 1024],
    camera_profile=calibration["profile"],
    model_sha256=hashlib.sha256(weights.read_bytes()).hexdigest(),
    backbone_sha256=hashlib.file_digest(open("/backbone/model.safetensors", "rb"), "sha256").hexdigest(),
    motion_permitted=False,
    insertion_pose=None,
    latch_state_verified=False,
    runtime_inputs="Native RGB and fixed calibration only; full-field frontend; offline files unavailable",
    runtime_code_sha256={
        p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in Path("/code").glob("*.py")
    },
    peak_cuda_allocated_bytes=torch.cuda.max_memory_allocated(),
)
(output / "predictions.json").write_text(json.dumps(report, indent=2) + "\n")
