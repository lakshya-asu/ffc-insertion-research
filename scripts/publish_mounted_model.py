"""Publish every frozen-test RGB / prediction / offline overlay, plus a review montage."""

import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont, ImageOps

ROOT = Path(__file__).resolve().parents[1]
data = ROOT / "outputs/macro-dinov3-test-001"
pred = ROOT / "outputs/macro-dinov3-predictions-001"
evaluation = json.loads((ROOT / "outputs/macro-dinov3-evaluation-001.json").read_text())
labels = json.loads((data / "offline/labels.json").read_text())["frames"]
out = ROOT / "docs/live/macro-model"
out.mkdir(exist_ok=True)
video = ROOT / "outputs/macro-model-video-001"
video.mkdir(exist_ok=True)
colors = np.array(
    [[0, 0, 0], [236, 146, 52], [56, 190, 170], [221, 75, 127], [91, 148, 240], [161, 106, 211]],
    dtype=np.uint8,
)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
frames = []
for i, row in enumerate(labels):
    name = row["file"]
    native = Image.open(data / "sensor" / name).convert("RGB")
    rgb = np.array(ImageOps.expand(native.resize((1224, 1024), Image.Resampling.BOX), border=(4, 0, 4, 0)))
    # At integer 2:1 scaling OpenCV INTER_NEAREST_EXACT picks [::2,::2], verified separately.
    gt = np.pad(np.array(Image.open(data / "offline" / name))[::2, ::2], ((0, 0), (4, 4)))
    pr = np.array(Image.open(pred / name))
    assert pr.shape == gt.shape == rgb.shape[:2]
    images = {"rgb": rgb}
    for kind, mask in [("prediction", pr), ("offline", gt)]:
        overlay = rgb.copy()
        overlay[mask > 0] = (rgb[mask > 0] * 0.48 + colors[mask[mask > 0]] * 0.52).astype(np.uint8)
        images[kind] = overlay
    prefix = f"{i:03d}"
    for kind, arr in images.items():
        im = Image.fromarray(arr)
        im.thumbnail((1000, 840))
        im.save(out / f"{prefix}-{kind}.webp", quality=90)
    panel = Image.new("RGB", (1600, 760), "#142820")
    d = ImageDraw.Draw(panel)
    for j, kind in enumerate(["rgb", "prediction"]):
        im = Image.fromarray(images[kind])
        im.thumbnail((790, 660))
        panel.paste(im, (j * 800 + (800 - im.width) // 2, 65))
    d.text((20, 16), f"Held-out {i + 1:02d}/60 / {row['condition']} / static scene", font=font, fill="white")
    d.text((20, 730), "RGB after fixed preprocessing", font=font, fill="white")
    d.text((820, 730), "DINOv3 prediction / no motion authority", font=font, fill="white")
    panel.save(video / f"{i:03d}.png")
    frames.append(
        dict(
            index=i,
            condition=row["condition"],
            **{k: f"{prefix}-{k}.webp" for k in images},
            metrics=evaluation["frames"][i],
        )
    )
manifest = dict(
    frames=frames,
    classes=evaluation["classes"],
    slider=evaluation["slider"],
    motion_permitted=False,
    model_sha256=evaluation["inference"]["model_sha256"],
    scope=evaluation["scope"],
)
(out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
(out / "evaluation.json").write_text(json.dumps(evaluation, indent=2) + "\n")
(out / "training-report.json").write_bytes(
    (ROOT / "outputs/macro-dinov3-model-002/training-report.json").read_bytes()
)
(out / "ATTRIBUTION.txt").write_bytes(
    (ROOT / "docs/live/macro-data/ATTRIBUTION.txt")
    .read_bytes()
    .replace(
        b"not robot actions or trained model predictions",
        b"not robot actions; prediction overlays are learned DINOv3 outputs",
    )
)
print("Published", len(frames), "RGB/prediction/offline triplets")
