"""Publish a complete held-out RGB/prediction/offline-label review with lossless image crops."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("data", type=Path)
p.add_argument("predictions", type=Path)
p.add_argument("evaluation", type=Path)
p.add_argument("web", type=Path)
a = p.parse_args()
report = json.loads(a.evaluation.read_text())
rows = json.loads((a.data / "offline/labels.json").read_text())["frames"]
a.web.mkdir(parents=True, exist_ok=True)
montage = a.predictions / "montage"
montage.mkdir(exist_ok=True)
colors = np.array([[0, 0, 0], [242, 152, 45], [70, 150, 240], [50, 220, 120], [230, 75, 145]], dtype=np.uint8)
manifest = []
video = []
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 23)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
for i, row in enumerate(rows):
    stem = Path(row["file"]).stem
    rgb = np.array(Image.open(a.data / "sensor" / row["file"]).convert("RGB").crop(report["crop_xyxy"]))
    record = {k: row[k] for k in ["file", "scene", "split", "condition", "camera_id"]}
    layers = {}
    for layer in ["rgb", "dino", "rgb_ablation", "offline"]:
        im = rgb.copy()
        if layer != "rgb":
            path = (
                a.data / "offline" / row["file"]
                if layer == "offline"
                else a.predictions / layer / row["file"]
            )
            mask = np.array(Image.open(path))
            on = mask > 0
            im[on] = (im[on] * 0.45 + colors[mask[on]] * 0.55).astype(np.uint8)
        dest = a.web / f"{stem}-{layer}.webp"
        Image.fromarray(im).save(dest, lossless=True)
        assert np.array_equal(np.array(Image.open(dest)), im)
        record[layer] = dest.name
        layers[layer] = im
    manifest.append(record)
    if row["split"] == "challenge" or i < 4:
        canvas = Image.new("RGB", (1792, 530), "#f2f3ed")
        d = ImageDraw.Draw(canvas)
        d.text(
            (16, 12),
            f"ZERO / {row['condition']} / {row['camera_id']} / scene {row['scene']:02d}",
            font=font,
            fill="#162925",
        )
        d.text((16, 48), "Native RGB window", font=small, fill="#45554d")
        d.text(
            (912, 48), "DINOv2 learned visible regions — not an insertion target", font=small, fill="#45554d"
        )
        canvas.paste(Image.fromarray(rgb), (0, 82))
        canvas.paste(Image.fromarray(layers["dino"]), (896, 82))
        canvas.save(montage / f"{len(video):04d}.png")
        video.append(row["file"])
(a.web / "manifest.json").write_text(
    json.dumps(
        {
            "frames": manifest,
            "video_frames": video,
            "crop_xyxy": report["crop_xyxy"],
            "all_webp_pixels_verified": True,
            "classes": report["classes"],
        },
        indent=2,
    )
)
(a.web / "evaluation.json").write_text(a.evaluation.read_text())
print(json.dumps({"images": len(manifest) * 4, "video_seconds": len(video)}))
