"""Publish paired static visibility probes without claiming entrance or grasp success."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("source", type=Path)
p.add_argument("destination", type=Path)
a = p.parse_args()
r = json.loads((a.source / "review.json").read_text())
assert len(r["frames"]) == 78 and r["physics_elapsed_s"] == 0 and not (a.source / "invalid.json").exists()
a.destination.mkdir(parents=True, exist_ok=True)
montage = a.source / "montage"
montage.mkdir(exist_ok=True)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
colors = np.array([[0, 0, 0], [245, 150, 30], [50, 220, 135]], dtype=np.uint8)
for row in r["frames"]:
    stem = Path(row["file"]).stem
    image = Image.open(a.source / row["file"]).convert("RGB").crop((1472, 856, 2368, 1304))
    image.save(a.destination / (stem + ".webp"), lossless=True)
    mask = np.array(Image.open(a.source / (stem + "-offline.png")).crop((1472, 856, 2368, 1304)))
    rgb = np.array(image)
    on = mask > 0
    rgb[on] = (rgb[on] * 0.5 + colors[mask[on]] * 0.5).astype(np.uint8)
    Image.fromarray(rgb).save(a.destination / (stem + "-offline.webp"), lossless=True)
for i, case in enumerate(r["cases"]):
    canvas = Image.new("RGB", (1792, 540), "#f2f3ed")
    d = ImageDraw.Draw(canvas)
    description = (
        "No fingers: matching baseline"
        if not case["jaws"]
        else (f"Setback {case['setback_mm']} mm / width {case['width_mm']} mm / "
              f"thickness {case['thickness_mm']} mm")
    )
    d.text(
        (16, 8),
        f"{case['gap_mm']} mm from nominal CAD review point | {description}",
        font=font,
        fill="#162925",
    )
    d.text((16, 39), "Axial camera", font=small, fill="#45554d")
    d.text((912, 39), "Offset camera", font=small, fill="#45554d")
    d.text(
        (16, 65),
        "Static visibility only. No grasp contact, deformation or verified aperture.",
        font=small,
        fill="#45554d",
    )
    for j, camera in enumerate(["entrance", "offset"]):
        canvas.paste(Image.open(a.destination / f"{i:03d}-{camera}.webp"), (j * 896, 92))
    canvas.save(montage / f"{i:04d}.png")
(a.destination / "review.json").write_text(json.dumps(r, indent=2) + "\n")
print(json.dumps({"frames": len(r["frames"]), "video_frames": len(r["cases"])}))
