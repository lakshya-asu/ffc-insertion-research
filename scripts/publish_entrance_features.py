"""Publish every held-out RGB, predicted feature mask, and offline annotation."""

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
a.web.mkdir(parents=True, exist_ok=True)
montage = a.predictions / "montage"
montage.mkdir(exist_ok=True)
colors = np.array(
    [[0, 0, 0], [255, 150, 0], [0, 190, 255], [70, 230, 140], [230, 70, 160], [150, 80, 235]], dtype=np.uint8
)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16)
manifest = []
for i, row in enumerate(report["frames"]):
    stem = Path(row["file"]).stem
    rgb = np.array(Image.open(a.data / "sensor" / row["file"]).convert("RGB").crop((1472, 856, 2368, 1304)))
    layers = {}
    for layer in ["rgb", "prediction", "offline"]:
        arr = rgb.copy()
        if layer != "rgb":
            folder = a.predictions if layer == "prediction" else a.data / "offline"
            mask = np.array(Image.open(folder / row["file"]))
            on = mask > 0
            arr[on] = (arr[on] * 0.5 + colors[mask[on]] * 0.5).astype(np.uint8)
        filename = stem + "-" + layer + ".webp"
        Image.fromarray(arr).save(a.web / filename, lossless=True)
        assert np.array_equal(np.array(Image.open(a.web / filename)), arr)
        layers[layer] = filename
    manifest.append(
        dict(
            file=row["file"],
            condition=row["condition"],
            camera=row["camera"],
            predicted_slider_state=row["predicted_slider_state"],
            **layers,
        )
    )
    canvas = Image.new("RGB", (1792, 540), "#f2f3ed")
    d = ImageDraw.Draw(canvas)
    d.text(
        (16, 10),
        f"{row['file']} / {row['condition']} / provisional slider: {row['predicted_slider_state']}",
        font=font,
        fill="#162925",
    )
    d.text((16, 42), "Raw simulated camera window", font=small, fill="#45554d")
    d.text((912, 42), "DINOv3 feature prediction; no motion permission", font=small, fill="#45554d")
    d.text(
        (16, 67),
        "Engineered socket substitute. Slot clearances and slider travel are assumptions.",
        font=small,
        fill="#45554d",
    )
    canvas.paste(Image.fromarray(rgb), (0, 92))
    canvas.paste(Image.open(a.web / layers["prediction"]), (896, 92))
    canvas.save(montage / f"{i:04d}.png")
(a.web / "manifest.json").write_text(
    json.dumps(dict(frames=manifest, lossless_pixels_verified=True), indent=2) + "\n"
)
(a.web / "evaluation.json").write_text(a.evaluation.read_text())
print(json.dumps(dict(images=len(manifest) * 3, video_frames=len(manifest))))
