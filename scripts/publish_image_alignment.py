"""Publish image-space geometry diagnostics; reference lines are offline scorer data."""

import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

root = Path(__file__).resolve().parents[1]
run = root / "outputs/image-alignment-001"
output = root / "docs/demo/alignment"
output.mkdir(exist_ok=False)
estimates = json.loads((run / "estimates.json").read_text())
report = json.loads((run / "evaluation-with-projections.json").read_text())
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
colors = {"upper_rim": (219, 130, 30), "lower_rim": (20, 162, 145), "cable_edge": (217, 62, 123)}
manifest = []
for index, (estimate, score) in enumerate(zip(estimates["frames"], report["frames"], strict=True)):
    native = Image.open(root / "outputs/macro-dinov3-test-001/sensor" / estimate["file"]).convert("RGB")
    rgb = Image.new("RGB", (1232, 1024))
    rgb.paste(native.resize((1224, 1024), Image.Resampling.BOX), (4, 0))
    canvas = Image.new("RGB", (1232, 1124), "#f5f4ef")
    canvas.paste(rgb, (0, 70))
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (20, 12),
        f"{index + 1:02d}/60  {score['condition']}  |  {estimate['status']}",
        font=font,
        fill="#203d36",
    )
    draw.text(
        (20, 40),
        "Solid: predicted edge fit. Dashed: authored projection (offline scorer only).",
        font=font,
        fill="#203d36",
    )
    for name, line in estimate["features"].items():
        low, high = line["x_support"]
        draw.line(
            [
                (low, line["slope"] * low + line["intercept"] + 70),
                (high, line["slope"] * high + line["intercept"] + 70),
            ],
            fill=colors[name],
            width=4,
        )
        endpoints = np.array(score["projected_reference_edges"][name])
        endpoints[:, 1] += 70
        for t in np.arange(0, 1, 0.025):
            points = [endpoints[0] + v * (endpoints[1] - endpoints[0]) for v in [t, min(t + 0.012, 1)]]
            draw.line([tuple(x) for x in points], fill=(245, 245, 245), width=2)
    reason = (
        "; ".join(estimate["reasons"])
        or "Image-space measurement only. No calibrated uncertainty or motion permission."
    )
    draw.text((20, 1096), reason[:108], font=font, fill="#203d36")
    filename = f"{index:03d}.webp"
    canvas.save(output / filename, quality=90)
    manifest.append(dict(image=filename, **score, measurement=estimate["measurement"]))
(output / "manifest.json").write_text(
    json.dumps(dict(frames=manifest, summary=report["summary"]), indent=2) + "\n"
)
shutil.copyfile(run / "evaluation-with-projections.json", output / "evaluation.json")
shutil.copyfile(root / "docs/live/macro-data/ATTRIBUTION.txt", output / "ATTRIBUTION.txt")
# Contact sheets for inspecting all sixty rendered diagnostic frames.
for offset in range(0, 60, 12):
    sheet = Image.new("RGB", (4 * 308, 3 * 281), "white")
    for j in range(12):
        im = Image.open(output / f"{offset + j:03d}.webp").resize((308, 281))
        sheet.paste(im, ((j % 4) * 308, (j // 4) * 281))
    sheet.save(run / f"review-{offset // 12}.jpg")
