"""Publish raw native macro views and a labelled comparison video sequence."""

import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "docs/live/macro"
DEST.mkdir(exist_ok=True)
frames = []
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
video = ROOT / "outputs/macro-review-video-001"
video.mkdir(exist_ok=True)
for run, label in [("001", "Diagonal view, yaw 35 degrees"), ("002", "Selected view, yaw 0 degrees")]:
    source = ROOT / ("outputs/macro-setup-" + run)
    report = json.loads((source / "report.json").read_text())
    (DEST / ("report-" + run + ".json")).write_text(json.dumps(report, indent=2) + "\n")
    for gap in ["6", "3", "1", "0.3"]:
        row = {"run": run, "placement": label, "gap_mm": float(gap)}
        canvas = Image.new("RGB", (1600, 760), "#111820")
        draw = ImageDraw.Draw(canvas)
        draw.text((24, 16), f"{label} | cable gap {gap} mm | static simulation", font=font, fill="white")
        for j, mode in enumerate(["ideal", "finite-aperture"]):
            im = Image.open(source / f"gap-{gap}-{mode}.png").convert("RGB")
            name = f"{run}-{gap}-{mode}.webp"
            im.save(DEST / name, lossless=True)
            row[mode] = name
            preview = im.resize((780, 652))
            canvas.paste(preview, (10 + j * 800, 90))
            draw.text(
                (24 + j * 800, 56),
                "Ideal framing" if j == 0 else "Finite aperture: uncalibrated proxy",
                font=font,
                fill="#b9c5ce",
            )
        name = f"{run}-{gap}-placement.webp"
        Image.open(source / f"placement-{gap}.png").save(DEST / name, lossless=True)
        row["overview"] = name
        canvas.save(video / f"{len(frames):03d}.png")
        frames.append(row)
(DEST / "manifest.json").write_text(json.dumps({"frames": frames}, indent=2) + "\n")
(DEST / "ATTRIBUTION.md").write_text(
    "Rendered from the project Zero 2 W scene. "
    "Original PCB asset provenance: ../zero-study/ATTRIBUTION.txt. "
    "Connector and camera/lens envelopes are engineered approximations; see report JSON and experiment 018. "
    "No vendor camera CAD is redistributed.\n"
)
