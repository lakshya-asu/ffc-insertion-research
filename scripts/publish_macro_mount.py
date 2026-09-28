"""Publish mounting design evidence without claiming a physical manipulation trial."""

import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
source = ROOT / "outputs/mount-review-003"
dest = ROOT / "docs/live/mount"
dest.mkdir(exist_ok=True)
review = json.loads((source / "review.json").read_text())
report = json.loads((ROOT / "outputs/mount-clearance-004/report.json").read_text())
sweeps = json.loads((ROOT / "outputs/mount-clearance-004/translation-sweeps.json").read_text())
frames = []
video = ROOT / "outputs/mount-video-002"
video.mkdir(exist_ok=True)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
for row in review["frames"]:
    i = row["frame"]
    entry = dict(row)
    canvas = Image.new("RGB", (1600, 720), "#101820")
    draw = ImageDraw.Draw(canvas)
    draw.text(
        (24, 16),
        f"Mount review / {row['segment'].replace('_', ' ')} / static offline pose",
        font=font,
        fill="white",
    )
    for j, name in enumerate(["placement", "macro", "cell"]):
        im = Image.open(source / f"{i:02d}-{name}.png").convert("RGB")
        filename = f"{i:02d}-{name}.webp"
        im.save(dest / filename, lossless=True)
        entry[name] = filename
        if j < 2:
            im.thumbnail((780, 650))
            canvas.paste(im, (10 + j * 800 + (780 - im.width) // 2, 60))
    canvas.save(video / f"{i:03d}.png")
    frames.append(entry)
(dest / "manifest.json").write_text(json.dumps(dict(frames=frames), indent=2) + "\n")
(dest / "clearance.json").write_text(json.dumps(report, indent=2) + "\n")
(dest / "translation-sweeps.json").write_text(json.dumps(sweeps, indent=2) + "\n")
(dest / "ATTRIBUTION.txt").write_text(
    "FR3 geometry: official Franka Robotics franka_simulation v0.1.0, FR3 v2.1. "
    "PCB provenance: ../zero-study/ATTRIBUTION.txt. "
    "Tool and stand are design envelopes, not manufacturing CAD. "
    "Static poses; no physics steps or runtime privileged-state control.\n"
)
