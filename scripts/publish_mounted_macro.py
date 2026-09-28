"""Curate RGB/offline-label review images; no learned predictions are implied."""

import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
out = ROOT / "docs/live/macro-data"
out.mkdir(exist_ok=True)
video = ROOT / "outputs/macro-data-video-001"
video.mkdir(exist_ok=True)
colors = np.array(
    [[0, 0, 0], [236, 146, 52], [56, 190, 170], [221, 75, 127], [91, 148, 240], [161, 106, 211]],
    dtype=np.uint8,
)
rows = []
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
for group, dirname in [("audit", "macro-dataset-audit-001"), ("matched", "macro-setback-pairs-001")]:
    run = ROOT / "outputs" / dirname
    labels = json.loads((run / "offline/labels.json").read_text())
    for row in labels["frames"]:
        rgb = np.array(Image.open(run / "sensor" / row["file"]))
        mask = np.array(Image.open(run / "offline" / row["file"]))
        mixed = rgb.copy()
        mixed[mask > 0] = (rgb[mask > 0] * 0.48 + colors[mask[mask > 0]] * 0.52).astype(np.uint8)
        name = f"{group}-{row['scene']:03d}"
        for kind, arr in [("rgb", rgb), ("offline", mixed)]:
            im = Image.fromarray(arr)
            im.thumbnail((1000, 840))
            im.save(out / f"{name}-{kind}.webp", quality=90)
        panel = Image.new("RGB", (1600, 760), "#142820")
        d = ImageDraw.Draw(panel)
        for j, arr in enumerate([rgb, mixed]):
            im = Image.fromarray(arr)
            im.thumbnail((790, 660))
            panel.paste(im, (j * 800 + (800 - im.width) // 2, 70))
        d.text(
            (25, 18),
            f"{group} {row['scene']:02d} / {row['condition']} / "
            f"setback {row['authored']['setback_m'] * 1000:.0f} mm",
            font=font,
            fill="white",
        )
        d.text((25, 730), "Camera RGB", font=font, fill="white")
        d.text((825, 730), "Offline labels only / no model prediction", font=font, fill="white")
        panel.save(video / f"{len(rows):03d}.png")
        rows.append(
            dict(
                group=group,
                scene=row["scene"],
                condition=row["condition"],
                setback_mm=row["authored"]["setback_m"] * 1000,
                visible_pixels=row["visible_pixels"],
                rgb=f"{name}-rgb.webp",
                offline=f"{name}-offline.webp",
            )
        )
manifest = dict(
    frames=rows,
    classes=labels["config"]["classes"],
    colors=colors.tolist(),
    motion_permitted=False,
    scope="Static visibility audit. Offline geometry labels, not learned predictions or an insertion result.",
)
(out / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
print("Published", len(rows), "review frames")
