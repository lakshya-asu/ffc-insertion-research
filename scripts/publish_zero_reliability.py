"""Publish all frozen reliability test views and a decision-labelled review montage."""

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
colors = np.array([[0, 0, 0], [242, 152, 45], [70, 150, 240], [50, 220, 120], [230, 75, 145]], dtype=np.uint8)
font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 22)
small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
manifest, video, seen = [], [], set()
for row in report["frames"]:
    rgb = np.array(Image.open(a.data / "sensor" / row["file"]).convert("RGB").crop(report["crop_xyxy"]))
    record = {
        k: row[k] for k in ["file", "scene", "condition", "camera_id", "decision", "good_component_pair"]
    }
    layers = {}
    for layer in ["rgb", "ensemble", "previous", "offline"]:
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
        dest = a.web / f"{Path(row['file']).stem}-{layer}.webp"
        Image.fromarray(im).save(dest, lossless=True)
        assert np.array_equal(np.array(Image.open(dest)), im)
        record[layer] = dest.name
        layers[layer] = im
    manifest.append(record)
    key = (row["condition"], row["camera_id"])
    bad_accept = row["decision"]["accepted"] and not row["good_component_pair"]
    if key not in seen or bad_accept:
        seen.add(key)
        canvas = Image.new("RGB", (1792, 540), "#f2f3ed")
        d = ImageDraw.Draw(canvas)
        status = "REVIEW CANDIDATE" if row["decision"]["accepted"] else "NEED ANOTHER VIEW"
        audit = " / OFFLINE: BAD ACCEPT" if bad_accept else ""
        d.text((16, 10), f"{row['file']} / {row['condition']} / {status}{audit}", font=font, fill="#162925")
        d.text((16, 46), "Raw native camera window", font=small, fill="#45554d")
        d.text(
            (912, 46), "Two-head masks; no insertion pose or motion permission", font=small, fill="#45554d"
        )
        canvas.paste(Image.fromarray(rgb), (0, 92))
        canvas.paste(Image.fromarray(layers["ensemble"]), (896, 92))
        canvas.save(montage / f"{len(video):04d}.png")
        video.append(row["file"])
(a.web / "manifest.json").write_text(
    json.dumps({"frames": manifest, "video_frames": video, "all_webp_pixels_verified": True}, indent=2)
)
(a.web / "evaluation.json").write_text(a.evaluation.read_text())
print(json.dumps({"images": 4 * len(manifest), "video_seconds": len(video)}))
