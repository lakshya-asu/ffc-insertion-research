"""Publish actual RGB, prediction overlays and offline annotation comparisons."""

import argparse
import json
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

COLOURS = np.array(
    [[0, 0, 0], [67, 219, 183], [255, 212, 93], [115, 156, 255], [244, 134, 180], [207, 176, 255]],
    dtype=np.uint8,
)


def overlay(rgb, mask):
    output = rgb.copy()
    selected = mask > 0
    output[selected] = (rgb[selected] * 0.45 + COLOURS[mask[selected]] * 0.55).astype(np.uint8)
    return output


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--predictions", type=Path, required=True)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--output", type=Path, default=Path("docs/pi-perception"))
    p.add_argument("--video-frames", type=Path, required=True)
    a = p.parse_args()
    assets, data = a.output / "assets", a.output / "data"
    assets.mkdir(parents=True, exist_ok=True)
    data.mkdir(parents=True, exist_ok=True)
    a.video_frames.mkdir(parents=True, exist_ok=True)
    frames = json.loads((a.data / "offline/labels.json").read_text())["frames"]
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 20)
    selected_cases = {0, 3, 7, 12, 20, 30, 39} | set(range(40, 56))
    index = 0
    for row in frames:
        file = row["file"]
        stem = Path(file).stem
        rgb = np.asarray(Image.open(a.data / "sensor" / file))
        truth = np.asarray(Image.open(a.data / "offline" / file))
        prediction = np.asarray(Image.open(a.predictions / file))
        annotated = overlay(rgb, prediction)
        for mode, array in [("rgb", rgb), ("prediction", annotated), ("truth", overlay(rgb, truth))]:
            Image.fromarray(array).save(assets / f"{stem}-{mode}.webp", lossless=True)
        if int(stem.split("-")[0]) in selected_cases:
            canvas = Image.new("RGB", (1536, 568), (23, 33, 31))
            canvas.paste(Image.fromarray(rgb), (0, 0))
            canvas.paste(Image.fromarray(annotated), (768, 0))
            draw = ImageDraw.Draw(canvas)
            text = f"Scene {stem} / {row['condition'].replace('_', ' ')} / static synthetic review"
            draw.text((20, 527), text, fill=(235, 241, 234), font=font)
            draw.text((785, 527), "RGB-only prediction / no robot action", fill=(145, 200, 173), font=font)
            canvas.save(a.video_frames / f"{index:04d}.png")
            if index == 0:
                canvas.save(assets / "review-poster.webp", quality=90)
            index += 1
    for source, name in [
        (a.predictions / "evaluation.json", "evaluation.json"),
        (a.predictions / "replay-check.json", "replay-check.json"),
        (a.model / "training-report.json", "training-report.json"),
        (a.model / "history.json", "history.json"),
        (a.model / "model.pt", "model.pt"),
        (a.data / "capture-report.json", "capture-report.json"),
        (a.data / "label-audit.json", "label-audit.json"),
        (a.data / "sensor/calibration.json", "calibration.json"),
        (a.data / "sensor/frames.json", "frames.json"),
    ]:
        shutil.copy(source, data / name)
    (data / "review.json").write_text(
        json.dumps(
            {
                "images": len(frames),
                "video_frames": index,
                "raw_webp": "lossless encoding of the exact RGB input pixels",
                "overlays": "offline visualization only",
                "motion_permitted": False,
            },
            indent=2,
        )
    )
    print("Published images:", len(frames), "Video frames:", index)


if __name__ == "__main__":
    main()
