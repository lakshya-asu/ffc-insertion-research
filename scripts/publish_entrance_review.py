"""Combine separate RGB and offline-label passes; never publish label-pass RGB as sensor input."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rgb-run", type=Path, required=True)
    parser.add_argument("--label-run", type=Path, required=True)
    parser.add_argument("--web", type=Path, required=True)
    a = parser.parse_args()
    rgb_report = json.loads((a.rgb_run / "review.json").read_text())
    label_report = json.loads((a.label_run / "review.json").read_text())
    assert rgb_report["configuration"] == label_report["configuration"]
    assert not rgb_report.get("rgb_contains_annotation_plane", False)
    assert label_report["rgb_contains_annotation_plane"]
    assert rgb_report["timeline_elapsed_s"] == label_report["timeline_elapsed_s"] == 0
    cfg = rgb_report["configuration"]
    counts = label_report["offline_recess_pixels"]
    visibility = []
    for camera in cfg["cameras"]:
        name = camera["id"]
        assert counts[f"loose-{name}"] > 1000
        assert abs(counts[f"present-{name}"] / counts[f"loose-{name}"] - 1) < 0.02
        visibility.append(
            {
                "camera": name,
                "unobstructed_pixels": counts[f"loose-{name}"],
                "approach_visible_fraction": counts[f"approach-{name}"] / counts[f"loose-{name}"],
                "near_visible_fraction": counts[f"near-{name}"] / counts[f"loose-{name}"],
            }
        )
    a.web.mkdir(parents=True, exist_ok=True)
    (a.label_run / "montage").mkdir(exist_ok=True)
    for frame in rgb_report["frames"]:
        stem = Path(frame["file"]).stem
        raw = np.array(Image.open(a.rgb_run / frame["file"]).convert("RGB"))
        mask = np.array(Image.open(a.label_run / (stem + "-offline-mask.png"))) > 0
        assert raw.shape[:2] == mask.shape == tuple(reversed(cfg["image_size"]))
        overlay = raw.copy()
        overlay[mask] = (0.40 * raw[mask] + 0.60 * np.array([30, 240, 130])).astype(np.uint8)
        for kind, arr in [("rgb", raw), ("offline", overlay)]:
            dest = a.web / (stem + "-" + kind + ".webp")
            Image.fromarray(arr).save(dest, lossless=True)
            assert np.array_equal(np.asarray(Image.open(dest)), arr)
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 26)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 18)
    for i, case in enumerate(["loose", "approach", "near"]):
        canvas = Image.new("RGB", (2100, 1390), "#f2f3ed")
        draw = ImageDraw.Draw(canvas)
        draw.text((22, 16), f"CSI ENTRANCE AUDIT / {case.upper()}", fill="#162925", font=font)
        draw.text(
            (22, 53),
            "Top: raw RGB. Bottom: offline CAD recess overlay. No learned prediction or robot motion.",
            fill="#45554d",
            font=small,
        )
        for row, mode in enumerate(["rgb", "offline"]):
            for col, camera in enumerate(cfg["cameras"]):
                img = Image.open(a.web / f"{case}-{camera['id']}-{mode}.webp").convert("RGB")
                img.thumbnail((680, 580), Image.Resampling.LANCZOS)
                x, y = col * 700 + 10, 100 + row * 640
                canvas.paste(img, (x, y))
                draw.text((x, y + 588), camera["name"] + " / " + mode, fill="#162925", font=small)
        canvas.save(a.label_run / "montage" / f"{i:04d}.png")
    report = {
        "configuration": cfg,
        "camera_metrics": rgb_report["camera_metrics"],
        "visibility": visibility,
        "labels": "offline CAD recess plane; not learned predictions",
        "physical_validation": False,
        "motion_permitted": False,
        "rgb_source_script_sha256": rgb_report["script_sha256"],
        "label_source_script_sha256": label_report["script_sha256"],
        "asset_audit": {
            "lossless_images": 24,
            "all_match_source_pixels": True,
            "unobstructed_label_consistency_passed": True,
        },
    }
    (a.web / "review.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(visibility, indent=2))


if __name__ == "__main__":
    main()
