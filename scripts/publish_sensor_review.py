"""Build a labelled review montage and lossless web assets from actual Isaac images."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    p.add_argument("web", type=Path)
    a = p.parse_args()
    report = json.loads((a.run / "review.json").read_text())
    assert report["timeline_elapsed_s"] == 0 and report["motion_permitted"] is False
    a.web.mkdir(parents=True, exist_ok=True)
    (a.run / "montage").mkdir(exist_ok=True)
    cfg = report["configuration"]
    assert len(report["frames"]) == 16
    for frame in report["frames"]:
        source = Image.open(a.run / frame["file"]).convert("RGB")
        assert source.size == tuple(cfg["image_size"])
        path = a.web / (Path(frame["file"]).stem + ".webp")
        source.save(path, lossless=True)
        assert np.array_equal(np.asarray(source), np.asarray(Image.open(path)))
    font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 24)
    small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 17)
    captions = [
        "Loose cable / pickup view",
        "Authored end-presentation pose",
        "Authored tip 15 mm above nominal socket",
        "Authored tip 3 mm above nominal socket",
    ]
    for i, case in enumerate(["loose", "present", "approach", "near"]):
        canvas = Image.new("RGB", (1600, 1480), "#f2f3ed")
        d = ImageDraw.Draw(canvas)
        d.text((24, 18), "SENSOR LAYOUT V2  /  " + captions[i], fill="#162925", font=font)
        d.text(
            (24, 55),
            "Static optical study. Jaw envelope only. No simulated grasp, physics or robot motion.",
            fill="#45554d",
            font=small,
        )
        for j, camera in enumerate(cfg["cameras"]):
            x, y = (j % 2) * 800, 96 + (j // 2) * 690
            img = Image.open(a.run / f"{case}-{camera['id']}.png").convert("RGB")
            img.thumbnail((784, 650), Image.Resampling.LANCZOS)
            canvas.paste(img, (x + 8, y))
            d.text((x + 15, y + 654), camera["name"], fill="#162925", font=small)
        canvas.save(a.run / "montage" / f"{i:04d}.png")
    # Image-plane sampling in task axes; this is not a detector-accuracy estimate.
    for camera in report["camera_metrics"]:
        eye, target = np.array(camera["eye_m"]), np.array(camera["target_m"])
        forward = target - eye
        distance = np.linalg.norm(forward)
        forward /= distance
        right = np.cross(forward, [0, 0, 1])
        right /= np.linalg.norm(right)
        up = np.cross(right, forward)
        scale = camera["normal_plane_pixels_per_mm"]
        camera["at_target_axis_sampling_px_per_mm"] = {
            axis: float(scale * np.hypot(np.dot(right, vector), np.dot(up, vector)))
            for axis, vector in zip(["world_x", "world_y", "world_z"], np.eye(3), strict=True)
        }
    report["asset_audit"] = {"lossless_images": 16, "all_match_source_pixels": True}
    (a.web / "review.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["camera_metrics"], indent=2))


if __name__ == "__main__":
    main()
