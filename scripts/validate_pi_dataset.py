"""Fail closed on missing/misaligned semantic classes before model training."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image


def validate(root):
    root = Path(root)
    labels = json.loads((root / "offline/labels.json").read_text())
    sensor = json.loads((root / "sensor/frames.json").read_text())
    frame_map = {v["file"]: v for v in sensor["frames"]}
    if set(frame_map) != {v["file"] for v in labels["frames"]}:
        raise ValueError("RGB/annotation manifests differ")
    totals = {}
    for row in labels["frames"]:
        name = row["file"]
        if Path(name).name != name:
            raise ValueError("Invalid image filename")
        frame = frame_map[name]
        rgb_path = root / "sensor" / name
        rgb = np.asarray(Image.open(rgb_path))
        mask = np.asarray(Image.open(root / "offline" / name))
        if rgb.shape != (frame["height"], frame["width"], 3) or mask.shape != rgb.shape[:2]:
            raise ValueError("Image/label dimensions disagree")
        if hashlib.sha256(rgb_path.read_bytes()).hexdigest() != frame["sha256"]:
            raise ValueError("RGB checksum mismatch")
        if mask.max() > 5:
            raise ValueError("Unknown semantic class")
        counts = np.bincount(mask.ravel(), minlength=6)
        if counts.tolist() != row["class_pixels"]:
            raise ValueError("Mask/count mismatch")
        totals.setdefault(row["split"], np.zeros(6, dtype=np.int64))
        totals[row["split"]] += counts
        if row["camera_id"] == "desk" and row["condition"] == "ordinary" and min(counts[1:4]) == 0:
            raise ValueError(f"Visible ordinary cable has missing annotations: {name}: {counts.tolist()}")
        if row["condition"] == "absent_cable" and counts[1:4].sum():
            raise ValueError("Invisible cable still has annotations")
        if row["condition"] == "absent_board" and counts[4:6].sum():
            raise ValueError("Invisible board still has annotations")
    if any(min(total[1:]) == 0 for total in totals.values()):
        raise ValueError("A dataset split has an entirely empty task class")
    return {
        "images": len(frame_map),
        "class_pixels_per_split": {k: v.tolist() for k, v in totals.items()},
        "gate": "passed",
        "scope": "label presence, shape, checksum and absence consistency; not real-world accuracy",
    }


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("data", type=Path)
    a = p.parse_args()
    report = validate(a.data)
    (a.data / "label-audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
