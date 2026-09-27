"""Fail closed on split leakage, corrupt inputs, missing classes and stale negative labels."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("data", type=Path)
p.add_argument("--development", type=Path)
a = p.parse_args()
if (a.data / "invalid.json").exists():
    raise ValueError("Rejected capture")
report = json.loads((a.data / "capture-report.json").read_text())
assert report["timeline_elapsed_s"] == 0
frames = json.loads((a.data / "sensor/frames.json").read_text())["frames"]
labels = json.loads((a.data / "offline/labels.json").read_text())["frames"]
assert len(frames) == len(labels) == report["images"]
assert len({r["file"] for r in frames}) == len(frames)
assert len({r["sha256"] for r in frames}) == len(frames)
by_name = {r["file"]: r for r in frames}
counts = {}
scenes = {}
for row in labels:
    frame = by_name[row["file"]]
    src = a.data / "sensor" / row["file"]
    assert hashlib.sha256(src.read_bytes()).hexdigest() == frame["sha256"]
    assert Image.open(src).size == (3840, 2160)
    mask = np.array(Image.open(a.data / "offline" / row["file"]))
    assert mask.shape == (448, 896) and mask.max() < 5
    pixels = np.bincount(mask.ravel(), minlength=5)
    assert pixels.tolist() == row["class_pixels"]
    if row["condition"] in ["absent_board", "empty"]:
        assert pixels[1] == 0
    if row["condition"] in ["absent_cable", "empty"]:
        assert sum(pixels[2:]) == 0
    counts.setdefault(row["split"], np.zeros(5, dtype=np.int64))
    counts[row["split"]] += pixels
    scenes.setdefault(row["scene"], set()).add(row["split"])
assert all(len(s) == 1 for s in scenes.values())
assert all(min(p[1:]) > 0 for p in counts.values())
if a.development:
    old = json.loads((a.development / "sensor/frames.json").read_text())["frames"]
    assert not {r["sha256"] for r in old} & {r["sha256"] for r in frames}
print(
    json.dumps(
        {
            "valid": True,
            "images": len(frames),
            "scenes": len(scenes),
            "split_pixels": {k: v.tolist() for k, v in counts.items()},
            "unique_rgb": True,
            "development_hash_disjoint": bool(a.development),
        },
        indent=2,
    )
)
