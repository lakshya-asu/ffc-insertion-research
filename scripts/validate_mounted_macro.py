"""Offline dataset checks, including independent projection containment of the tip band."""

import argparse
import hashlib
import itertools
import json
from pathlib import Path

import numpy as np
from PIL import Image

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("run", type=Path)
a = p.parse_args()
if (a.run / "invalid.json").exists():
    raise SystemExit("Capture is marked invalid")
report = json.loads((a.run / "capture-report.json").read_text())
labels = json.loads((a.run / "offline/labels.json").read_text())
cal = json.loads((a.run / "sensor/camera.json").read_text())
assert report["physics_elapsed_s"] == 0 and not report["motion_permitted"]
assert len(labels["frames"]) == report["frames"]
k = np.array(cal["k"]).reshape(3, 3)
optical_world = np.linalg.inv(np.array(cal["world_optical"]))
seen = set()
counts = {}
for row in labels["frames"]:
    name = row["file"]
    rgb_path, mask_path = a.run / "sensor" / name, a.run / "offline" / name
    rgb, mask = np.array(Image.open(rgb_path)), np.array(Image.open(mask_path))
    assert rgb.shape == (2048, 2448, 3) and mask.shape == (2048, 2448), name
    assert set(np.unique(mask)) <= set(range(6)), name
    assert hashlib.sha256(rgb_path.read_bytes()).hexdigest() == row["rgb_sha256"]
    assert hashlib.sha256(mask_path.read_bytes()).hexdigest() == row["mask_sha256"]
    assert row["rgb_sha256"] not in seen, "Duplicate RGB"
    seen.add(row["rgb_sha256"])
    assert np.bincount(mask.ravel(), minlength=6).tolist() == row["visible_pixels"]
    assert not (mask == (5 if row["slider_open"] else 4)).any(), "Conflicting slider label"
    if row["condition"] in ["absent_cable", "empty_targets"]:
        assert not (mask == 3).any()
    if row["condition"] in ["absent_socket", "empty_targets"]:
        assert not np.isin(mask, [1, 2, 4, 5]).any()
    # Bound the 0.3 mm leading band with the entire header thickness (conservative).
    points = np.array(
        [list(x) + [1] for x in itertools.product([0, 0.0003], [-0.00575, 0.00575], [-0.00015, 0.0003])]
    )
    pose = np.array(row["authored"]["board_world"]) @ np.array(row["authored"]["cable_local"])
    camera = (optical_world @ pose @ points.T).T[:, :3]
    assert (camera[:, 2] > 0).all()
    uv = (k @ camera.T).T
    uv = uv[:, :2] / uv[:, 2:]
    low, high = uv.min(axis=0) - 3, uv.max(axis=0) + 3
    yy, xx = np.where(mask == 3)
    assert ((xx >= low[0]) & (xx <= high[0]) & (yy >= low[1]) & (yy <= high[1])).all(), (
        "Tip label outside projected envelope"
    )
    item = counts.setdefault(row["condition"], dict(images=0, visible=[0] * 5, pixels=[0] * 5))
    item["images"] += 1
    for c in range(1, 6):
        item["visible"][c - 1] += int((mask == c).any())
        item["pixels"][c - 1] += int((mask == c).sum())
result = dict(
    valid=True,
    frames=len(seen),
    conditions=counts,
    checks=[
        "native dimensions",
        "label range",
        "RGB/mask hashes",
        "no duplicate RGB",
        "class counts",
        "latch consistency",
        "strict absent labels",
        "independent leading-band containment",
        "zero physics",
    ],
    scope="Annotation integrity only; not learned accuracy, optical resolution or physical validation",
)
(a.run / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result, indent=2))
