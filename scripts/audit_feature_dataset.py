"""Offline feature-label integrity audit; independent from the learned estimator."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("data", type=Path)
p.add_argument("--other", type=Path)
a = p.parse_args()
assert not (a.data / "invalid.json").exists()
report = json.loads((a.data / "capture-report.json").read_text())
labels = json.loads((a.data / "offline/labels.json").read_text())
counts = np.zeros(6, dtype=np.int64)
hashes = set()
seen = {}
for row in labels["frames"]:
    rgb = a.data / "sensor" / row["file"]
    mask = np.array(Image.open(a.data / "offline" / row["file"]))
    assert Image.open(rgb).size == (3840, 2160) and mask.shape == (448, 896)
    assert mask.dtype == np.uint8 and mask.max() <= 5
    actual = np.bincount(mask.ravel(), minlength=6)
    assert actual.tolist() == row["visible_pixels"]
    assert not ((mask == 4).any() and (mask == 5).any())
    if row["slider_open"]:
        assert not (mask == 5).any()
    else:
        assert not (mask == 4).any()
    if row["condition"] == "absent_cable":
        assert not (mask == 3).any()
    if row["condition"] == "absent_socket":
        assert not np.isin(mask, [1, 2, 4, 5]).any()
    sha = hashlib.sha256(rgb.read_bytes()).hexdigest()
    assert sha == row["rgb_sha256"]
    hashes.add(sha)
    seen.setdefault(row["split"], set()).add(row["scene"])
    counts += actual
assert report["frames"] == len(labels["frames"]) and report["physics_elapsed_s"] == 0
assert len(hashes) == len(labels["frames"])
assert np.all(counts[1:] > 0), "Every feature class must appear somewhere in this dataset"
if "train" in seen:
    assert not seen["train"] & seen["validation"]
if a.other:
    other = json.loads((a.other / "offline/labels.json").read_text())
    assert not hashes & {r["rgb_sha256"] for r in other["frames"]}
result = dict(
    frames=len(hashes),
    class_pixels=counts.tolist(),
    status="pass",
    scope="Label IDs, state consistency, absences and split integrity; not physical accuracy",
)
(a.data / "validation.json").write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps(result))
