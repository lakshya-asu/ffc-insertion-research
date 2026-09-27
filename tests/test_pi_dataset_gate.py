"""Regression for the discarded run: visible cable RGB with empty cable annotations."""

import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from PIL import Image

spec = importlib.util.spec_from_file_location(
    "pi_dataset_gate", Path(__file__).resolve().parents[1] / "scripts/validate_pi_dataset.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_missing_cable_annotations_cannot_pass_the_dataset_gate(tmp_path):
    for folder in ["sensor", "offline"]:
        (tmp_path / folder).mkdir()
    Image.fromarray(np.full((8, 12, 3), 190, dtype=np.uint8)).save(tmp_path / "sensor/0000-desk.png")
    rgb_hash = hashlib.sha256((tmp_path / "sensor/0000-desk.png").read_bytes()).hexdigest()
    frame = {"file": "0000-desk.png", "height": 8, "width": 12, "sha256": rgb_hash}
    (tmp_path / "sensor/frames.json").write_text(json.dumps({"frames": [frame]}))
    mask = np.zeros((8, 12), dtype=np.uint8)
    mask[:2, :2] = 4
    mask[-2:, -2:] = 5
    Image.fromarray(mask).save(tmp_path / "offline/0000-desk.png")
    annotation = {
        "file": frame["file"],
        "split": "train",
        "camera_id": "desk",
        "condition": "ordinary",
        "class_pixels": np.bincount(mask.ravel(), minlength=6).tolist(),
    }
    (tmp_path / "offline/labels.json").write_text(json.dumps({"frames": [annotation]}))
    with pytest.raises(ValueError, match="missing annotations"):
        module.validate(tmp_path)
    mask[2:4, :4] = 1
    mask[4:6, :4] = 2
    mask[6:8, :4] = 3
    Image.fromarray(mask).save(tmp_path / "offline/0000-desk.png")
    annotation["class_pixels"] = np.bincount(mask.ravel(), minlength=6).tolist()
    (tmp_path / "offline/labels.json").write_text(json.dumps({"frames": [annotation]}))
    assert module.validate(tmp_path)["gate"] == "passed"
