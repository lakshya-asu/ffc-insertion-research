"""A missing visible target must not count as an empty-view success."""

import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image

ROOT = Path(__file__).parents[1]
spec = importlib.util.spec_from_file_location("zero_eval", ROOT / "scripts/evaluate_zero_regions.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_visible_miss_and_absent_false_positive_are_separate(tmp_path):
    data = tmp_path / "data"
    preds = tmp_path / "preds"
    (data / "offline").mkdir(parents=True)
    classes = ["background", "connector", "cable", "mini_end", "gripper"]
    truth = np.zeros((20, 20), np.uint8)
    truth[:4, :8] = 3
    Image.fromarray(truth).save(data / "offline/a.png")
    row = {"file": "a.png", "scene": 0, "split": "test", "condition": "ordinary", "camera_id": "entrance"}
    (data / "offline/labels.json").write_text(json.dumps({"classes": classes, "frames": [row]}))
    predicted = np.zeros_like(truth)
    predicted[8:12, :8] = 1
    frames = []
    # Two frames give the scorer a warmed latency sample as well.
    Image.fromarray(truth).save(data / "offline/b.png")
    rows = [row, dict(row, file="b.png", scene=1)]
    (data / "offline/labels.json").write_text(json.dumps({"classes": classes, "frames": rows}))
    for name in ["dino", "rgb_ablation"]:
        (preds / name).mkdir(parents=True)
        for file in ["a.png", "b.png"]:
            Image.fromarray(predicted).save(preds / name / file)
            frames.append(
                {"file": file, "model": name, "compute_s": 0.1, "objects": [{"class": "connector"}]}
            )
    (preds / "predictions.json").write_text(
        json.dumps(
            {
                "classes": classes,
                "frames": frames,
                "model_sha256": {},
                "backbone_sha256": "test",
                "crop_xyxy": [0, 0, 20, 20],
                "timing_scope": "test",
            }
        )
    )
    result = tmp_path / "report.json"
    module.evaluate(data, preds, result)
    scores = json.loads(result.read_text())["models"]["dino"]["groups"]["all"]["classes"]
    connector = next(x for x in scores if x["class"] == "connector")
    end = next(x for x in scores if x["class"] == "mini_end")
    assert connector["absent_images"] == 2 and connector["false_positive_images"] == 2
    assert end["visible_images"] == 2 and end["visible_iou_at_least_half"] == 0 and end["iou"] == 0
