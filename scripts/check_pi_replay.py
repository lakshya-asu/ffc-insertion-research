"""Compare independent isolated runs; ignore timing, compare all decisions and mask bytes."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("first", type=Path)
    p.add_argument("second", type=Path)
    a = p.parse_args()
    reports = [json.loads((v / "predictions.json").read_text()) for v in [a.first, a.second]]
    if reports[0]["model_sha256"] != reports[1]["model_sha256"]:
        raise ValueError("Different checkpoints")
    frames = [{r["file"]: r for r in v["frames"]} for v in reports]
    if set(frames[0]) != set(frames[1]):
        raise ValueError("Different frame sets")
    for file, first in frames[0].items():
        second = frames[1][file]
        if {k: v for k, v in first.items() if k != "latency_s"} != {
            k: v for k, v in second.items() if k != "latency_s"
        }:
            raise ValueError(f"Prediction mismatch: {file}")
        if (
            hashlib.sha256((a.first / file).read_bytes()).digest()
            != hashlib.sha256((a.second / file).read_bytes()).digest()
        ):
            raise ValueError(f"Mask mismatch: {file}")
    result = {
        "images": len(frames[0]),
        "identical_masks_and_object_outputs": True,
        "timing_excluded": True,
        "model_sha256": reports[0]["model_sha256"],
        "scope": "two CPU inference processes with RGB and frozen weights; offline labels never mounted",
        "motion_permitted": False,
    }
    (a.first / "replay-check.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
