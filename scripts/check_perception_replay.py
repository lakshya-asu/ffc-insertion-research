"""Compare isolated RGB replays after withholding the offline label directory."""

import argparse
import hashlib
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("run", type=Path)
    a = p.parse_args()
    original = a.run / "predictions"
    replay = a.run / "replay-without-labels"

    def deterministic(path):
        data = json.loads((path / "predictions.json").read_text())
        for item in data["predictions"]:
            item.pop("inference_ms")
        return data

    before, after = deterministic(original), deterministic(replay)
    if before != after:
        raise RuntimeError("Predictions changed despite identical sensor inputs")
    for item in before["predictions"]:
        name = item["mask"]
        if (original / name).read_bytes() != (replay / name).read_bytes():
            raise RuntimeError("Mask changed")
    result = {
        "status": "pass",
        "images": len(before["predictions"]),
        "same_predictions_and_masks": True,
        "experiment": (
            "offline/ was renamed out of the expected path during replay; "
            "it was never mounted into either worker"
        ),
        "scope": "functional isolation for this worker, not an adversarial security certification",
        "sensor_manifest_sha256": hashlib.sha256((a.run / "sensor/frames.json").read_bytes()).hexdigest(),
    }
    (a.run / "replay-audit.json").write_text(json.dumps(result, indent=2))
    print(json.dumps(result))


if __name__ == "__main__":
    main()
