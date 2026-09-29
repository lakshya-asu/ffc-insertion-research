"""Audit evidence before collecting a released simulation skill dataset."""

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from ffc.cable_spec import load_spec  # noqa: E402
from ffc.dataset_release import review_release  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--spec", type=Path, default=ROOT / "config/cables/rpi-camera-standard-mini-200-rev2.json"
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument("--evidence-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = review_release(load_spec(args.spec), json.loads(args.manifest.read_text()), args.evidence_root)
    with args.output.open("x") as stream:
        stream.write(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["evidence_gate_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
