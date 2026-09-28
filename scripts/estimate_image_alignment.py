"""Image-space geometry from saved predictions only; no annotations or scene state."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image

from ffc.image_alignment import estimate_alignment

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("predictions", type=Path)
p.add_argument("protocol", type=Path)
p.add_argument("output", type=Path)
a = p.parse_args()
if a.output.exists():
    p.error("Fresh output required")
protocol = json.loads(a.protocol.read_text())
manifest = json.loads((a.predictions / "predictions.json").read_text())
rows = []
for item in manifest["frames"]:
    path = a.predictions / item["file"]
    rows.append(
        dict(
            file=item["file"],
            rgb_sha256=item["rgb_sha256"],
            prediction_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            **estimate_alignment(np.array(Image.open(path)), protocol),
        )
    )
a.output.parent.mkdir(parents=True, exist_ok=True)
a.output.write_text(
    json.dumps(
        dict(
            frames=rows,
            protocol=protocol,
            protocol_sha256=hashlib.sha256(a.protocol.read_bytes()).hexdigest(),
            estimator_sha256=hashlib.sha256(
                Path(__import__("ffc.image_alignment", fromlist=["x"]).__file__).read_bytes()
            ).hexdigest(),
            model_sha256=manifest["model_sha256"],
            scope="Retrospective image-space diagnostic; no depth or controls",
        ),
        indent=2,
    )
    + "\n"
)
print("Measurements:", sum(r["status"] == "image_measurement" for r in rows), "/", len(rows))
