"""Publish a matched, explicitly kinematic CAD review and custom-part downloads."""
import argparse
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

from PIL import Image


def publish(cad, render, output):
    review = json.loads((render / "review.json").read_text())
    manifest_bytes = (cad / "assembly.json").read_bytes()
    if review["input_cad_manifest_sha256"] != hashlib.sha256(manifest_bytes).hexdigest():
        raise ValueError("Render and CAD manifest differ")
    if review["frames"] != 240 or review["physics_time_advanced"] != 0:
        raise ValueError("Unexpected review scope or length")
    output.mkdir(parents=True, exist_ok=True)
    for name in ("assembled", "exploded"):
        Image.open(render / (name + ".png")).save(output / (name + ".webp"), quality=90)
    for p in render.glob("phase-*.jpg"):
        shutil.copy2(p, output / p.name)
    for name in ("mechanism.mp4", "review.json"):
        shutil.copy2(render / name, output / name)
    for name in ("assembly-envelope.step", "printed-parts.step", "assembly.json", "audit.json"):
        shutil.copy2(cad / name, output / name)
    with zipfile.ZipFile(output / "fit-review-stls.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for p in sorted((cad / "print_fit_only").glob("*.stl")):
            z.write(p, p.name)
        z.writestr(
            "READ-ME.txt",
            "FIT REVIEW ONLY. Not released manufacturing parts. Screw retention, running fits, "
            "print orientation, structural loads and force cartridge remain unresolved. "
            "Models use assembly coordinates in millimetres. See the design README before printing.\n",
        )
    print(f"Published matched CAD and render from {cad.name} / {render.name}")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cad", type=Path, required=True)
    p.add_argument("--render", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    publish(a.cad, a.render, a.output)
