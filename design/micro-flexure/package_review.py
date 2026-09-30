"""Package CAD review images, a dimension envelope sheet and source identities."""

import argparse
import hashlib
import json
import shutil
import zipfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from PIL import Image


def package(output):
    source = Path(__file__).resolve().parent
    fig, axes = plt.subplots(1, 2, figsize=(16, 9), facecolor="#f3f2ee")
    fig.suptitle("Envelope dimensions / concept 01", fontsize=25, color="#213b3c", y=0.94)
    for ax, title, width, height in [
        (axes[0], "Top envelope", 30, 28),
        (axes[1], "Side envelope", 30, 24.7),
    ]:
        ax.add_patch(Rectangle((0, 0), width, height, fill=False, lw=2, ec="#213b3c"))
        ax.annotate("", (0, -3), (width, -3), arrowprops={"arrowstyle": "<->"})
        ax.text(width / 2, -5, f"{width:g} mm", ha="center", fontsize=15)
        ax.annotate("", (width + 3, 0), (width + 3, height), arrowprops={"arrowstyle": "<->"})
        ax.text(width + 5, height / 2, f"{height:g} mm", rotation=90, va="center", fontsize=15)
        ax.text(
            width / 2,
            height / 2,
            "Complete head envelope\nincluding fingers and suction",
            ha="center",
            va="center",
            fontsize=12,
        )
        ax.set(xlim=(-3, 40), ylim=(-8, 35), aspect="equal", title=title)
        ax.set_axis_off()
    fig.text(
        0.08,
        0.12,
        "Main body 22 × 20 × 23 mm     •     Pad footprint 2 × 6 mm     •     Open pad gap 1.20 mm",
        fontsize=13,
    )
    fig.text(
        0.08,
        0.065,
        "Envelope diagram, not a manufacturing drawing. Adapter, wiring and tubing clearance are additional.",
        fontsize=11,
    )
    fig.savefig(output / "dimensions.png", dpi=130)
    plt.close(fig)
    images = [
        Image.open(output / name).convert("RGB")
        for name in ["concept.png", "board-scale.png", "open-closed.png", "dimensions.png"]
    ]
    images[0].save(output / "design-review.pdf", save_all=True, append_images=images[1:], resolution=130)
    for name in ["render.py", "raster.cpp", "review.py", "package_review.py", "README.md"]:
        shutil.copy2(source / name, output / name)
    identities = {
        p.name: hashlib.sha256(p.read_bytes()).hexdigest()
        for p in sorted(output.iterdir())
        if p.is_file() and p.suffix != ".zip" and p.name != "manifest.json"
    }
    board = source.parents[1] / "third_party/raspberry_pi/zero/zero2w.usdc"
    manifest = {
        "files_sha256": identities,
        "board_asset_sha256": hashlib.sha256(board.read_bytes()).hexdigest(),
        "physics": False,
        "scope": "Design review; prescribed animation only",
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2))
    with zipfile.ZipFile(output / "micro-flexure-review.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(output.iterdir()):
            if path.is_file() and path.suffix != ".zip":
                archive.write(path, path.name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    package(parser.parse_args().output)
