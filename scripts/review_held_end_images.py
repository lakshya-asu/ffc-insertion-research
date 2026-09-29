"""Reproducible RGB-only review on saved human-view renders, not macro validation."""

import argparse
import hashlib
import json
import sys
from dataclasses import asdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.camera_observations import CameraGate, CameraSpec  # noqa: E402
from ffc.held_end_inspection import InspectionConfig, compare, inspect  # noqa: E402


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    source = ROOT / "docs/library/fr3-flex-008"
    start, end = [
        np.asarray(Image.open(source / name).convert("RGB")) for name in ["isaac-start.png", "isaac-end.png"]
    ]
    dark = np.rint(end.astype(float) * 0.2).astype(np.uint8)
    occluded = end.copy()
    h, w = end.shape[:2]
    occluded[int(0.3 * h) : int(0.64 * h), int(0.35 * w) : int(0.72 * w)] = 35
    decoy = end.copy()
    decoy[20:65, 20:75] = [40, 100, 220]
    cases = [
        ("saved-start", start, "Saved human-view RGB, initial pose"),
        ("saved-end", end, "Saved human-view RGB, lifted pose"),
        ("dark-edit", dark, "Image multiplied by 0.2; not a physical exposure simulation"),
        ("occlusion-edit", occluded, "Added rectangular obstruction; not a new scene"),
        ("blue-decoy-edit", decoy, "Added blue region exposes colour-only ambiguity"),
        ("empty-image", np.zeros_like(end), "Synthetic absent-input negative"),
    ]
    cfg = InspectionConfig("legacy-action-unqualified-v1")
    results = []
    for sequence, (name, pixels, note) in enumerate(cases):
        gate = CameraGate({"held": CameraSpec(w, h, cfg.calibration_id)})
        packet = {
            "camera_id": "held",
            "sequence": sequence,
            "timestamp_s": float(sequence),
            "width": w,
            "height": h,
            "calibration_id": cfg.calibration_id,
            "rgb": pixels.tobytes(),
        }
        frame = gate.accept(packet, float(sequence))
        result, cable_mask, tool_mask = inspect(frame, cfg)
        result.update(case=name, note=note, clock="review_index_seconds; not sensor acquisition time")
        results.append(result)
        Image.fromarray(pixels).save(a.output / (name + "-rgb.png"))
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.7), layout="constrained")
        for ax in axes:
            ax.imshow(pixels)
            ax.axis("off")
        overlay = np.zeros((h, w, 4))
        overlay[cable_mask] = [0, 0.7, 1, 0.5]
        overlay[tool_mask] = [1, 0.55, 0, 0.35]
        axes[1].imshow(overlay)
        for field, color in [("terminal_candidate", "#05e0ff"), ("tool_candidate", "#ffb04a")]:
            if result[field]:
                box = np.asarray(result[field]["oriented_extent_px"])
                box = np.vstack([box, box[0]])
                axes[1].plot(box[:, 0], box[:, 1], color=color, linewidth=1)
        for ax in axes:
            ax.set_xlim(0, w)
            ax.set_ylim(h, 0)
        axes[0].set_title("RGB input")
        axes[1].set_title("Appearance regions / extents, not leading-edge labels")
        fig.suptitle(name + " · motion disabled", fontsize=14)
        fig.savefig(a.output / (name + ".png"), dpi=120)
        plt.close(fig)
    report = {
        "scope": "Six diagnostic inputs from two saved human-view frames. No new Isaac run, "
        "macro-camera validation, learned inference or independent held-out dataset.",
        "config": asdict(cfg),
        "results": results,
        "start_end_comparison": compare(results[0], results[1]),
        "sources": {
            name: hashlib.sha256((source / name).read_bytes()).hexdigest()
            for name in ["isaac-start.png", "isaac-end.png"]
        },
        "code_sha256": {
            name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in ["src/ffc/held_end_inspection.py", "scripts/review_held_end_images.py"]
        },
    }
    (a.output / "report.json").write_text(json.dumps(report, indent=2))
    page = """<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Held-end inspection · review preparation</title><style>body{font:17px/1.65 system-ui;background:#f4f3ed;color:#253c37;max-width:1100px;margin:auto;padding:40px 24px}h1{font-size:48px;line-height:1.1;letter-spacing:-.04em}p{max-width:850px}a{color:#176b57}article{border-top:1px solid #bbc8c0;padding:24px 0}img{width:100%}code{background:#e3e8df;padding:2px 5px}</style><a href="../library/#mounted">← Mounted motion library</a><h1>Inspecting the held cable end.</h1><p><b>Review preparation, not a passed perception milestone.</b> Fresh Isaac capture is pending because this session cannot access Docker. This page reviews two saved human-view images and four diagnostic image edits. They are not calibrated macro-camera frames or independent held-out scenes.</p><p>The baseline uses colour to find candidate blue terminal and yellow tool regions. Boxes show image extents. They do not establish the physical leading edge, grasp position or insertion target. In the saved views, this simple colour baseline picks up the blue adapter/tool shading as well as the terminal and rejects the result as ambiguous; it has not solved terminal localization. Every result keeps motion disabled and slip unverified.</p><p>The proposed fixed camera reuses the Basler ace 2 / Kowa 35 mm simulation reference at 45°, with native optical dimensions and a half-resolution 1224 × 1024 output. Placement, occlusion, mount clearance and physical calibration still need review. The next capture exports raw RGB, intrinsics, camera transforms, clock identity and hashes.</p><p><a href="report.json">Read all measurements and source hashes</a> · <a href="README.md">Capture instructions and review criteria</a></p>"""  # noqa: E501
    for result in results:
        name = result["case"]
        page += f'<article><h2>{name}</h2><p>{result["note"]}</p><p>Reported: {", ".join(result["reasons"])}</p><img src="{name}.png" alt="RGB and appearance-region review for {name}" loading="lazy"><a href="{name}-rgb.png">Input pixels</a></article>'  # noqa: E501
    page += "<p>Next: render the proposed camera, label visible leading edge and occlusion independently, compare a learned edge estimate, and evaluate repeatable relative pose with uncertainty. Image centroid changes alone cannot establish mechanical slip.</p></html>"  # noqa: E501
    (a.output / "index.html").write_text(page)
    print(json.dumps({r["case"]: r["reasons"] for r in results}, indent=2))


if __name__ == "__main__":
    main()
