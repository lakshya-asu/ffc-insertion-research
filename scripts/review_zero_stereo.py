"""Offline RGB stereo evaluation. Truth is read only after the estimator returns."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from pxr import Gf, Usd, UsdGeom

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.stereo_terminal import estimate, estimate_top_surface  # noqa: E402


def review(capture, replay, output, method="silhouette"):
    output.mkdir(parents=True, exist_ok=False)
    if (capture / "inspection-observation.json").exists():
        observation = json.loads((capture / "inspection-observation.json").read_text())
        manifest = {
            "frames": [
                {
                    "sequence": 0,
                    "time_s": observation["acquisition_simulation_time_s"],
                    "images": observation["images"][:2],
                }
            ]
        }
        cameras = json.loads((capture / "camera-calibration.json").read_text())[:2]
        scope = (
            "Single fresh physics lift; same controlled presentation, no randomized reliability evaluation"
        )
    else:
        manifest = json.loads((capture / "manifest.json").read_text())
        cameras = json.loads((capture / "calibration.json").read_text())
        scope = "Development capture; adjacent frames of one lift, not held-out trials"
    stage = Usd.Stage.Open(str(replay))
    rows = []
    for frame in manifest["frames"]:
        images = [np.array(Image.open(capture / name).convert("RGB")) for name in frame["images"]]
        row = dict(frame)
        try:
            estimator = estimate_top_surface if method == "top" else estimate
            result = estimator(*images, *[c["P"] for c in cameras])
            row["prediction"] = result
        except (ValueError, np.linalg.LinAlgError) as error:
            row.update(abstained=True, reason=str(error))
            rows.append(row)
            continue
        # Offline-only: the exact top leading edge of the recorded first segment.
        matrix = UsdGeom.XformCache(frame["time_s"] * stage.GetTimeCodesPerSecond()).GetLocalToWorldTransform(
            stage.GetPrimAtPath("/World/Cable/segment_000")
        )
        truth = np.array([matrix.Transform(Gf.Vec3d(x, -0.001, 0.00015)) for x in [-0.00575, 0.00575]])
        center = truth.mean(axis=0)
        delta = np.array(result["tip_center_m"]) - center
        row.update(
            offline_top_edge_center_m=center.tolist(),
            error_xyz_um=(delta * 1e6).tolist(),
            error_norm_um=float(np.linalg.norm(delta) * 1e6),
        )
        for i, (rgb, c) in enumerate(zip(images, cameras, strict=True)):
            image = Image.fromarray(rgb)
            draw = ImageDraw.Draw(image)
            pixels = np.c_[truth, np.ones(2)] @ np.asarray(c["P"]).T
            pixels = pixels[:, :2] / pixels[:, 2:]
            draw.line([tuple(v) for v in pixels], fill="white", width=3)
            key = "left_corners_px" if i == 0 else "right_corners_px"
            poly = [tuple(v) for v in result[key]]
            draw.line(poly + [poly[0]], fill="#ffad40", width=2)
            name = f"overlay-{frame['sequence']}-{i}.png"
            image.save(output / name)
        rows.append(row)
    report = {
        "source_capture": capture.name,
        "scope": scope,
        "estimator_sha256": hashlib.sha256((ROOT / "src/ffc/stereo_terminal.py").read_bytes()).hexdigest(),
        "ground_truth": "Offline recorded USD first-segment top edge; never estimator input",
        "surface_warning": "Blue surface measurement is not the terminal midplane",
        "motion_permitted": False,
        "frames": rows,
    }
    (output / "score.json").write_text(json.dumps(report, indent=2))
    print(
        json.dumps(
            [
                {"sequence": r["sequence"], "error_um": r.get("error_norm_um"), "reason": r.get("reason")}
                for r in rows
            ]
        )
    )


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--capture", type=Path, required=True)
    p.add_argument("--replay", type=Path, default=ROOT / "docs/library/fr3-flex-008/replay.usdz")
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--method", choices=["silhouette", "top"], default="silhouette")
    args = p.parse_args()
    review(args.capture, args.replay, args.output, args.method)
