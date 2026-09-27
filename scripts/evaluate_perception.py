"""Offline-only scoring; this process may read renderer labels, the detector may not."""

import argparse
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw


def evaluate(run: Path, prediction_dir: str):
    predictions = json.loads((run / prediction_dir / "predictions.json").read_text())["predictions"]
    truth = {x["file"]: x for x in json.loads((run / "offline/labels.json").read_text())}
    calibration = json.loads((run / "sensor/calibration.json").read_text())
    rows = []
    review = run / prediction_dir / "review"
    review.mkdir(exist_ok=True)
    for index, p in enumerate(predictions):
        t = truth[p["file"]]
        case = t["case"]
        actual = np.array(Image.open(run / "offline" / t["mask"])) > 0
        predicted = np.array(Image.open(run / prediction_dir / p["mask"])) > 0
        union = (actual | predicted).sum()
        iou = float((actual & predicted).sum() / union) if union else 1.0
        name = "global" if "global" in p["file"] else "local"
        c = calibration[name]
        angle = np.deg2rad(case["angle"])
        direction = np.array([np.cos(angle), np.sin(angle), 0.0])
        center = np.array([0.375 + case["dx"], -0.18 + case["dy"], 0.00015])
        endpoints = np.array([center - 0.075 * direction, center + 0.075 * direction])
        camera = np.column_stack([endpoints, np.ones(2)]) @ np.linalg.inv(
            c["world_from_camera_usd_row_matrix"]
        )
        uv = np.column_stack(
            [
                c["K"][0][0] * camera[:, 0] / -camera[:, 2] + c["K"][0][2],
                c["K"][1][2] - c["K"][1][1] * camera[:, 1] / -camera[:, 2],
            ]
        )
        both_visible = (
            bool(
                np.all((uv[:, 0] >= 0) & (uv[:, 0] < c["width"]) & (uv[:, 1] >= 0) & (uv[:, 1] < c["height"]))
            )
            and case["condition"] != "absent"
        )
        error = None
        if both_visible and p.get("unordered_endpoints_uv") is not None:
            ends = np.array(p["unordered_endpoints_uv"])
            error = float(
                min(np.linalg.norm(ends - uv, axis=1).max(), np.linalg.norm(ends - uv[::-1], axis=1).max())
            )
        row = dict(
            file=p["file"],
            split=case["split"],
            condition=case["condition"],
            camera=name,
            visible_pixels=int(actual.sum()),
            detected=p["detected"],
            iou=iou,
            both_physical_endpoints_in_image=both_visible,
            maximum_endpoint_error_px=error,
            mask_gate_pass=iou >= 0.85,
            full_endpoint_gate_pass=both_visible and error is not None and error <= 3.0,
            false_positive=bool(p["detected"] and not actual.any()),
        )
        rows.append(row)
        img = Image.open(run / "sensor" / p["file"]).convert("RGB")
        draw = ImageDraw.Draw(img)
        # Evaluation annotations belong in review outputs only.
        if p.get("unordered_endpoints_uv") is not None:
            for x, y in p["unordered_endpoints_uv"]:
                draw.ellipse((x - 5, y - 5, x + 5, y + 5), outline=(255, 30, 40), width=2)
        if case["condition"] != "absent":
            for x, y in uv:
                draw.ellipse((x - 4, y - 4, x + 4, y + 4), outline=(30, 255, 80), width=2)
        draw.rectangle((0, 0, img.width, 45), fill=(20, 30, 32))
        draw.text((10, 8), f"{case['id']} / {name} / IoU {iou:.3f} / detected {p['detected']}", fill="white")
        draw.text(
            (10, 25),
            "STATIC TEST CASE | red: predicted ends; green: offline reference | no robot motion",
            fill="white",
        )
        img.save(review / f"{index:03d}.png")
    groups = {}
    for split in sorted({r["split"] for r in rows}):
        selected = [r for r in rows if r["split"] == split]
        positive = [r for r in selected if r["visible_pixels"]]
        eligible = [r for r in positive if r["both_physical_endpoints_in_image"]]
        groups[split] = {
            "images": len(selected),
            "positive_images": len(positive),
            "detections_on_positive": sum(r["detected"] for r in positive),
            "mean_positive_iou": float(np.mean([r["iou"] for r in positive])) if positive else None,
            "mask_gate_passes": sum(r["mask_gate_pass"] for r in positive),
            "endpoint_eligible_images": len(eligible),
            "endpoint_gate_passes": sum(r["full_endpoint_gate_pass"] for r in eligible),
            "false_positives": sum(r["false_positive"] for r in selected),
        }
    report = {
        "thresholds": {
            "mask_iou_minimum": 0.85,
            "maximum_endpoint_error_px": 3.0,
            "scope": "predeclared synthetic image checks, not hardware tolerances",
        },
        "groups": groups,
        "rows": rows,
        "motion_qualified": False,
        "contact_face_qualified": False,
        "limits": [
            "same synthetic cable asset",
            "static straight cable only",
            "ideal camera optics",
            "no real data",
            "visible-pixel mask score does not prove full-object visibility",
        ],
    }
    (run / prediction_dir / "evaluation.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(groups, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--predictions", default="predictions-v2")
    a = parser.parse_args()
    evaluate(a.run, a.predictions)
