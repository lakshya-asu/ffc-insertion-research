"""Offline-only scorer. Never imported by the perception worker or controller."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

CLASSES = ("background", "cable", "contacts", "stiffener", "csi", "dsi")


def largest(mask):
    cc, count = ndimage.label(mask)
    if not count:
        return None
    sizes = np.bincount(cc.ravel())
    sizes[0] = 0
    label = int(sizes.argmax())
    if sizes[label] < 8:
        return None
    yy, xx = np.where(cc == label)
    return np.array([xx.mean(), yy.mean()])


def summarize(rows):
    confusion = sum((np.array(r["confusion"]) for r in rows), np.zeros((6, 6), dtype=np.int64))
    tp = confusion.diagonal()
    union = confusion.sum(0) + confusion.sum(1) - tp
    iou = np.divide(tp, union, out=np.full(6, np.nan), where=union > 0)
    detection = {}
    for name in CLASSES[1:]:
        values = [r["objects"][name] for r in rows]
        visible = [v for v in values if v["target_present"]]
        absent = [v for v in values if not v["target_present"]]
        distances = [v["centroid_error_px"] for v in visible if v["prediction_present"]]
        detection[name] = {
            "visible_frames": len(visible),
            "detected_frames": sum(v["prediction_present"] for v in visible),
            "localized_within_12px": sum(
                v["centroid_error_px"] is not None and v["centroid_error_px"] <= 12 for v in visible
            ),
            "absent_frames": len(absent),
            "false_positive_frames": sum(v["prediction_present"] for v in absent),
            "median_centroid_error_px": float(np.median(distances)) if distances else None,
        }
    return {
        "images": len(rows),
        "iou": {k: float(v) if np.isfinite(v) else None for k, v in zip(CLASSES, iou, strict=True)},
        "foreground_mean_iou": float(np.nanmean(iou[1:])),
        "objects": detection,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--data", type=Path, required=True)
    p.add_argument("--predictions", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    a = p.parse_args()
    truth = json.loads((a.data / "offline/labels.json").read_text())["frames"]
    prediction_report = json.loads((a.predictions / "predictions.json").read_text())
    predictions = {v["file"]: v for v in prediction_report["frames"]}
    if set(predictions) != {r["file"] for r in truth}:
        raise ValueError("Prediction/annotation manifest mismatch")
    rows = []
    for item in truth:
        gt = np.array(Image.open(a.data / "offline" / item["file"]))
        pred = np.array(Image.open(a.predictions / item["file"]))
        if gt.shape != pred.shape or pred.max() > 5:
            raise ValueError("Invalid prediction mask")
        confusion = np.bincount((gt.astype(np.int64) * 6 + pred).ravel(), minlength=36).reshape(6, 6)
        objects = {}
        for cls, name in enumerate(CLASSES[1:], 1):
            g, v = largest(gt == cls), largest(pred == cls)
            objects[name] = {
                "target_present": g is not None,
                "prediction_present": v is not None,
                "centroid_error_px": float(np.linalg.norm(g - v))
                if g is not None and v is not None
                else None,
            }
        rows.append(
            {k: item[k] for k in ["file", "split", "camera_id", "condition"]}
            | {"confusion": confusion.tolist(), "objects": objects}
        )
    groups = {"all": summarize(rows)}
    for field in ["split", "camera_id", "condition"]:
        groups[field] = {
            value: summarize([r for r in rows if r[field] == value])
            for value in sorted({r[field] for r in rows})
        }
    latencies = [r["latency_s"] for r in predictions.values()]
    report = {
        "groups": groups,
        "frames": rows,
        "model_sha256": prediction_report["model_sha256"],
        "label_sha256": hashlib.sha256((a.data / "offline/labels.json").read_bytes()).hexdigest(),
        "cpu_latency_ms": {
            "median": float(np.median(latencies) * 1000),
            "p95": float(np.percentile(latencies, 95) * 1000),
        },
        "object_metric": (
            "largest 4-connected region of at least 8 pixels; centroid error is NOT insertion pose error"
        ),
        "evaluation_scope": "held-out static synthetic scenes; fixed ideal cameras; same CAD assets",
        "real_camera_validation": False,
        "motion_permitted": False,
        "missing_before_motion": [
            "real camera calibration and holdout",
            "cable endpoint and orientation uncertainty",
            "socket mouth geometry calibration",
            "occlusion and abstention policy",
            "measured contact and tactile feedback",
        ],
    }
    a.output.write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(groups, indent=2))


if __name__ == "__main__":
    main()
