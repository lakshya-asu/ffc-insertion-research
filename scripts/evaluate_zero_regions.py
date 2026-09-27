"""Offline scorer only: compare frozen RGB predictions against withheld component labels."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage


def evaluate(data, predictions, output):
    labels = json.loads((data / "offline/labels.json").read_text())
    replay = json.loads((predictions / "predictions.json").read_text())
    classes = labels["classes"]
    assert classes == replay["classes"]
    rows = labels["frames"]
    lookup = {(r["file"], r["model"]): r for r in replay["frames"]}
    assert len(lookup) == 2 * len(rows)
    report = {
        "classes": classes,
        "models": {},
        "motion_permitted": False,
        "scope": "visible CAD component regions, not aperture or leading-edge accuracy",
        "model_sha256": replay["model_sha256"],
        "backbone_sha256": replay["backbone_sha256"],
        "labels_sha256": hashlib.sha256((data / "offline/labels.json").read_bytes()).hexdigest(),
        "frame_count": len(rows),
        "scene_count": len({r["scene"] for r in rows}),
        "crop_xyxy": replay["crop_xyxy"],
        "minimum_visible_pixels": 16,
        "timing_scope": replay["timing_scope"],
    }
    conditions = sorted({r["condition"] for r in rows})
    for model in ["dino", "rgb_ablation"]:
        groups = {name: [] for name in ["all", "test", "challenge", "entrance", "offset"] + conditions}
        details = []
        for row in rows:
            truth = np.array(Image.open(data / "offline" / row["file"]))
            pred = np.array(Image.open(predictions / model / row["file"]))
            if pred.shape != truth.shape or pred.max() >= len(classes):
                raise ValueError("Invalid prediction")
            cm = np.bincount(
                (truth.astype(int) * len(classes) + pred).ravel(), minlength=len(classes) ** 2
            ).reshape(len(classes), len(classes))
            per = []
            objects = {o["class"] for o in lookup[row["file"], model]["objects"]}
            for cls in range(1, len(classes)):
                t = truth == cls
                p = pred == cls
                visible = int(t.sum()) >= 16
                fp = (not visible) and classes[cls] in objects
                tp = int((t & p).sum())
                union = int((t | p).sum())
                iou = tp / union if union else None
                boundary95 = None
                if visible and p.any():
                    tb = t ^ ndimage.binary_erosion(t)
                    pb = p ^ ndimage.binary_erosion(p)
                    errors = np.r_[
                        ndimage.distance_transform_edt(~tb)[pb], ndimage.distance_transform_edt(~pb)[tb]
                    ]
                    boundary95 = float(np.percentile(errors, 95))
                per.append(
                    {
                        "class": classes[cls],
                        "visible": visible,
                        "false_positive": fp,
                        "iou": iou,
                        "silhouette_boundary_p95_px": boundary95,
                        "visible_iou_at_least_half": bool(visible and iou >= 0.5),
                    }
                )
            detail = {
                "file": row["file"],
                "split": row["split"],
                "condition": row["condition"],
                "camera_id": row["camera_id"],
                "metrics": per,
                "compute_s": lookup[row["file"], model]["compute_s"],
            }
            details.append(detail)
            for name in set(["all", row["split"], row["camera_id"], row["condition"]]):
                groups[name].append((cm, detail))
        summaries = {}
        for name, items in groups.items():
            if not items:
                continue
            cm = sum(x[0] for x in items)
            tp = cm.diagonal()
            union = cm.sum(0) + cm.sum(1) - tp
            iou = np.divide(tp, union, out=np.zeros(len(classes)), where=union > 0)
            scores = []
            for cls in range(1, len(classes)):
                entries = [x[1]["metrics"][cls - 1] for x in items]
                vis = [x for x in entries if x["visible"]]
                bounds = [
                    x["silhouette_boundary_p95_px"]
                    for x in vis
                    if x["silhouette_boundary_p95_px"] is not None
                ]
                scores.append(
                    {
                        "class": classes[cls],
                        "iou": float(iou[cls]) if union[cls] else None,
                        "visible_images": len(vis),
                        "visible_iou_at_least_half": sum(x["visible_iou_at_least_half"] for x in vis),
                        "absent_images": len(entries) - len(vis),
                        "false_positive_images": sum(x["false_positive"] for x in entries),
                        "median_silhouette_p95_px_on_detected": float(np.median(bounds)) if bounds else None,
                    }
                )
            summaries[name] = {
                "images": len(items),
                "foreground_mean_iou": (
                    float(iou[1:][union[1:] > 0].mean()) if (union[1:] > 0).any() else None
                ),
                "classes": scores,
            }
        times = [d["compute_s"] for d in details[1:]]
        report["models"][model] = {
            "groups": summaries,
            "frames": details,
            "latency_ms_excluding_first": {
                "median": float(np.median(times) * 1000),
                "p95": float(np.percentile(times, 95) * 1000),
            },
        }
    output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v["groups"]["all"] for k, v in report["models"].items()}, indent=2))


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("data", type=Path)
    p.add_argument("predictions", type=Path)
    p.add_argument("output", type=Path)
    a = p.parse_args()
    evaluate(a.data, a.predictions, a.output)
