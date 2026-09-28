"""Offline-only feature scoring on frozen test predictions; no thresholds fitted here."""

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
from scipy.ndimage import binary_erosion, distance_transform_edt

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("data", type=Path)
p.add_argument("predictions", type=Path)
p.add_argument("output", type=Path)
a = p.parse_args()
if a.output.exists():
    p.error("Fresh output required")
truth = json.loads((a.data / "offline/labels.json").read_text())
assert not any(json.loads((a.data / "sensor/camera.json").read_text())["d"]), (
    "Nonzero distortion requires matched mask rectification"
)
inference = json.loads((a.predictions / "predictions.json").read_text())
assert len(truth["frames"]) == len(inference["frames"])
assert {r["split"] for r in truth["frames"]} == {"test"}
assert {r["file"]: r["rgb_sha256"] for r in truth["frames"]} == {
    r["file"]: r["rgb_sha256"] for r in inference["frames"]
}
classes = inference["classes"]
cm = np.zeros((6, 6), np.int64)
rows = []


def boundary_f1(gt, pred):
    a = gt & ~binary_erosion(gt)
    b = pred & ~binary_erosion(pred)
    if not a.any() or not b.any():
        return 0.0
    precision = float((distance_transform_edt(~a)[b] <= 2).mean())
    recall = float((distance_transform_edt(~b)[a] <= 2).mean())
    return 2 * precision * recall / (precision + recall) if precision + recall else 0.0


for row in truth["frames"]:
    native = np.array(Image.open(a.data / "offline" / row["file"]))
    gt = cv2.copyMakeBorder(
        cv2.resize(native, (1224, 1024), interpolation=cv2.INTER_NEAREST_EXACT),
        0,
        0,
        4,
        4,
        cv2.BORDER_CONSTANT,
        value=0,
    )
    pred = np.array(Image.open(a.predictions / row["file"]))
    assert pred.shape == gt.shape and pred.max() < 6
    cm += np.bincount((gt.astype(int) * 6 + pred).ravel(), minlength=36).reshape(6, 6)
    scores = []
    for c in range(1, 6):
        g, b = gt == c, pred == c
        union = (g | b).sum()
        scores.append(
            dict(
                class_id=c,
                visible=int(g.sum()) >= 8,
                gt_pixels=int(g.sum()),
                predicted_pixels=int(b.sum()),
                iou=float((g & b).sum() / union) if union else None,
                boundary_f1_2px=boundary_f1(g, b) if g.any() else None,
            )
        )
    pg = [int((pred == c).sum()) for c in [4, 5]]
    decision = "unknown" if max(pg) < 32 else ("open" if pg[0] > pg[1] else "closed")
    visible_latch = sum(int((gt == c).sum()) for c in [4, 5]) >= 8
    rows.append(
        dict(
            file=row["file"],
            scene=row["scene"],
            camera="macro",
            condition=row["condition"],
            scores=scores,
            latch_visible=visible_latch,
            authored_slider_state="open" if row["slider_open"] else "closed",
            predicted_slider_state=decision,
        )
    )
summary = []
for c, name in enumerate(classes[1:], 1):
    values = [r["scores"][c - 1] for r in rows]
    visible = [s for s in values if s["visible"]]
    union = cm[c].sum() + cm[:, c].sum() - cm[c, c]
    summary.append(
        dict(
            name=name,
            pooled_iou=float(cm[c, c] / union) if union else None,
            visible_images=len(visible),
            mean_visible_iou=float(np.mean([s["iou"] for s in visible])) if visible else None,
            mean_boundary_f1_2px=float(np.mean([s["boundary_f1_2px"] for s in visible])) if visible else None,
            absent_images=sum(not s["visible"] for s in values),
            absent_false_positives_8px=sum(not s["visible"] and s["predicted_pixels"] >= 8 for s in values),
        )
    )
visible = [r for r in rows if r["latch_visible"]]
report = dict(
    classes=summary,
    frames=rows,
    confusion=cm.tolist(),
    inference=inference,
    slider=dict(
        visible_views=len(visible),
        correct=sum(r["predicted_slider_state"] == r["authored_slider_state"] for r in visible),
        unknown=sum(r["predicted_slider_state"] == "unknown" for r in visible),
        unobservable_views=sum(not r["latch_visible"] for r in rows),
        predictions_on_unobservable_views=sum(
            not r["latch_visible"] and r["predicted_slider_state"] != "unknown" for r in rows
        ),
    ),
    scope="Simulation geometry; 2-pixel boundary tolerance is descriptive, not insertion clearance",
    motion_permitted=False,
    thresholds_fitted_on_test=False,
)
a.output.write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(dict(classes=summary, slider=report["slider"]), indent=2))
