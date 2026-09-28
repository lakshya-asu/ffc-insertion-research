"""Offline-only calibration/evaluation; labels never enter the inference worker."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.stats import beta

from ffc.zero_reliability import apply_review_gate


def score_rows(data, predictions):
    truth_report = json.loads((data / "offline/labels.json").read_text())
    replay = json.loads((predictions / "predictions.json").read_text())
    assert list(replay["classes"]) == truth_report["classes"]
    lookup = {r["file"]: r for r in replay["frames"]}
    assert len(lookup) == len(truth_report["frames"])
    rows = []
    matrices = {"ensemble": [], "previous": []}
    for label in truth_report["frames"]:
        truth = np.array(Image.open(data / "offline" / label["file"]))
        assert truth.shape == (448, 896)
        row = {k: label[k] for k in ["file", "scene", "camera_id", "condition", "split"]}
        row.update(lookup[label["file"]])
        row["metrics"] = {}
        for name in matrices:
            pred = np.array(Image.open(predictions / name / label["file"]))
            assert pred.shape == truth.shape and pred.max() < 5
            cm = np.bincount((truth.astype(int) * 5 + pred).ravel(), minlength=25).reshape(5, 5)
            matrices[name].append(cm)
            per = []
            for cls in range(1, 5):
                t, p = truth == cls, pred == cls
                union = int((t | p).sum())
                per.append(
                    {
                        "class": truth_report["classes"][cls],
                        "visible": int(t.sum()) >= 16,
                        "iou": float((t & p).sum() / union) if union else None,
                    }
                )
            row["metrics"][name] = per
        target = row["metrics"]["ensemble"]
        row["good_component_pair"] = all(target[i]["visible"] and target[i]["iou"] >= 0.75 for i in [0, 2])
        row["pair_visible"] = all(target[i]["visible"] for i in [0, 2])
        rows.append(row)
    return rows, matrices, replay


def model_summary(rows, matrices):
    summaries = {}
    for name, values in matrices.items():
        cm = sum(values)
        tp = cm.diagonal()
        union = cm.sum(0) + cm.sum(1) - tp
        iou = np.divide(tp, union, out=np.zeros(5), where=union > 0)
        per = []
        for i in range(4):
            visible = [r["metrics"][name][i] for r in rows if r["metrics"][name][i]["visible"]]
            per.append(
                {
                    "class": rows[0]["metrics"][name][i]["class"],
                    "pooled_iou": float(iou[i + 1]),
                    "visible_images": len(visible),
                    "mean_visible_image_iou": float(np.mean([x["iou"] for x in visible]))
                    if visible
                    else None,
                    "visible_iou_at_least_half": sum(x["iou"] >= 0.5 for x in visible),
                    "visible_iou_at_least_three_quarters": sum(x["iou"] >= 0.75 for x in visible),
                }
            )
        summaries[name] = {"foreground_mean_iou": float(iou[1:].mean()), "classes": per}
    return summaries


def gate_summary(rows, policy, interval=False):
    accepted = [r for r in rows if apply_review_gate(r["evidence"], r["camera_id"], policy)["accepted"]]
    bad = [r for r in accepted if not r["good_component_pair"]]
    good = [r for r in rows if r["good_component_pair"]]
    n, failures = len(accepted), len(bad)
    upper = None
    if interval and n:
        upper = 1.0 if failures == n else float(beta.ppf(0.975, failures + 1, n - failures))
    return {
        "images": len(rows),
        "accepted": n,
        "abstained": len(rows) - n,
        "coverage": n / len(rows) if rows else None,
        "bad_accepts": failures,
        "empirical_bad_accept_rate": failures / n if n else None,
        "nominal_bad_accept_upper_97_5pct": upper,
        "good_pairs_available": len(good),
        "good_pairs_accepted": n - failures,
        "good_pair_retention": (n - failures) / len(good) if good else None,
        "bad_accept_files": [r["file"] for r in bad],
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("mode", choices=["calibrate", "evaluate"])
    p.add_argument("data", type=Path)
    p.add_argument("predictions", type=Path)
    p.add_argument("output", type=Path)
    p.add_argument("--policy", type=Path)
    a = p.parse_args()
    if a.output.exists():
        raise ValueError("Fresh report output required")
    rows, matrices, replay = score_rows(a.data, a.predictions)
    if a.mode == "calibrate":
        assert {r["split"] for r in rows} == {"calibration"}
        thresholds, calibration = {}, {}
        grid = [0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95]
        for camera in sorted({r["camera_id"] for r in rows}):
            subset = [r for r in rows if r["camera_id"] == camera]
            candidates = []
            for threshold in grid:
                summary = gate_summary(subset, {"thresholds": {camera: threshold}})
                candidates.append({"threshold": threshold, **summary})
            feasible = [r for r in candidates if r["bad_accepts"] == 0 and r["accepted"] >= 20]
            best = max(feasible, key=lambda r: r["accepted"]) if feasible else None
            thresholds[camera] = best["threshold"] if best else None
            calibration[camera] = {"grid": candidates, "selected": best}
        report = {
            "thresholds": thresholds,
            "calibration": calibration,
            "model_sha256": replay["model_sha256"],
            "backbone_sha256": replay["backbone_sha256"],
            "runtime_code_sha256": replay["runtime_code_sha256"],
            "calibration_ids": replay["calibration_ids"],
            "evidence_code_sha256": replay["evidence_code_sha256"],
            "calibration_labels_sha256": hashlib.sha256(
                (a.data / "offline/labels.json").read_bytes()
            ).hexdigest(),
            "motion_permitted": False,
            "scope": "provisional component review, not insertion readiness",
            "test_used_for_threshold_selection": False,
            "selection": (
                "maximum calibration coverage, zero observed bad accepts, "
                "at least 20 accepts per camera; otherwise abstain"
            ),
            "good_definition": "both connector and mini-end visible; both mask IoU >= 0.75",
        }
    else:
        assert {r["split"] for r in rows} == {"test"}
        if a.policy is None:
            raise ValueError("Frozen policy required")
        policy = json.loads(a.policy.read_text())
        assert replay["policy_sha256"] == hashlib.sha256(a.policy.read_bytes()).hexdigest()
        assert replay["model_sha256"] == policy["model_sha256"]
        for row in rows:
            assert row["decision"] == apply_review_gate(row["evidence"], row["camera_id"], policy)
        report = {
            "models": model_summary(rows, matrices),
            "all": gate_summary(rows, policy),
            "cameras": {
                c: gate_summary([r for r in rows if r["camera_id"] == c], policy, interval=True)
                for c in sorted({r["camera_id"] for r in rows})
            },
            "conditions": {
                c: gate_summary([r for r in rows if r["condition"] == c], policy)
                for c in sorted({r["condition"] for r in rows})
            },
            "model_sha256": replay["model_sha256"],
            "backbone_sha256": replay["backbone_sha256"],
            "runtime_code_sha256": replay["runtime_code_sha256"],
            "policy_sha256": replay["policy_sha256"],
            "crop_xyxy": replay["crop_xyxy"],
            "classes": replay["classes"],
            "thresholds": policy["thresholds"],
            "frames": rows,
            "stress": replay["stress"],
            "motion_permitted": False,
            "compute_ms_median": float(np.median([r["compute_s"] for r in rows[1:]]) * 1000),
            "timing_scope": replay["timing_scope"],
            "interval_interpretation": (
                "Per-camera one-sided 97.5% exact binomial upper limits; "
                "two-camera Bonferroni 95% family coverage only under independent, identically distributed "
                "scene draws from this synthetic generator. No pooled two-camera independence "
                "or real-world guarantee is claimed."
            ),
        }
    a.output.write_text(json.dumps(report, indent=2))
    print(
        json.dumps(
            {
                k: report[k]
                for k in (["thresholds"] if a.mode == "calibrate" else ["models", "all", "cameras"])
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
