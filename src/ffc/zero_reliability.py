"""RGB-derived mask diagnostics and a fail-closed, non-motion review gate."""

import math

import numpy as np
from scipy import ndimage


def review_evidence(probabilities, member_masks, rgb):
    """No labels/poses. Score is a ranking statistic, never a probability of success."""
    if probabilities.shape != (5, *rgb.shape[:2]) or member_masks.shape != (2, *rgb.shape[:2]):
        raise ValueError("Unexpected model output shape")
    if not np.isfinite(probabilities).all():
        return {"eligible": False, "score": 0.0, "reasons": ["nonfinite_model_output"], "classes": {}}
    mask = probabilities.argmax(0)
    reasons = []
    # Constant, black or white sensor failures. These checks do not validate lens focus.
    if float(rgb.std()) < 2 or float(rgb.mean()) < 3 or float(rgb.mean()) > 252:
        reasons.append("uninformative_camera_frame")
    detail_energy = float(ndimage.laplace(ndimage.gaussian_filter(rgb.astype(float).mean(2), 1)).var())
    if detail_energy < 0.02:
        reasons.append("insufficient_image_detail")
    records = {}
    scores = []
    for cls, name, minimum in [(1, "connector", 64), (3, "mini_end", 32)]:
        pred = mask == cls
        cc, _ = ndimage.label(pred)
        counts = np.bincount(cc.ravel())
        counts[0] = 0
        area = int(counts.max(initial=0))
        if area < minimum:
            reasons.append(name + "_too_small_or_missing")
        if pred[0].any() or pred[-1].any() or pred[:, 0].any() or pred[:, -1].any():
            reasons.append(name + "_cut_by_window")
        p0, p1 = member_masks[0] == cls, member_masks[1] == cls
        union = int((p0 | p1).sum())
        agreement = float((p0 & p1).sum() / union) if union else 0.0
        confidence = float(probabilities[cls][pred].mean()) if pred.any() else 0.0
        scores.extend([agreement, confidence])
        records[name] = {
            "largest_component_pixels": area,
            "member_iou": agreement,
            "mean_softmax": confidence,
        }
    return {
        "eligible": not reasons,
        "score": min(scores),
        "reasons": reasons,
        "classes": records,
        "detail_energy": detail_energy,
    }


def apply_review_gate(evidence, camera_id, policy):
    """Acceptance means a provisional component review, never robot permission."""
    threshold = policy.get("thresholds", {}).get(camera_id)
    reasons = list(evidence.get("reasons", []))
    score = evidence.get("score")
    if threshold is None:
        reasons.append("camera_not_qualified_by_calibration")
    elif not isinstance(threshold, (int, float)) or not math.isfinite(threshold) or not 0 <= threshold <= 1:
        reasons.append("invalid_policy_threshold")
    elif not isinstance(score, (int, float)) or not math.isfinite(score) or not 0 <= score <= 1:
        reasons.append("invalid_evidence_score")
    elif score < threshold:
        reasons.append("insufficient_model_agreement_or_confidence")
    if evidence.get("eligible") is not True and not reasons:
        reasons.append("ineligible_evidence")
    accepted = not reasons
    return {
        "status": "provisional_component_review" if accepted else "need_another_view",
        "accepted": accepted,
        "reasons": reasons,
        "score": score,
        "motion_permitted": False,
        "insertion_pose": "unknown",
        "latch_state": "unknown",
    }
