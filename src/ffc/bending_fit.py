"""Select an effective gain using fitting cases only; never consult validation."""

import math

import numpy as np


def rotated_offset(quaternion_wxyz, offset):
    """Rotate an offset without losing near-identity float32 quaternion angles.

    Normalize in float64 and use the quaternion vector formula, avoiding acos(w).
    PhysX may return w rounded to one while its vector part still carries rotation.
    """
    q = np.asarray(quaternion_wxyz, dtype=np.float64)
    v = np.asarray(offset, dtype=np.float64)
    if q.shape != (4,) or v.shape != (3,) or not np.isfinite(q).all() or not np.isfinite(v).all():
        raise ValueError("Expected finite wxyz quaternion and xyz offset")
    scale = np.max(np.abs(q))
    if scale == 0:
        raise ValueError("Zero quaternion has no orientation")
    q = q / scale
    q /= np.linalg.norm(q)
    cross = 2 * np.cross(q[1:], v)
    return v + q[0] * cross + np.cross(q[1:], cross)


def select_gain(report: dict) -> dict:
    if report["settings"]["split"] != "fit":
        raise ValueError("Validation results cannot select simulator parameters")
    groups = {}
    for case in report["cases"]:
        if case["split"] != "fit":
            raise ValueError("Mixed fitting and validation cases")
        target = case["reference"]["nonlinear_hinge_chain_prediction_m"]
        observed = case["observed_sag_m"]
        if not math.isfinite(target) or target <= 0 or not math.isfinite(observed):
            raise ValueError("Invalid fitting response")
        groups.setdefault(case["gain"], []).append(case)
    if not groups:
        raise ValueError("Empty fit")
    expected = None
    scores = []
    for gain, rows in groups.items():
        keys = [(c["length_m"], c["modulus_multiplier"], c["uniform_load_multiplier"]) for c in rows]
        if len(set(keys)) != len(keys):
            raise ValueError("Duplicate fitting cases")
        if expected is not None and set(keys) != expected:
            raise ValueError("Candidate gains used different fitting cases")
        expected = set(keys)
        errors = [
            c["observed_sag_m"] / c["reference"]["nonlinear_hinge_chain_prediction_m"] - 1 for c in rows
        ]
        scores.append(
            {
                "gain": gain,
                "relative_rmse": math.sqrt(sum(e * e for e in errors) / len(errors)),
                "max_relative_error": max(map(abs, errors)),
                "all_fit_cases_passed": all(c["passed"] for c in rows),
            }
        )
    best = min(scores, key=lambda s: s["relative_rmse"])
    return {
        "selected_gain": best["gain"],
        "candidates": scores,
        "fit_passed": best["all_fit_cases_passed"],
        "selection_used_validation": False,
        "scope": "Static bending only; not full cable or contact qualification",
    }
