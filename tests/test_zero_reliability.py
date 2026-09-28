"""The review gate must refuse incomplete evidence and never grant robot motion."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest

from ffc.zero_reliability import apply_review_gate, review_evidence


def inputs():
    mask = np.zeros((40, 50), dtype=np.uint8)
    mask[4:12, 4:20] = 1
    mask[24:30, 20:30] = 3
    prob = np.eye(5)[mask].transpose(2, 0, 1).astype(np.float32)
    rgb = np.random.default_rng(7).integers(10, 240, (40, 50, 3), dtype=np.uint8)
    return prob, np.stack([mask, mask]), rgb


def test_good_review_is_not_motion_authority():
    evidence = review_evidence(*inputs())
    decision = apply_review_gate(
        evidence, "offset", {"thresholds": {"offset": 0.7}, "motion_permitted": True}
    )
    assert decision["accepted"]
    assert not decision["motion_permitted"]
    assert decision["insertion_pose"] == "unknown"


@pytest.mark.parametrize("failure", ["black", "white", "missing", "border", "disagreement", "nan"])
def test_bad_evidence_abstains(failure):
    prob, masks, rgb = inputs()
    if failure == "black":
        rgb[:] = 0
    elif failure == "white":
        rgb[:] = 255
    elif failure == "missing":
        prob[3] = 0
    elif failure == "border":
        prob[3, 0, 0] = 2
    elif failure == "disagreement":
        masks[1][masks[1] == 3] = 0
    else:
        prob[0, 0, 0] = np.nan
    decision = apply_review_gate(review_evidence(prob, masks, rgb), "offset", {"thresholds": {"offset": 0.7}})
    assert not decision["accepted"] and not decision["motion_permitted"]
    assert decision["reasons"]


@pytest.mark.parametrize("threshold", [None, -1, 1.1, float("nan"), float("inf"), "0.7"])
def test_unqualified_or_invalid_policy_abstains(threshold):
    decision = apply_review_gate(review_evidence(*inputs()), "offset", {"thresholds": {"offset": threshold}})
    assert not decision["accepted"]


def test_missing_eligibility_cannot_accept():
    assert not apply_review_gate({"score": 1}, "offset", {"thresholds": {"offset": 0.5}})["accepted"]


def test_zero_failures_does_not_mean_zero_risk():
    spec = importlib.util.spec_from_file_location(
        "reliability_eval", Path(__file__).parents[1] / "scripts/evaluate_zero_reliability.py"
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rows = [
        {
            "file": f"{i}.png",
            "camera_id": "offset",
            "evidence": {"eligible": True, "score": 1, "reasons": []},
            "good_component_pair": True,
        }
        for i in range(20)
    ]
    report = module.gate_summary(rows, {"thresholds": {"offset": 0.7}}, interval=True)
    assert report["bad_accepts"] == 0
    assert 0.16 < report["nominal_bad_accept_upper_97_5pct"] < 0.18
    rows[0]["good_component_pair"] = False
    assert module.gate_summary(rows, {"thresholds": {"offset": 0.7}})["bad_accepts"] == 1
    abstain = module.gate_summary(rows, {"thresholds": {"offset": None}}, interval=True)
    assert abstain["accepted"] == 0 and abstain["empirical_bad_accept_rate"] is None
    assert abstain["nominal_bad_accept_upper_97_5pct"] is None
