import json
from pathlib import Path

import pytest

from ffc.cable_spec import load_spec, width_at

SPEC = Path(__file__).resolve().parents[1] / "config/cables/rpi-camera-standard-mini-200-rev2.json"


def test_revision_two_outline_and_assumptions_remain_explicit():
    spec = load_spec(SPEC)
    assert width_at(spec, 0) == pytest.approx(0.0115)
    assert width_at(spec, 0.2) == pytest.approx(0.016)
    assert width_at(spec, 0.1715) == pytest.approx(0.01375)
    assert spec["parameters"]["body_thickness_m"]["status"] == "design_assumption"
    assert not spec["qualification"]["learning_dataset_release"]


def test_missing_provenance_and_out_of_bounds_values_rejected(tmp_path):
    s = json.loads(SPEC.read_text())
    s["parameters"]["body_thickness_m"]["value"] = 0.01
    p = tmp_path / "spec.json"
    p.write_text(json.dumps(s))
    with pytest.raises(ValueError, match="range"):
        load_spec(p)
    s = json.loads(SPEC.read_text())
    s["parameters"]["mini_width_m"]["sources"] = []
    p.write_text(json.dumps(s))
    with pytest.raises(ValueError, match="source"):
        load_spec(p)


def test_outline_does_not_extrapolate():
    with pytest.raises(ValueError):
        width_at(load_spec(SPEC), 0.21)
