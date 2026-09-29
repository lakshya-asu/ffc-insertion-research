"""Full-profile properties must preserve mass and series bending compliance."""

import math
from pathlib import Path

import pytest

from ffc.cable_spec import load_spec
from ffc.profile_cable import joint_stiffness_nm_per_degree, sections


def profile():
    return load_spec(
        Path(__file__).resolve().parents[1] / "config/cables/rpi-camera-standard-mini-200-rev2.json"
    )


def test_total_mass_converges_to_integrated_piecewise_profile():
    # Exact integral of the declared piecewise-linear width and two reinforcements.
    area = 0.0115 * 0.170 + (0.0115 + 0.016) / 2 * 0.003 + 0.016 * 0.027
    expected_mass = 1800 * (area * 0.00014 + (0.0115 + 0.016) * 0.006 * (0.0003 - 0.00014))
    coarse, fine = sections(profile(), 100), sections(profile(), 200)
    coarse_mass = sum(s["mass_kg"] for s in coarse)
    fine_mass = sum(s["mass_kg"] for s in fine)
    assert fine_mass == pytest.approx(expected_mass, rel=1e-12)
    assert coarse_mass == pytest.approx(expected_mass, rel=0.001)
    assert abs(fine_mass - expected_mass) < abs(coarse_mass - expected_mass)
    for mesh in [coarse, fine]:
        assert sum(s["length_m"] for s in mesh) == pytest.approx(0.2)
        assert sum(s["length_m"] for s in mesh if s["reinforced"]) == pytest.approx(0.012)
        assert all(s["ei_nm2"] > 0 and s["mass_kg"] > 0 for s in mesh)
        assert all(not s["reinforced"] for s in mesh if 0.009 <= s["center_m"] <= 0.012)


def test_compliance_is_symmetric_and_limited_by_soft_section():
    soft = {"length_m": 0.002, "ei_nm2": 1e-5}
    stiff = {"length_m": 0.002, "ei_nm2": 1000 * 1e-5}
    uniform = joint_stiffness_nm_per_degree(soft, soft)
    assert uniform == pytest.approx(1e-5 / 0.002 * math.pi / 180)
    mixed = joint_stiffness_nm_per_degree(soft, stiff)
    assert mixed == joint_stiffness_nm_per_degree(stiff, soft)
    assert uniform < mixed < 2 * uniform


def test_unreviewed_segmentation_rejected():
    with pytest.raises(ValueError):
        sections(profile(), 10)
