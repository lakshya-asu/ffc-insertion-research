import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ffc.bending_fit import rotated_offset, select_gain


def test_float32_small_angle_survives_scalar_rounding():
    q = np.array([1, 0, 0.00022779005, 0], dtype=np.float32)
    v = np.array([0.001, 0, 0])
    expected = Rotation.from_quat(q, scalar_first=True).apply(v)
    np.testing.assert_allclose(rotated_offset(q, v), expected, atol=1e-16)
    assert rotated_offset(q, v)[2] == pytest.approx(-4.5558e-7, rel=1e-5)


def test_rotation_sign_scale_and_general_orientation():
    rng = np.random.default_rng(41)
    for _ in range(100):
        q, v = rng.normal(size=4), rng.normal(size=3)
        expected = Rotation.from_quat(q, scalar_first=True).apply(v)
        for scale in [1, -1, 1e200, 1e-200]:
            np.testing.assert_allclose(rotated_offset(q * scale, v), expected, atol=2e-15)


@pytest.mark.parametrize("q", [[0, 0, 0, 0], [1, 0, float("nan"), 0], [1, 0, 0]])
def test_invalid_rotation_rejected(q):
    with pytest.raises(ValueError):
        rotated_offset(q, [1, 0, 0])


def report():
    return {
        "settings": {"split": "fit"},
        "cases": [
            {
                "split": "fit",
                "gain": gain,
                "length_m": span,
                "modulus_multiplier": 1,
                "uniform_load_multiplier": 1,
                "reference": {"nonlinear_hinge_chain_prediction_m": span / 100},
                "observed_sag_m": span / 100 * ratio,
                "passed": abs(ratio - 1) <= 0.05,
            }
            for gain, ratio in [(1, 1.5), (2, 1.02)]
            for span in [0.024, 0.032]
        ],
    }


def test_selects_using_all_fit_cases():
    result = select_gain(report())
    assert result["selected_gain"] == 2
    assert result["fit_passed"]


def test_validation_and_unequal_cases_rejected():
    data = report()
    data["settings"]["split"] = "validation"
    with pytest.raises(ValueError):
        select_gain(data)
    data = report()
    data["cases"].pop()
    with pytest.raises(ValueError):
        select_gain(data)
