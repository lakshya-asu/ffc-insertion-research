import math

import pytest

from ffc.grasp_footprint import EndGeometry, rectangle_half_bounds, review


def test_lower_face_contacts_reject_nominal_stiffener_grasp():
    end = EndGeometry(11.5, 4, 6)
    result = review(end, 0, 5, (3, 1.5), 0, 0)
    assert not result["footprint_clear_under_assumptions"]
    assert result["contact_margin_mm"] == -0.5


def test_continuous_yaw_bound_includes_interior_extremum():
    x, y = rectangle_half_bounds(6, 3, 80)
    assert x == pytest.approx(math.hypot(3, 1.5))
    assert y == pytest.approx(math.hypot(3, 1.5))


def test_placement_error_turns_nominal_fit_into_rejection():
    end = EndGeometry(11.5, 4, 6)
    assert review(end, 0, 6.5, (3, 1.5), 0, 1)["footprint_clear_under_assumptions"]
    assert not review(end, 0, 6.5, (3, 1.5), 0.5, 1)["footprint_clear_under_assumptions"]


def test_body_pad_candidate_and_offcenter_cup():
    end = EndGeometry(11.5, 4, 6)
    result = review(end, 0, 10, rectangle_half_bounds(6, 3, 10), 0.5, 1)
    assert result["wholly_on_body"]
    assert not result["wholly_on_stiffener"]
    assert review(end, 0, 12, (3.5, 3.5), 0.5, 1)["wholly_on_body"]
    assert not review(end, 1, 12, (3.5, 3.5), 0.5, 1)["footprint_clear_under_assumptions"]


@pytest.mark.parametrize("error", [-1, float("nan"), float("inf")])
def test_invalid_uncertainty_rejected(error):
    with pytest.raises(ValueError):
        review(EndGeometry(11.5, 4, 6), 0, 10, (3, 1.5), error, 1)
