import numpy as np
import pytest

from ffc.joint_trajectory import reference, validate_move


def test_quintic_endpoints_monotonic_and_speed_bound():
    start = np.zeros(7)
    target = np.arange(7) * 0.001
    validate_move(start, target, [-1] * 7, [1] * 7, 2, 0.01, 0.01, 0.02)
    values = [reference(start, target, t, 2) for t in np.linspace(0, 2, 201)]
    assert np.array_equal(values[0][0], start)
    assert np.allclose(values[-1][0], target)
    assert np.allclose(values[0][1], 0) and np.allclose(values[-1][1], 0)
    assert np.min(np.diff([q for q, v in values], axis=0)) >= -1e-12
    assert max(np.max(abs(v)) for q, v in values) <= 0.01


@pytest.mark.parametrize(
    "target,duration,message",
    [
        ([np.nan] * 7, 2, "Invalid seven"),
        ([2] * 7, 2, "Joint bounds"),
        ([0.02] * 7, 2, "step"),
        ([0.01] * 7, 0.1, "Velocity"),
        ([0.01] * 7, 1, "Acceleration"),
    ],
)
def test_rejects_unbounded_trajectories(target, duration, message):
    with pytest.raises(ValueError, match=message):
        validate_move([0] * 7, target, [-1] * 7, [1] * 7, duration, 0.01, 0.02, 0.02)


@pytest.mark.parametrize("boundary,measured", [(0.0, -0.00002), (-0.02, -0.01998)])
def test_integrator_does_not_accumulate_at_a_travel_limit(boundary, measured):
    from ffc.joint_trajectory import bounded_integral

    offset = np.zeros(1)
    for _ in range(5000):
        offset = bounded_integral([boundary], [measured], offset, [-0.02], [0.0], 0.001, 0.005)
    assert offset[0] == 0
    # A subsequent inward command must not be delayed by accumulated saturation.
    target = -0.001 if boundary == 0 else -0.019
    offset = bounded_integral([target], [measured], offset, [-0.02], [0.0], 0.001, 0.005)
    assert np.sign(offset[0]) == np.sign(target - measured)


def test_integrator_unwinds_and_retains_unsaturated_gravity_correction():
    from ffc.joint_trajectory import bounded_integral

    offset = bounded_integral(
        [0.0, 0.5], [0.01, 0.49], np.array([0.001, 0.0]), [-1.0, -1.0], [0.0, 1.0], 0.01, 0.1
    )
    assert offset[0] == pytest.approx(0.0)
    assert offset[1] == pytest.approx(0.001)
