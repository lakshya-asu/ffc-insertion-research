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
