"""Regression for loss of jaw preload during the suction-to-pinch transition."""

import numpy as np

from ffc.control_reference import retain_unchanged_targets


def test_hold_keeps_preload_through_every_interpolation_fraction():
    actual = np.array([0.2, -0.00616])
    previous = np.array([0.3, -0.0068])
    goal = np.array([0.4, -0.0068])
    start = retain_unchanged_targets(actual, previous, goal, [1])
    for fraction in np.linspace(0, 1, 101):
        commanded = start + fraction * (goal - start)
        assert np.isclose(600 * (actual[1] - commanded[1]), 0.384)
    assert start[0] == actual[0]


def test_opening_command_starts_from_actual_jaw_position():
    actual = np.array([-0.00616])
    start = retain_unchanged_targets(actual, [-0.0068], [0.0], [0])
    assert start[0] == actual[0]
