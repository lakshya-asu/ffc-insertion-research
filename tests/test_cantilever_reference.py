"""Independent one-link torque balance checks the nonlinear static reference."""

import numpy as np
from scipy.optimize import brentq

from ffc.cantilever_reference import hinge_chain_reference


def test_single_free_segment_matches_exact_torque_balance():
    cable = {
        "segments": 2,
        "length_m": 0.10,
        "mass_kg": 0.01,
        "bend_stiffness_nm_per_degree": 0.002 * np.pi / 180,
        "stiffener_segments": 0,
    }
    length, mass, stiffness = 0.05, 0.005, 0.002
    angle = brentq(lambda theta: stiffness * theta - mass * 9.81 * length / 2 * np.cos(theta), 0, 0.61)
    result = hinge_chain_reference(cable)
    assert abs(result["nonlinear_hinge_chain_prediction_m"] - length * np.sin(angle)) < 1e-9
    assert result["nonlinear_hinge_chain_prediction_m"] < result["linear_hinge_chain_prediction_m"]
