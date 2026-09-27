"""Independent analytical checks on the transform and bounded-IK conventions."""

import numpy as np
from scipy.spatial.transform import Rotation

from ffc.isaac_kinematics import Chain


def planar_chain():
    p0, p1 = np.eye(4), np.eye(4)
    p1[0, 3] = 0.3
    return Chain(
        np.eye(4),
        [p0, p1],
        [np.eye(4), np.eye(4)],
        [np.array([0.0, 0.0, 1.0])] * 2,
        np.array([-2.0, -2.0]),
        np.array([2.0, 2.0]),
    )


def test_forward_matches_planar_analytical_solution():
    chain = planar_chain()
    a, b = 0.7, -0.4
    matrix = chain.forward(np.array([a, b]))
    endpoint = matrix[:3, 3] + matrix[:3, :3] @ np.array([0.2, 0.0, 0.0])
    expected = [0.3 * np.cos(a) + 0.2 * np.cos(a + b), 0.3 * np.sin(a) + 0.2 * np.sin(a + b), 0.0]
    np.testing.assert_allclose(endpoint, expected, atol=1e-12)
    np.testing.assert_allclose(matrix[:3, :3], Rotation.from_euler("z", a + b).as_matrix(), atol=1e-12)


def test_inverse_matches_reachable_analytical_pose():
    chain = planar_chain()
    a, b = 0.7, -0.4
    target = np.array([0.3 * np.cos(a) + 0.2 * np.cos(a + b), 0.3 * np.sin(a) + 0.2 * np.sin(a + b), 0.0])
    q, report = chain.solve(
        target,
        Rotation.from_euler("z", a + b).as_matrix(),
        np.zeros(2),
        np.array([0.2, 0.0, 0.0]),
        np.random.default_rng(1),
    )
    np.testing.assert_allclose(q, [a, b], atol=1e-6)
    assert report["position_error_m"] < 1e-7
