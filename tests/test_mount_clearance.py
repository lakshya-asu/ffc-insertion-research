import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from ffc.mount_clearance import box, separation


def test_separation_known_gap_touch_overlap():
    a = box("a", [0, 0, 0], [2, 2, 2])
    assert separation(a, box("b", [3, 0, 0], [2, 2, 2])) == pytest.approx(1)
    assert separation(a, box("b", [2, 0, 0], [2, 2, 2])) == pytest.approx(0)
    assert separation(a, box("b", [1, 0, 0], [2, 2, 2])) < 0


def test_separation_is_rigid_transform_invariant():
    a = box("a", [0, 0, 0], [2, 1, 1])
    b = box("b", [3, 0.1, 0], [1, 1, 1], Rotation.from_euler("z", 37, degrees=True).as_matrix())
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_euler("xyz", [0.4, -0.6, 0.3]).as_matrix()
    transform[:3, 3] = [3, -7, 2]
    assert separation(a.transformed(transform), b.transformed(transform)) == pytest.approx(separation(a, b))


def test_diagonal_projection_bound_never_claims_euclidean_distance():
    a = box("a", [0, 0, 0], [2, 2, 2])
    b = box("b", [3, 3, 0], [2, 2, 2])
    assert separation(a, b) == pytest.approx(1)
    assert separation(a, b) < np.sqrt(2)
