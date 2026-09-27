import numpy as np
from scipy.spatial.transform import Rotation

from ffc.seating import seating_metrics

CABLE = {"width_m": 0.008, "thickness_m": 0.0003}
CONNECTOR = {
    "mouth_xyz_m": [0, 0, 0],
    "slot_width_m": 0.0084,
    "slot_height_m": 0.0005,
    "insertion_depth_m": 0.004,
}


def test_aligned_tip_seats_with_positive_clearance():
    assert seating_metrics([0.0038, 0, 0], np.eye(3), CABLE, CONNECTOR)["success"]


def test_centered_tip_with_excess_roll_is_not_seated():
    rotation = Rotation.from_euler("x", 5, degrees=True).as_matrix()
    result = seating_metrics([0.0038, 0, 0], rotation, CABLE, CONNECTOR)
    assert not result["success"]
    assert not result["tip_face_inside_slot"]


def test_tip_beyond_connector_backstop_is_not_seated():
    assert not seating_metrics([0.005, 0, 0], np.eye(3), CABLE, CONNECTOR)["success"]


def test_partially_inserted_tip_is_not_seated():
    result = seating_metrics([0.0032, 0, 0], np.eye(3), CABLE, CONNECTOR)
    assert result["tip_face_inside_slot"]
    assert not result["all_tip_corners_seated"]
    assert not result["success"]
