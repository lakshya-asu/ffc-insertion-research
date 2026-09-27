"""Geometric seating checks on the entire cable tip cross-section."""

from itertools import product

import numpy as np


def seating_metrics(tip_xyz, tip_rotation, cable, connector):
    offsets = np.array(
        [[0, y * cable["width_m"] / 2, z * cable["thickness_m"] / 2] for y, z in product((-1, 1), repeat=2)]
    )
    center = np.asarray(tip_xyz) - np.asarray(connector["mouth_xyz_m"])
    corners = offsets @ np.asarray(tip_rotation).T + center
    inside = bool(
        np.max(np.abs(corners[:, 1])) <= connector["slot_width_m"] / 2
        and np.max(np.abs(corners[:, 2])) <= connector["slot_height_m"] / 2
    )
    depth_ok = bool(
        corners[:, 0].min()
        > connector["insertion_depth_m"] - connector.get("seating_depth_tolerance_m", 0.0003)
        and corners[:, 0].max() <= connector["insertion_depth_m"] + 0.00005
    )
    return {
        "success": inside and depth_ok,
        "depth_m": float(center[0]),
        "lateral_error_m": float(center[1]),
        "height_error_m": float(center[2]),
        "tip_face_inside_slot": inside,
        "all_tip_corners_seated": depth_ok,
        "minimum_required_corner_depth_m": connector["insertion_depth_m"]
        - connector.get("seating_depth_tolerance_m", 0.0003),
        "tip_corners_relative_to_mouth_m": corners.tolist(),
        "electrical_continuity_verified": False,
        "latch_closed": False,
    }
