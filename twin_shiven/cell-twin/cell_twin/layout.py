"""Cell geometry taken from the NUC's recordings (twin_layout) and station.yaml.

World frame: table top is z = 0, x runs from the left arm to the right arm,
z is up. Both arm bases sit on 45 mm plates at x = +-0.6192 m; the right arm
is yawed 180 deg so the two arms face each other. Values are the ones every
recording carries in its twin_layout block (revision 29).
"""
from __future__ import annotations

import numpy as np

TABLE_W, TABLE_D, TABLE_H = 1.8, 1.0, 0.75          # metres, from twin_layout.table
ARM_BASE = {                                        # xyz metres, yaw degrees
    "left": (np.array([-0.6192, 0.0, 0.045]), 0.0),
    "right": (np.array([0.6199, -0.0688, 0.0895]), 178.4),
}
# Registered 2026-10-07 (scripts/register_cell.py): the right base pose that makes the 16 seated demos' tool axes
# collinear (3.2 mm, 1.9 deg) AND both wrist frames match (7.5 px). Roll/pitch are what the recorded joint angles
# imply about the real base; they stand in for base tilt plus any joint-zero offsets of the real right arm.
ARM_BASE_RP = {"left": (0.0, 0.0), "right": (19.93, 4.05)}   # roll, pitch degrees, applied before the yaw
FPS = 30
# Normalised gripper command (0 closed .. 1 open) to Menagerie finger slide (m per finger).
FINGER_SLIDE_RANGE = (-0.005, 0.0475)
JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# Gripper mount roll about the tool axis, applied as a joint-6 offset when recorded joints go into the model
# (2026-10-07): with the recorded j6 alone the twin's pinch axes sit 28 deg (left) and 55 deg (right) off vertical,
# while every real top-view frame shows both finger faces square to the camera (pinch axes vertical). The offsets are
# constant across all 16 preinsertion2 demos (left 28.0 +- 0.1 deg). They stand for how the grippers are bolted
# to the flanges relative to the Menagerie model, not for an encoder fault.
import os as _os
_j6 = _os.environ.get("TWIN_J6_OFFSETS", "0,-27")
JOINT6_OFFSET_DEG = {"left": float(_j6.split(",")[0]), "right": float(_j6.split(",")[1])}


def j6_offset(arm: str) -> float:
    return float(np.radians(JOINT6_OFFSET_DEG[arm]))


def model_joints(state14):
    """Recorded 14-D state -> the same vector with the gripper mount roll folded into joint 6 of each arm."""
    q = np.array(state14, dtype=float).copy()
    q[5] += j6_offset("left"); q[12] += j6_offset("right")
    return q


def base_rotation(arm: str) -> np.ndarray:
    _, yaw = ARM_BASE[arm]
    r, p = np.radians(ARM_BASE_RP.get(arm, (0.0, 0.0)))
    y = np.radians(yaw)
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def base_transform(arm: str) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = base_rotation(arm)
    T[:3, 3] = ARM_BASE[arm][0]
    return T


def grip_to_slide(g: float) -> float:
    lo, hi = FINGER_SLIDE_RANGE
    return lo + float(np.clip(g, 0.0, 1.0)) * (hi - lo)


def slide_to_grip(s: float) -> float:
    lo, hi = FINGER_SLIDE_RANGE
    return float(np.clip((s - lo) / (hi - lo), 0.0, 1.0))
