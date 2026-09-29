"""Observable seating detector: decides "seated" from the packet alone.

Three conditions, all required:
  1. the tool has travelled by about the seat depth since the estimated tip crossed the mouth (commanded x),
     not counting travel during ticks the jaw pad flagged as slipping (the cable did not follow the tool)
  2. the wrench shows the stall against the backstop (axial force above a threshold)
  3. the estimated tip pose is inside the slot band and near the seat depth
Its false-positive and false-negative rates are measured offline against truth by scripts/grade_detector.py.
"""
from __future__ import annotations

import numpy as np

from .spec import DEFAULT, Spec

TRAVEL_TOL = 0.3e-3
STALL_FORCE = 0.12
EST_DEPTH_TOL = 0.2e-3


def seated_from_packet(obs: dict, mouth_cross_x: float | None, ctrl_x: float, spec: Spec = DEFAULT, slip_travel: float = 0.0) -> bool:
    k = spec.connector
    est = obs["tip_estimate"]
    w = obs["fixture_wrench_n_nm"]
    if mouth_cross_x is None:
        return False
    travelled = ctrl_x - mouth_cross_x - slip_travel
    cond_travel = travelled >= k.seat_depth.value - TRAVEL_TOL
    cond_stall = abs(w[0]) > STALL_FORCE or float(np.linalg.norm(w[:3])) > STALL_FORCE
    cond_est = (est[0] - slip_travel > k.seat_depth.value - EST_DEPTH_TOL) and abs(est[1]) < k.opening_width.value / 2 and abs(est[2]) < 0.4e-3
    return bool(cond_travel and cond_stall and cond_est)
