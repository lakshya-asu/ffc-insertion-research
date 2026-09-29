"""Offline planar tooling review; never a runtime grasp authorization.

Coordinates are mm: x across the cable, y back from its leading edge.
The full terminal width is excluded where contacts occur on either face.
Bounds cover continuous yaw and independent x/y placement error intervals.
No seal, damage, bending, insertion or collision physics is represented.
"""

from dataclasses import dataclass
from math import atan2, cos, isfinite, pi, sin


@dataclass(frozen=True)
class EndGeometry:
    width_mm: float
    contact_length_mm: float
    stiffener_length_mm: float
    reviewed_length_mm: float = 30

    def __post_init__(self):
        values = (self.width_mm, self.contact_length_mm, self.stiffener_length_mm, self.reviewed_length_mm)
        if not all(isfinite(v) and v > 0 for v in values):
            raise ValueError("Geometry must be finite and positive")
        if not self.contact_length_mm <= self.stiffener_length_mm <= self.reviewed_length_mm:
            raise ValueError("Expected contact <= stiffener <= reviewed length")


def rectangle_half_bounds(width_mm, length_mm, yaw_error_deg):
    """Exact axis envelope over yaw in [-error, +error], not sampled angles."""
    if not all(isfinite(v) for v in (width_mm, length_mm, yaw_error_deg)):
        raise ValueError("Nonfinite footprint")
    if min(width_mm, length_mm) <= 0 or not 0 <= yaw_error_deg <= 90:
        raise ValueError("Positive dimensions and yaw error in [0, 90] required")
    angle = yaw_error_deg * pi / 180

    def extent(a, b):
        t = min(angle, atan2(b, a))
        return a * cos(t) + b * sin(t)

    return extent(width_mm / 2, length_mm / 2), extent(length_mm / 2, width_mm / 2)


def review(end, center_x_mm, setback_mm, half_bounds_mm, placement_error_mm, margin_mm):
    """Conservative footprint clearance on both faces under stated assumptions."""
    hx, hy = half_bounds_mm
    values = (center_x_mm, setback_mm, hx, hy, placement_error_mm, margin_mm)
    if not all(isfinite(v) for v in values) or min(hx, hy) <= 0:
        raise ValueError("Invalid footprint or center")
    if min(placement_error_mm, margin_mm) < 0:
        raise ValueError("Negative uncertainty or margin")
    x_extent = abs(center_x_mm) + hx + placement_error_mm
    near, far = setback_mm - hy - placement_error_mm, setback_mm + hy + placement_error_mm
    clearances = {
        "contact_margin_mm": near - end.contact_length_mm - margin_mm,
        "side_margin_mm": end.width_mm / 2 - x_extent - margin_mm,
        "reviewed_end_margin_mm": end.reviewed_length_mm - far - margin_mm,
    }
    clear = min(clearances.values()) >= -1e-9
    return {
        "offline_only": True,
        "footprint_clear_under_assumptions": clear,
        "bounds_y_mm": [near, far],
        "wholly_on_stiffener": clear and far <= end.stiffener_length_mm - margin_mm + 1e-9,
        "wholly_on_body": clear and near >= end.stiffener_length_mm + margin_mm - 1e-9,
        **clearances,
    }
