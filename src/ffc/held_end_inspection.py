"""RGB-only commissioning features, deliberately insufficient to authorize motion.

Blue terminal and yellow tool regions are appearance heuristics for the current
rendered materials. Neither identifies the physical leading edge or proves slip.
"""

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from ffc.camera_observations import CameraFrame


@dataclass(frozen=True)
class InspectionConfig:
    calibration_id: str
    min_pixels: int = 24
    min_component_share: float = 0.65

    def __post_init__(self):
        if (
            type(self.calibration_id) is not str
            or not self.calibration_id
            or type(self.min_pixels) is not int
            or self.min_pixels < 3
            or not 0 < self.min_component_share <= 1
        ):
            raise ValueError("Invalid inspection configuration")


def component(mask, config):
    labels, count = ndimage.label(mask)
    if not count:
        return None, np.zeros_like(mask)
    sizes = np.bincount(labels.ravel())[1:]
    chosen = int(np.argmax(sizes)) + 1
    region = labels == chosen
    if int(region.sum()) < config.min_pixels:
        return None, region
    y, x = np.nonzero(region)
    points = np.c_[x, y].astype(float)
    center = points.mean(axis=0)
    _, vectors = np.linalg.eigh(np.cov(points.T))
    basis = vectors[:, ::-1]
    local = (points - center) @ basis
    lo, hi = local.min(axis=0), local.max(axis=0)
    corners = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]]) @ basis.T + center
    touches_border = bool(region[0].any() or region[-1].any() or region[:, 0].any() or region[:, -1].any())
    share = float(region.sum() / max(mask.sum(), 1))
    return {
        "centroid_px": center.tolist(),
        "oriented_extent_px": corners.tolist(),
        "area_px": int(region.sum()),
        "component_share": share,
        "touches_border": touches_border,
        "ambiguous_components": share < config.min_component_share,
    }, region


def inspect(frame: CameraFrame, config: InspectionConfig):
    if frame.calibration_id != config.calibration_id:
        raise ValueError("Inspection calibration mismatch")
    image = np.frombuffer(frame.rgb, dtype=np.uint8).reshape(frame.height, frame.width, 3).astype(float) / 255
    red, green, blue = np.moveaxis(image, -1, 0)
    blue_mask = (blue > 0.16) & (blue > 1.18 * red) & (blue > 1.08 * green)
    yellow_mask = (red > 0.28) & (green > 0.22) & (blue < 0.78 * np.minimum(red, green))
    # Small appearance gaps only; never use a simulator mask to fill occlusion.
    blue_border = bool(
        blue_mask[0].any() or blue_mask[-1].any() or blue_mask[:, 0].any() or blue_mask[:, -1].any()
    )
    blue_mask = ndimage.binary_closing(blue_mask, iterations=1)
    terminal, terminal_mask = component(blue_mask, config)
    tool, tool_mask = component(yellow_mask, config)
    if terminal is not None:
        terminal["touches_border"] |= blue_border
    reasons = []
    for name, value in [("terminal", terminal), ("tool", tool)]:
        if value is None:
            reasons.append(name + "_not_resolved")
        elif value["ambiguous_components"] or value["touches_border"]:
            reasons.append(name + "_ambiguous_or_clipped")
    relative = (
        (np.asarray(terminal["centroid_px"]) - tool["centroid_px"]).tolist() if terminal and tool else None
    )
    result = {
        "schema": "ffc.held-end-review.v1",
        "sequence": frame.sequence,
        "acquisition_s": frame.timestamp_s,
        "calibration_id": frame.calibration_id,
        "terminal_candidate": terminal,
        "tool_candidate": tool,
        "relative_image_offset_px": relative,
        "reasons": reasons + ["leading_edge_unverified"],
        "leading_edge": None,
        "slip_state": "unverified",
        "motion_permitted": False,
        "scope": "Appearance-only component measurements. Extents are not grasp/insertion targets.",
    }
    return result, terminal_mask, tool_mask


def compare(previous, current):
    if previous["calibration_id"] != current["calibration_id"]:
        raise ValueError("Cannot compare different cameras/calibrations")
    if current["acquisition_s"] <= previous["acquisition_s"] or current["sequence"] <= previous["sequence"]:
        raise ValueError("Cannot compare stale/reordered observations")
    offsets = [previous["relative_image_offset_px"], current["relative_image_offset_px"]]
    resolved = all(r["reasons"] == ["leading_edge_unverified"] for r in [previous, current])
    return {
        "relative_image_shift_px": (np.asarray(offsets[1]) - offsets[0]).tolist()
        if resolved and all(x is not None for x in offsets)
        else None,
        "slip_state": "unverified",
        "motion_permitted": False,
        "reason": "Perspective, rotation, deformation and occlusion can also change this image measurement",
    }
