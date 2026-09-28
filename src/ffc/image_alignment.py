"""Image-only line geometry. No scene poses, depth, physical clearance or controls."""

import numpy as np
from scipy.ndimage import label

from ffc.macro_contract import OUTPUT_SIZE, require


def edge_line(mask, class_id, side, config):
    binary = mask == class_id
    total = int(binary.sum())
    require(total >= config["min_pixels"], "insufficient visible pixels")
    components, count = label(binary)
    sizes = np.bincount(components.ravel())[1:]
    selected = int(sizes.argmax()) + 1
    require(float(sizes[selected - 1]) / total >= config["min_component_fraction"], "fragmented feature")
    y, x = np.where(components == selected)
    require(
        x.min() > 4 and x.max() < mask.shape[1] - 5 and y.min() > 0 and y.max() < mask.shape[0] - 1,
        "feature touches image boundary",
    )
    low, high = int(x.min()), int(x.max())
    require(high - low >= config["min_span_px"], "insufficient horizontal span")
    columns = np.unique(x)
    require(len(columns) / (high - low + 1) >= config["min_column_coverage"], "missing edge columns")
    boundary = np.full(mask.shape[1], -np.inf if side == "max" else np.inf)
    (np.maximum.at if side == "max" else np.minimum.at)(boundary, x, y)
    trim = config["trim_fraction"] * (high - low)
    use = columns[(columns >= low + trim) & (columns <= high - trim)]
    points = np.column_stack([use, boundary[use]])
    inliers = np.ones(len(points), bool)
    for _ in range(3):
        slope, intercept = np.polyfit(points[inliers, 0], points[inliers, 1], 1)
        residual = points[:, 1] - (slope * points[:, 0] + intercept)
        mad = float(np.median(np.abs(residual - np.median(residual))))
        inliers = np.abs(residual) <= max(2.0, 3 * 1.4826 * mad)
        require(inliers.sum() >= 0.8 * len(points), "unstable line support")
    p95 = float(np.percentile(np.abs(residual), 95) / np.sqrt(1 + slope * slope))
    require(p95 <= config["max_residual_p95_px"], "edge is curved, occluded or irregular")
    return dict(
        slope=float(slope),
        intercept=float(intercept),
        x_support=[low, high],
        residual_p95_px=p95,
        visible_pixels=total,
        inlier_fraction=float(inliers.mean()),
        angle_deg=float(np.degrees(np.arctan(slope))),
    )


def estimate_alignment(mask, config):
    require(isinstance(mask, np.ndarray) and mask.dtype == np.uint8, "Expected uint8 class mask")
    require(mask.shape == OUTPUT_SIZE[::-1] and int(mask.max()) < 6, "Wrong mask shape or class IDs")
    result = dict(
        revision=config["revision"],
        status="abstain",
        features={},
        reasons=[],
        measurement=None,
        calibrated_uncertainty=None,
        motion_permitted=False,
    )
    for name, c, side in [("upper_rim", 1, "max"), ("lower_rim", 2, "min"), ("cable_edge", 3, "min")]:
        try:
            result["features"][name] = edge_line(mask, c, side, config)
        except ValueError as exc:
            result["reasons"].append(name + ": " + str(exc))
    if result["reasons"]:
        return result
    upper, lower, cable = (result["features"][n] for n in ["upper_rim", "lower_rim", "cable_edge"])
    try:
        require(
            abs(upper["angle_deg"] - lower["angle_deg"]) <= config["max_rim_angle_difference_deg"],
            "entrance rims disagree in angle",
        )
        low = max(f["x_support"][0] for f in [upper, lower, cable])
        high = min(f["x_support"][1] for f in [upper, lower, cable])
        require(high - low >= config["min_span_px"], "insufficient common support")
        at = np.array([low, (low + high) / 2, high])
        gap = (lower["slope"] - upper["slope"]) * at + lower["intercept"] - upper["intercept"]
        require(
            np.all((gap >= config["min_gap_px"]) & (gap <= config["max_gap_px"])),
            "rim ordering or apparent gap invalid",
        )
        slope = (upper["slope"] + lower["slope"]) / 2
        intercept = (upper["intercept"] + lower["intercept"]) / 2
        x = float(at[1])
        delta = (cable["slope"] * x + cable["intercept"] - (slope * x + intercept)) / np.sqrt(
            1 + slope * slope
        )
        result["measurement"] = dict(
            reference_x_px=x,
            common_x_support=[low, high],
            projected_normal_separation_px=float(delta),
            relative_image_angle_deg=cable["angle_deg"] - float(np.degrees(np.arctan(slope))),
            apparent_rim_gap_px=float(gap[1]),
        )
        result["status"] = "image_measurement"
    except ValueError as exc:
        result["reasons"].append(str(exc))
    return result
