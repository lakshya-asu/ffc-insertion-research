"""Frozen RGB-only color/shape baseline, not a trained or deployable detector.

No simulator imports, scene paths, masks or true poses. This baseline intentionally
exposes reliance on the current yellow/blue cable appearance. It cannot infer
contact-face polarity, depth or physical grasp validity.
"""

import numpy as np
from scipy import ndimage


def color_features(rgb: np.ndarray) -> np.ndarray:
    r, g, b = np.moveaxis(rgb.astype(float) / 255.0, -1, 0)
    return np.stack([np.ones_like(r), r, g, b, r * r, g * g, b * b, r * g, r * b, g * b], axis=-1)


def detect_cable(rgb: np.ndarray, model: dict | None = None) -> tuple[dict, np.ndarray]:
    if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("Expected RGB8 image")
    r, g, b = np.moveaxis(rgb.astype(float), -1, 0)
    yellow = (r > b * 1.08) & (g > b * 1.08) & (r > g * 0.9) & (r > 70)
    blue = (b > r * 1.2) & (b > g * 1.1) & (b > 60)
    if model is None:
        candidates = yellow | blue
    else:
        coefficients = np.asarray(model["coefficients"], dtype=float)
        if coefficients.shape != (10,) or not np.isfinite(coefficients).all():
            raise ValueError("Invalid pixel classifier")
        candidates = color_features(rgb) @ coefficients > 0.0
    mask = ndimage.binary_closing(candidates, iterations=2)
    components, count = ndimage.label(mask)
    sizes = np.bincount(components.ravel())
    sizes[0] = 0
    empty = np.zeros(rgb.shape[:2], dtype=bool)
    if not count or sizes.max() < 80:
        return {"detected": False, "reason": "insufficient_color_support"}, empty
    supported = []
    for component_id in np.flatnonzero(sizes >= 80):
        selected = components == component_id
        points = np.argwhere(selected)[:, ::-1].astype(float)
        eigenvalues, eigenvectors = np.linalg.eigh(np.cov(points.T))
        elongation = float(np.sqrt(eigenvalues[1] / max(eigenvalues[0], 1e-9)))
        if elongation < (8 if model is not None else 4):
            continue
        if model is None and (selected & yellow).sum() < 0.25 * selected.sum():
            continue
        supported.append((len(points), selected, points, eigenvectors, elongation))
    if not supported:
        return {"detected": False, "reason": "not_a_supported_ribbon_shape"}, empty
    supported.sort(key=lambda item: item[0], reverse=True)
    if len(supported) > 1 and supported[1][0] >= 0.25 * supported[0][0]:
        return {"detected": False, "reason": "ambiguous_multiple_ribbons"}, empty
    _, selected, points, eigenvectors, elongation = supported[0]
    center = points.mean(axis=0)
    axis = eigenvectors[:, 1]
    projected = (points - center) @ axis
    ends = [center + axis * projected.min(), center + axis * projected.max()]
    result = {
        "detected": True,
        "reason": "learned_color_shape_candidate" if model else "color_shape_candidate",
        "center_uv": center.tolist(),
        "unordered_endpoints_uv": (
            None
            if (points.min(axis=0) <= 2).any()
            or (points.max(axis=0) >= [rgb.shape[1] - 3, rgb.shape[0] - 3]).any()
            else np.asarray(ends).tolist()
        ),
        "partial_view": bool(
            (points.min(axis=0) <= 2).any()
            or (points.max(axis=0) >= [rgb.shape[1] - 3, rgb.shape[0] - 3]).any()
        ),
        "pixel_area": int(selected.sum()),
        "elongation": elongation,
        "contact_face": "unknown",
        "insertion_endpoint": "unknown",
        "motion_permitted": False,
    }
    return result, selected
