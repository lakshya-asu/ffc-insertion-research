"""Explicit physical lens units for ideal, fixed-focus reference camera geometry."""

import math

from pxr import UsdGeom


def apply_reference_optics(stage, camera, profile, focus_distance_m, resolution=None):
    native_w, native_h = profile["native_resolution"]
    width, height = resolution or (native_w, native_h)
    aperture_x, aperture_y = profile["active_crop_mm"]
    focal = profile["focal_length_mm"]
    if not math.isclose(width / height, aperture_x / aperture_y, rel_tol=1e-6):
        raise ValueError("Resolution aspect must preserve the physical sensor crop")
    if focus_distance_m <= 0 or focal <= 0:
        raise ValueError("Invalid camera distance or focal length")
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    mm_to_tenth_scene_unit = 0.01 / meters_per_unit
    camera.CreateFocalLengthAttr(focal * mm_to_tenth_scene_unit)
    camera.CreateHorizontalApertureAttr(aperture_x * mm_to_tenth_scene_unit)
    camera.CreateVerticalApertureAttr(aperture_y * mm_to_tenth_scene_unit)
    camera.CreateFocusDistanceAttr(focus_distance_m / meters_per_unit)
    # This framing reference does not claim a calibrated near-focus or blur model.
    camera.CreateFStopAttr(0)
    fx, fy = focal / aperture_x * width, focal / aperture_y * height
    fov_mm = focus_distance_m * 1000 * aperture_x / focal
    return {
        "model": profile["model"],
        "focal_length_mm": focal,
        "image_size": [width, height],
        "K": [[fx, 0, width / 2], [0, fy, height / 2], [0, 0, 1]],
        "horizontal_fov_mm": fov_mm,
        "normal_plane_pixels_per_mm": width / fov_mm,
        "focus_distance_m": focus_distance_m,
        "focus_mode": "fixed manual setting; physical focus reach unverified",
        "depth_of_field_enabled": False,
        "horizontal_fov_degrees": math.degrees(2 * math.atan(aperture_x / (2 * focal))),
        "usd_mm_scale": mm_to_tenth_scene_unit,
    }
