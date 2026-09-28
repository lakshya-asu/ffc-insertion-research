"""Validate the complete atomic ROS observation before any consumer trusts it.

Uses message attributes without importing ROS so malformed-envelope tests can run
in a CPU-only environment. This validates shape/provenance, not sensor authenticity.
"""

import re

import numpy as np

from ffc_cell.macro_contract import (
    CAMERA_PROFILE,
    OPTICAL_FRAME,
    OUTPUT_SIZE,
    PREPROCESSING_REVISION,
    ContractError,
    require,
)

QUALITY_NAMES = frozenset(
    {
        "mean_luma",
        "std_luma",
        "low_clipped_fraction",
        "high_clipped_fraction",
        "laplacian_variance",
        "edge_fraction",
    }
)


def validate_observation(message):
    try:
        return _validate(message)
    except (AttributeError, TypeError, IndexError) as exc:
        raise ContractError("Malformed observation envelope") from exc


def _validate(message):
    require(message.preprocessing_revision == PREPROCESSING_REVISION, "Observation preprocessing changed")
    require(message.source_profile == CAMERA_PROFILE, "Observation camera profile changed")
    require(re.fullmatch(r"[0-9a-f]{64}", message.calibration_id) is not None, "Invalid calibration identity")
    header = message.header
    require(header.frame_id == OPTICAL_FRAME, "Wrong observation optical frame")
    require(
        type(header.stamp.sec) is int
        and type(header.stamp.nanosec) is int
        and header.stamp.sec >= 0
        and 0 <= header.stamp.nanosec < 1_000_000_000,
        "Invalid acquisition stamp",
    )
    width, height = OUTPUT_SIZE
    for image, encoding, channels in [(message.image, "rgb8", 3), (message.edges, "mono8", 1)]:
        require(image.header == header, "Observation contains mixed acquisition headers")
        require((image.width, image.height) == OUTPUT_SIZE, "Wrong processed image dimensions")
        require(image.encoding == encoding and not image.is_bigendian, "Wrong processed image encoding")
        require(
            image.step == width * channels and len(image.data) == width * height * channels,
            "Truncated or padded processed image payload",
        )
    info = message.camera_info
    require(info.header == header, "Observation calibration timestamp/frame mismatch")
    require((info.width, info.height) == OUTPUT_SIZE, "Wrong processed calibration dimensions")
    require(
        info.distortion_model == "plumb_bob" and len(info.d) == 5 and not any(info.d),
        "Observation must be rectified with explicit zero distortion",
    )
    k, r, p = (np.asarray(getattr(info, key), dtype=float) for key in ["k", "r", "p"])
    require(k.shape == (9,) and r.shape == (9,) and p.shape == (12,), "Malformed processed calibration")
    require(all(np.isfinite(x).all() for x in [k, r, p]), "Nonfinite processed calibration")
    require(k[0] > 0 and k[4] > 0 and np.allclose(k[6:], [0, 0, 1]), "Invalid processed intrinsic matrix")
    require(np.allclose(r, np.eye(3).ravel()), "Unexpected rectification rotation")
    require(
        np.allclose(p.reshape(3, 4), np.column_stack([k.reshape(3, 3), np.zeros(3)])),
        "Projection matrix disagrees with transformed K",
    )
    require(
        info.binning_x in [0, 1]
        and info.binning_y in [0, 1]
        and not any([info.roi.width, info.roi.height, info.roi.x_offset, info.roi.y_offset]),
        "Unexpected processed ROI or binning",
    )
    names, values = message.quality_names, message.quality_values
    require(
        len(names) == len(values) == len(QUALITY_NAMES) and set(names) == QUALITY_NAMES,
        "Missing, duplicate or unexpected quality fields",
    )
    quality = dict(zip(names, values, strict=True))
    require(all(np.isfinite(v) and v >= 0 for v in values), "Invalid quality measurements")
    require(
        quality["mean_luma"] <= 255
        and all(quality[n] <= 1 for n in ["low_clipped_fraction", "high_clipped_fraction", "edge_fraction"]),
        "Out-of-range image quality",
    )
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec, message.calibration_id
