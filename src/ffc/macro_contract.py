"""Versioned macro artifact contracts; no Torch, ROS, USD or scene-state dependency.

The ROS image frontend and offline learning tools share these constants. Camera
calibration is fixed rig metadata, not an object pose inferred from the simulator.
"""

import hashlib
import json
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np

NATIVE_SIZE = (2448, 2048)  # width, height
OUTPUT_SIZE = (1232, 1024)
PREPROCESSING_REVISION = "macro-rectify-area-half-pad4-v1"
CAMERA_PROFILE = "macro-mount-elevation45-v1"
OPTICAL_FRAME = "macro_optical_frame"
CLASSES = (
    "background",
    "upper_entrance_rim",
    "lower_entrance_rim",
    "cable_leading_band",
    "slider_open",
    "slider_closed",
)
CALIBRATION_FIELDS = frozenset(
    {
        "width",
        "height",
        "k",
        "d",
        "distortion_model",
        "frame_id",
        "profile",
        "world_optical",
        "preprocessing_revision",
    }
)


class ContractError(ValueError):
    """An artifact is incompatible or malformed; callers must reject it."""


def require(condition, message):
    if not condition:
        raise ContractError(message)


@dataclass(frozen=True)
class CameraCalibration:
    """Immutable native camera contract. K is pixels; world_optical translation is m."""

    k: tuple[float, ...]
    d: tuple[float, ...]
    world_optical: tuple[tuple[float, ...], ...]

    @classmethod
    def from_mapping(cls, value):
        require(isinstance(value, Mapping), "Camera calibration must be an object")
        require(set(value) == CALIBRATION_FIELDS, "Missing or unexpected camera calibration fields")
        require(
            type(value["width"]) is int and type(value["height"]) is int, "Image dimensions must be integers"
        )
        require((value["width"], value["height"]) == NATIVE_SIZE, "Native camera dimensions changed")
        require(value["distortion_model"] == "plumb_bob", "Unsupported distortion model")
        require(value["frame_id"] == OPTICAL_FRAME, "Wrong optical coordinate frame")
        require(value["profile"] == CAMERA_PROFILE, "Unsupported camera profile")
        require(value["preprocessing_revision"] == PREPROCESSING_REVISION, "Preprocessing revision changed")
        try:
            k, d, pose = (np.asarray(value[key], dtype=float) for key in ("k", "d", "world_optical"))
        except (TypeError, ValueError) as exc:
            raise ContractError("Nonnumeric camera calibration") from exc
        require(
            k.shape == (9,) and d.shape == (5,) and pose.shape == (4, 4), "Invalid calibration array shape"
        )
        require(all(np.isfinite(a).all() for a in (k, d, pose)), "Nonfinite camera calibration")
        require(k[0] > 0 and k[4] > 0 and np.allclose(k[6:], [0, 0, 1]), "Invalid intrinsic matrix")
        require(np.allclose(pose[3], [0, 0, 0, 1]), "Invalid homogeneous calibration transform")
        rotation = pose[:3, :3]
        require(
            np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-6)
            and np.isclose(np.linalg.det(rotation), 1, atol=1e-6),
            "Camera rotation is not proper orthonormal",
        )
        return cls(tuple(k.tolist()), tuple(d.tolist()), tuple(tuple(row) for row in pose.tolist()))

    def as_mapping(self):
        return dict(
            width=NATIVE_SIZE[0],
            height=NATIVE_SIZE[1],
            k=list(self.k),
            d=list(self.d),
            distortion_model="plumb_bob",
            frame_id=OPTICAL_FRAME,
            profile=CAMERA_PROFILE,
            world_optical=[list(row) for row in self.world_optical],
            preprocessing_revision=PREPROCESSING_REVISION,
        )

    @property
    def fingerprint(self):
        """Includes fixed extrinsics; distinct from the ROS intrinsic-only calibration_id."""
        payload = json.dumps(self.as_mapping(), sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(payload.encode()).hexdigest()


def validate_checkpoint_metadata(checkpoint):
    require(isinstance(checkpoint, Mapping), "Checkpoint metadata must be an object")
    require(tuple(checkpoint.get("classes", ())) == CLASSES, "Checkpoint class IDs/order changed")
    require(
        checkpoint.get("preprocessing_revision") == PREPROCESSING_REVISION, "Checkpoint preprocessing changed"
    )
    require(tuple(checkpoint.get("input_size", ())) == OUTPUT_SIZE, "Checkpoint image size changed")
    require(checkpoint.get("camera_profile") == CAMERA_PROFILE, "Checkpoint camera profile changed")
    require(checkpoint.get("use_dino") is True, "Checkpoint requires the frozen DINOv3 encoder")


def verify_digest(actual, expected, name):
    require(
        isinstance(expected, str) and len(expected) == 64 and all(c in "0123456789abcdef" for c in expected),
        f"Invalid {name} SHA-256",
    )
    require(actual == expected, f"{name} SHA-256 mismatch")


def labels_to_model(native, calibration):
    """Offline only. Exact integer 2:1 nearest-exact tie rule and fixed side padding.

    OpenCV INTER_NEAREST_EXACT at this scale selects the top-left pixel of each
    2x2 cell. Validated against OpenCV; this is deliberately not PIL's tie rule.
    Nonzero distortion needs a separately implemented matching label rectifier.
    """
    require(isinstance(calibration, CameraCalibration), "Validated calibration required")
    require(not any(calibration.d), "Nonzero distortion requires matching label rectification")
    require(isinstance(native, np.ndarray) and native.dtype == np.uint8, "Labels must be uint8")
    require(native.shape == (NATIVE_SIZE[1], NATIVE_SIZE[0]), "Label dimensions differ from native RGB")
    require(int(native.max()) < len(CLASSES), "Unknown label class ID")
    return np.pad(native[::2, ::2], ((0, 0), (4, 4)), mode="constant")
