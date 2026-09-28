"""Classical CV before DINO. No ROS, simulator geometry, or model labels required."""

import hashlib
import json

import cv2
import numpy as np

from ffc_cell.macro_contract import NATIVE_SIZE
from ffc_cell.macro_contract import OUTPUT_SIZE as OUTPUT_SIZE
from ffc_cell.macro_contract import PREPROCESSING_REVISION as REVISION  # noqa: F401 — public API


def calibration_id(width, height, k, d, frame):
    payload = dict(
        width=int(width), height=int(height), k=list(map(float, k)), d=list(map(float, d)), frame=frame
    )
    return hashlib.sha256(json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


class Frontend:
    def __init__(self):
        self.maps = {}

    def process(self, rgb, k, d, distortion_model="plumb_bob"):
        if rgb.dtype != np.uint8 or rgb.shape != (2048, 2448, 3):
            raise ValueError("Expected native 2448x2048 RGB8")
        matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(d, dtype=np.float64)
        if not np.isfinite(matrix).all() or not np.isfinite(distortion).all():
            raise ValueError("Nonfinite calibration")
        if matrix[0, 0] <= 0 or matrix[1, 1] <= 0 or not np.allclose(matrix[2], [0, 0, 1]):
            raise ValueError("Uncalibrated or invalid intrinsics")
        if distortion_model != "plumb_bob" or len(distortion) != 5:
            raise ValueError("Require explicit five-coefficient plumb_bob calibration")
        if np.all(distortion == 0):
            rectified = rgb
        else:
            key = (tuple(matrix.ravel()), tuple(distortion))
            if key not in self.maps:
                self.maps[key] = cv2.initUndistortRectifyMap(
                    matrix, distortion, np.eye(3), matrix, NATIVE_SIZE, cv2.CV_32FC1
                )
                if len(self.maps) > 4:
                    self.maps.pop(next(iter(self.maps)))
            rectified = cv2.remap(rgb, *self.maps[key], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # Area averaging at exactly 2:1; preserve the complete field, then pad for ViT/16.
        resized = cv2.resize(rectified, (1224, 1024), interpolation=cv2.INTER_AREA)
        window = cv2.copyMakeBorder(resized, 0, 0, 4, 4, cv2.BORDER_CONSTANT, value=0)
        roi_k = matrix.copy()
        roi_k[:2, :] *= 0.5
        # OpenCV resize maps pixel centres: (x + 0.5) * scale - 0.5.
        roi_k[0, 2] += 4 - 0.25
        roi_k[1, 2] -= 0.25
        gray = cv2.cvtColor(window, cv2.COLOR_RGB2GRAY)
        edges = cv2.Canny(gray, 50, 150)
        low = float((gray <= 1).mean())
        high = float((gray >= 254).mean())
        metrics = dict(
            mean_luma=float(gray.mean()),
            std_luma=float(gray.std()),
            low_clipped_fraction=low,
            high_clipped_fraction=high,
            laplacian_variance=float(cv2.Laplacian(gray, cv2.CV_64F).var()),
            edge_fraction=float((edges > 0).mean()),
        )
        if low > 0.98 or high > 0.98 or metrics["std_luma"] < 1:
            raise ValueError("Blank or saturated camera window")
        # Sharpness is diagnostic only until task-specific thresholds are calibrated.
        return window, roi_k, edges, metrics
