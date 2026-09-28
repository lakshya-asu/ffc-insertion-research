"""Sensor-only admission and completion checks for the frozen mounted model."""

import numpy as np
from scipy.spatial.transform import Rotation

from ffc_cell.cv_frontend import calibration_id
from ffc_cell.guard import FrameGuard
from ffc_cell.macro_contract import OPTICAL_FRAME, CameraCalibration, require
from ffc_cell.observation_contract import validate_observation


class InferenceGate:
    def __init__(self, camera):
        require(isinstance(camera, CameraCalibration), "Validated fixed camera contract required")
        self.camera = camera
        self.guard = FrameGuard()
        self.rig_seen = False
        self.rig_fault = False
        self.identity = calibration_id(2448, 2048, camera.k, camera.d, OPTICAL_FRAME)
        self.k = np.asarray(camera.k).reshape(3, 3).copy()
        self.k[:2] *= 0.5
        self.k[0, 2] += 3.75
        self.k[1, 2] -= 0.25

    def observe_transform(self, parent, child, translation, quaternion):
        if child != OPTICAL_FRAME:
            return
        try:
            require(parent == "world", "Camera TF must be directly expressed in world")
            q, t = np.asarray(quaternion, float), np.asarray(translation, float)
            require(q.shape == (4,) and t.shape == (3,), "Malformed camera TF")
            require(np.isfinite(q).all() and np.isfinite(t).all(), "Nonfinite camera TF")
            require(abs(np.linalg.norm(q) - 1) < 1e-6, "Invalid camera TF quaternion")
            pose = np.eye(4)
            pose[:3, :3] = Rotation.from_quat(q).as_matrix()
            pose[:3, 3] = t
            require(np.allclose(pose, self.camera.world_optical, rtol=0, atol=1e-6), "Camera rig TF changed")
            require(not self.rig_fault, "Camera rig fault latched; reviewed restart required")
            self.rig_seen = True
        except ValueError:
            self.rig_fault = True
            self.rig_seen = False
            raise

    def check(self, message, now_ns):
        require(self.rig_seen and not self.rig_fault, "Reviewed camera TF unavailable or invalid")
        stamp, identity = validate_observation(message)
        require(identity == self.identity, "Observation differs from frozen native calibration")
        require(
            np.allclose(np.asarray(message.camera_info.k).reshape(3, 3), self.k, rtol=0, atol=1e-6),
            "Processed intrinsics differ from frozen camera",
        )
        self.guard.check(stamp, now_ns, identity)
        return stamp, identity

    def complete(self, message, now_ns):
        # Recheck all admission conditions after model execution, before publishing.
        stamp, identity = self.check(message, now_ns)
        self.guard.accept(stamp, identity)
