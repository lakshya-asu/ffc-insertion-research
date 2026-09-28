"""Offline mounting-envelope design checks. Never used by the runtime actor."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Box:
    name: str
    center: np.ndarray
    half: np.ndarray
    axes: np.ndarray

    def transformed(self, matrix):
        return Box(
            self.name, matrix[:3, :3] @ self.center + matrix[:3, 3], self.half, matrix[:3, :3] @ self.axes
        )

    def json(self):
        return dict(
            name=self.name, center=self.center.tolist(), half=self.half.tolist(), axes=self.axes.tolist()
        )


def box(name, center, size, axes=None):
    return Box(
        name,
        np.asarray(center, float),
        np.asarray(size, float) / 2,
        np.eye(3) if axes is None else np.asarray(axes, float),
    )


def separation(a, b):
    """Maximum separating-axis gap: a lower bound on clearance, NOT Euclidean distance.

    Positive means disjoint. Negative means OBB overlap, not mesh penetration.
    """
    axes = list(a.axes.T) + list(b.axes.T)
    axes.extend(np.cross(x, y) for x in a.axes.T for y in b.axes.T)
    axes = np.array([v / np.linalg.norm(v) for v in axes if np.linalg.norm(v) > 1e-9])
    ra = np.abs(axes @ a.axes) @ a.half
    rb = np.abs(axes @ b.axes) @ b.half
    return float(np.max(np.abs(axes @ (b.center - a.center)) - ra - rb))


def link_poses(chain, q):
    t, result = chain.base.copy(), []
    for angle, p, c, axis in zip(q, chain.parent_frames, chain.child_frames, chain.axes, strict=True):
        rot = np.eye(4)
        rot[:3, :3] = Rotation.from_rotvec(angle * axis).as_matrix()
        t = t @ p @ rot @ np.linalg.inv(c)
        result.append(t.copy())
    return result


def hardware(profile, mouth, post_side=-1):
    from ffc.macro_camera import optical_budget

    pose = profile["placement"]
    yaw, el = np.deg2rad([pose["yaw_degrees"], pose["elevation_degrees"]])
    axis = np.array([np.cos(el) * np.cos(yaw), np.cos(el) * np.sin(yaw), np.sin(el)])
    right = np.cross([0.0, 0.0, 1.0], axis)
    right /= np.linalg.norm(right)
    up = np.cross(axis, right)
    axes = np.column_stack([right, up, axis])
    target = mouth + np.asarray(pose["target_offset_from_mouth_mm"]) / 1000
    front = target + axis * 0.1
    lens = box("lens_keepout", front + axis * 0.0245, [0.043, 0.043, 0.049], axes)
    body = box("camera_body", front + axis * (0.049 + 0.0481 / 2), [0.029, 0.029, 0.0481], axes)
    adapter_center = body.center - up * 0.0185
    adapter = box("mount_adapter_envelope", adapter_center, [0.029, 0.008, 0.051], axes)
    saddle = box("locking_tilt_saddle", adapter_center - up * 0.014, [0.055, 0.020, 0.060], axes)
    # Short side arm meets the saddle and a foot bolted to the shared plate.
    arm_z = saddle.center[2] - 0.020
    post_xy = np.array([saddle.center[0], mouth[1] + post_side * 0.135])
    arm = box(
        "cross_arm",
        [post_xy[0], (post_xy[1] + saddle.center[1]) / 2, arm_z],
        [0.030, abs(post_xy[1] - saddle.center[1]) + 0.03, 0.030],
    )
    column = box("upright", [*post_xy, arm_z / 2], [0.030, 0.030, arm_z])
    foot = box("bolted_foot", [*post_xy, 0.004], [0.070, 0.070, 0.008])
    # Visible diagonal brace in the vertical plane of the side arm.
    start = np.array([*post_xy, 0.025])
    end = np.array([post_xy[0], (post_xy[1] + saddle.center[1]) / 2, arm_z])
    z = (end - start) / np.linalg.norm(end - start)
    x = np.array([1.0, 0.0, 0.0])
    y = np.cross(z, x)
    brace = box(
        "brace", (start + end) / 2, [0.012, 0.012, np.linalg.norm(end - start)], np.column_stack([x, y, z])
    )
    # A conservative volume for connector, service loop and strain relief; no bend-radius claim.
    usb = box(
        "usb_service_loop_keepout",
        front + axis * 0.115 + right * post_side * 0.022,
        [0.060, 0.035, 0.045],
        axes,
    )
    return [lens, body, adapter, saddle, arm, column, foot, brace, usb], optical_budget(profile)
