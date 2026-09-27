"""FR3 forward/inverse kinematics extracted from the actual composed USD joints."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


@dataclass
class Chain:
    """Seven revolute joints, column-vector transforms, SI/radian coordinates."""

    base: np.ndarray
    parent_frames: list[np.ndarray]
    child_frames: list[np.ndarray]
    axes: list[np.ndarray]
    lower: np.ndarray
    upper: np.ndarray

    def forward(self, q: np.ndarray) -> np.ndarray:
        """Return world pose of link7 from joint angles."""
        t = self.base.copy()
        for angle, p, c, axis in zip(q, self.parent_frames, self.child_frames, self.axes, strict=True):
            r = np.eye(4)
            r[:3, :3] = Rotation.from_rotvec(angle * axis).as_matrix()
            t = t @ p @ r @ np.linalg.inv(c)
        return t

    def solve(
        self,
        xyz: np.ndarray,
        rotation: np.ndarray,
        seed: np.ndarray,
        point_link7: np.ndarray,
        rng: np.random.Generator,
    ) -> tuple[np.ndarray, dict]:
        """Find a bounded pose solution; this does not prove path clearance."""

        def residual(q):
            t = self.forward(q)
            position = t[:3, 3] + t[:3, :3] @ point_link7
            return np.r_[
                100 * (position - xyz),
                Rotation.from_matrix(rotation.T @ t[:3, :3]).as_rotvec(),
                0.0001 * (q - seed),
            ]

        best = None
        seed_pose = self.forward(seed)
        near_target = np.linalg.norm(seed_pose[:3, 3] + seed_pose[:3, :3] @ point_link7 - xyz) < 0.05
        lower = np.maximum(self.lower, seed - 0.5) if near_target else self.lower
        upper = np.minimum(self.upper, seed + 0.5) if near_target else self.upper
        for attempt in range(12):
            x0 = seed if attempt == 0 else rng.uniform(lower, upper)
            result = least_squares(
                residual,
                np.clip(x0, lower + 1e-7, upper - 1e-7),
                bounds=(lower, upper),
                max_nfev=700,
                ftol=1e-10,
                xtol=1e-10,
                gtol=1e-10,
            )
            err = residual(result.x)
            metric = {
                "position_error_m": float(np.linalg.norm(err[:3]) / 100),
                "orientation_error_rad": float(np.linalg.norm(err[3:6])),
                "attempt": attempt,
            }
            if best is None or np.linalg.norm(err) < best[0]:
                best = (np.linalg.norm(err), result.x, metric)
            if metric["position_error_m"] < 0.0001 and metric["orientation_error_rad"] < 0.003:
                return result.x, metric
        raise RuntimeError(f"IK pose was not solved: target={xyz.tolist()}, best={best[2]}")


def from_stage(stage) -> Chain:
    """Extract joint anchors, axes, limits, and the base pose from vendor USD."""
    from pxr import Gf, UsdGeom, UsdPhysics

    parent, child, axes, lower, upper = [], [], [], [], []
    cache = UsdGeom.XformCache()
    base = None
    for i in range(1, 8):
        j = UsdPhysics.RevoluteJoint.Get(stage, f"/World/FR3/Physics/fr3v2_1_joint{i}")
        if i == 1:
            base = np.asarray(
                cache.GetLocalToWorldTransform(stage.GetPrimAtPath(j.GetBody0Rel().GetTargets()[0]))
            ).T
        for positions, rotations, output in (
            (j.GetLocalPos0Attr(), j.GetLocalRot0Attr(), parent),
            (j.GetLocalPos1Attr(), j.GetLocalRot1Attr(), child),
        ):
            t = np.eye(4)
            t[:3, 3] = positions.Get()
            t[:3, :3] = np.asarray(Gf.Matrix3d(Gf.Rotation(rotations.Get()))).T
            output.append(t)
        axes.append(np.eye(3)["XYZ".index(j.GetAxisAttr().Get())])
        lower.append(np.deg2rad(j.GetLowerLimitAttr().Get()))
        upper.append(np.deg2rad(j.GetUpperLimitAttr().Get()))
    return Chain(base, parent, child, axes, np.asarray(lower), np.asarray(upper))
