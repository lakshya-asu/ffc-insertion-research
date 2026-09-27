"""Explicit, proximity-gated suction attachment surrogate; no fluid simulation."""

from __future__ import annotations

import math

from pxr import Gf, UsdGeom, UsdPhysics


def _attach_patch(stage, cable_path: str, patch_local, path: str, maximum_gap_m: float) -> str:
    """Attach only a nearby aligned cable patch; use a compliant, breakable joint.

    Pressure is an assumed 30 kPa over a 3 mm effective diameter. This ideal
    force is a configurable surrogate, not a measured seal or holding force.
    Cable local +Z/-Z must align with the vacuum shoe's local Y normal.
    """
    body_path = "/World/Tool/Body"
    cache = UsdGeom.XformCache()
    tool = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(body_path))
    cable_prim = stage.GetPrimAtPath(cable_path)
    if not cable_prim or not cable_prim.HasAPI(UsdPhysics.RigidBodyAPI):
        raise ValueError("Suction requires a valid cable rigid body")
    cable = cache.GetLocalToWorldTransform(cable_prim)
    patch_local = Gf.Vec3d(*patch_local)
    patch_world = tool.Transform(patch_local)
    contact_local = cable.GetInverse().Transform(patch_world)
    scale = UsdGeom.Xformable(stage.GetPrimAtPath(cable_path + "/Shape")).GetOrderedXformOps()[0].Get()
    half = Gf.Vec3d(scale) / 2
    if abs(contact_local[0]) > half[0] or abs(contact_local[1]) > half[1]:
        raise ValueError("Vacuum patch is outside the selected cable segment")
    if abs(abs(contact_local[2]) - half[2]) > maximum_gap_m:
        raise ValueError("Cable is outside suction capture gap")
    normal = tool.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
    cable_normal = cable.TransformDir(Gf.Vec3d(0, 0, 1)).GetNormalized()
    if abs(Gf.Dot(normal, cable_normal)) < math.cos(math.radians(15)):
        raise ValueError("Cable face is not aligned with suction shoe")
    # A captured surface point belongs on the cable, not in the initial air gap.
    # Keeping patch_world as body1's anchor would make the spring resist the
    # cable moving up into the shoe during pinch, then unload on venting.
    contact_local[2] = math.copysign(half[2], contact_local[2])
    if stage.GetPrimAtPath(path):
        existing = UsdPhysics.Joint(stage.GetPrimAtPath(path))
        if (
            existing.GetJointEnabledAttr().Get()
            and UsdPhysics.DriveAPI(existing.GetPrim(), "transY").GetMaxForceAttr().Get() > 0
        ):
            raise RuntimeError("Release existing suction attachment first")
        stage.RemovePrim(path)
    j = UsdPhysics.Joint.Define(stage, path)
    j.CreateBody0Rel().SetTargets([body_path])
    j.CreateBody1Rel().SetTargets([cable_path])
    j.CreateLocalPos0Attr(Gf.Vec3f(patch_local))
    j.CreateLocalPos1Attr(Gf.Vec3f(contact_local))
    j.CreateLocalRot0Attr(Gf.Quatf(1))
    relative = tool.ExtractRotationMatrix() * cable.ExtractRotationMatrix().GetInverse()
    j.CreateLocalRot1Attr(Gf.Quatf(Gf.Matrix4d(relative, Gf.Vec3d(0)).ExtractRotationQuat()))
    mass_scale = 1 / UsdPhysics.GetStageKilogramsPerUnit(stage)
    j.CreateBreakForceAttr(mass_scale * 30000 * math.pi * (0.003 / 2) ** 2)
    j.CreateBreakTorqueAttr(mass_scale * 0.0001)
    j.CreateExcludeFromArticulationAttr(True)
    # Vacuum must not suppress the shoe/cable contact needed by the pinch.
    j.CreateCollisionEnabledAttr(True)
    # Silicone-cup compliance surrogate. Finite rotational compliance lets the
    # two separated patches carry bending moment through a force couple.
    for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
        angular = axis.startswith("rot")
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
        d.CreateTypeAttr("force")
        d.CreateTargetPositionAttr(0.0)
        d.CreateStiffnessAttr(mass_scale * (1e-6 if angular else (1000.0 if axis == "transY" else 300.0)))
        d.CreateDampingAttr(mass_scale * (1e-8 if angular else 0.2))
        d.CreateMaxForceAttr(mass_scale * (0.0001 if angular else 30000 * math.pi * 0.0015**2))
    return path


def attach(stage, cable_path: str, maximum_gap_m: float = 0.0003) -> list[str]:
    """Two 3 mm patches, 5 mm apart, supply a finite holding-force couple.

    Both must pass the geometric seal gate. These remain idealized breakable
    attachments; no vacuum-flow or lip-deformation model is implied.
    """
    index = int(cable_path.rsplit("_", 1)[1])
    paths = []
    try:
        for number, offset in enumerate((0, 1)):
            paths.append(
                _attach_patch(
                    stage,
                    f"/World/Cable/segment_{index - offset:03d}",
                    (0, 0.120, 0.086 - 0.005 * offset),
                    f"/World/Tool/SuctionAttachments/patch_{number}",
                    maximum_gap_m,
                )
            )
    except Exception:
        release(stage)
        raise
    return paths


def release(stage, mode="zero_drives") -> None:
    """Vent the ideal attachment without applying an artificial release impulse."""
    if mode not in ("zero_drives", "disable_joint"):
        raise ValueError(f"Unknown vacuum release mode: {mode}")
    root = stage.GetPrimAtPath("/World/Tool/SuctionAttachments")
    if root:
        for prim in root.GetChildren():
            if mode == "disable_joint":
                UsdPhysics.Joint(prim).GetJointEnabledAttr().Set(False)
                continue
            # All six DOFs are free. Zeroing every drive vents the surrogate
            # without destroying a constraint and rebuilding simulation handles.
            for axis in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ"):
                drive = UsdPhysics.DriveAPI(prim, axis)
                drive.GetStiffnessAttr().Set(0.0)
                drive.GetDampingAttr().Set(0.0)
                drive.GetMaxForceAttr().Set(0.0)


def attachment_geometry(stage) -> list[dict]:
    """Record actual body-anchor separation before venting the surrogate."""
    root = stage.GetPrimAtPath("/World/Tool/SuctionAttachments")
    if not root:
        return []
    cache = UsdGeom.XformCache()
    measurements = []
    for prim in root.GetChildren():
        joint = UsdPhysics.Joint(prim)
        points = []
        for relation, local in (
            (joint.GetBody0Rel(), joint.GetLocalPos0Attr()),
            (joint.GetBody1Rel(), joint.GetLocalPos1Attr()),
        ):
            points.append(
                cache.GetLocalToWorldTransform(stage.GetPrimAtPath(relation.GetTargets()[0])).Transform(
                    Gf.Vec3d(local.Get())
                )
            )
        measurements.append(
            {
                "joint": str(prim.GetPath()),
                "cable_anchor_minus_tool_anchor_m": list(points[1] - points[0]),
                "anchor_distance_m": (points[1] - points[0]).GetLength(),
            }
        )
    return measurements


def check_seal(stage, maximum_deflection_m=0.001):
    """Abort if a compliant patch separates beyond its assumed seal travel."""
    root = stage.GetPrimAtPath("/World/Tool/SuctionAttachments")
    if not root:
        return
    cache = UsdGeom.XformCache()
    for prim in root.GetChildren():
        j = UsdPhysics.Joint(prim)
        if (
            not j.GetJointEnabledAttr().Get()
            or UsdPhysics.DriveAPI(prim, "transY").GetMaxForceAttr().Get() <= 0
        ):
            continue
        points = []
        for relation, local in (
            (j.GetBody0Rel(), j.GetLocalPos0Attr()),
            (j.GetBody1Rel(), j.GetLocalPos1Attr()),
        ):
            points.append(
                cache.GetLocalToWorldTransform(stage.GetPrimAtPath(relation.GetTargets()[0])).Transform(
                    Gf.Vec3d(local.Get())
                )
            )
        if (points[0] - points[1]).GetLength() > maximum_deflection_m:
            raise RuntimeError(f"Vacuum seal displacement limit exceeded at {prim.GetPath()}")
