"""Structural checks on composed USD before any claim about physics or assembly."""

from __future__ import annotations

from itertools import product

from pxr import Gf, UsdGeom, UsdPhysics


def validate_stage(stage, cfg: dict) -> dict:
    """Fail on missing bodies, mass errors, invalid anchors or obstructed nominal fit."""
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    assert UsdGeom.GetStageUpAxis(stage) == "Z"
    c, connector = cfg["cable"], cfg["connector"]
    assert connector["slot_width_m"] > c["width_m"]
    assert connector["slot_height_m"] > c["thickness_m"]
    assert 0 < connector["insertion_depth_m"] < connector["depth_m"]
    assert c["segments"] >= 3
    for i in range(1, 8):
        assert stage.GetPrimAtPath(f"/World/FR3/Physics/fr3v2_1_joint{i}")
    for name in ("deploy", "clamp"):
        assert stage.GetPrimAtPath("/World/Tool/" + name).IsA(UsdPhysics.PrismaticJoint)
    # The guide rod must clear the upper shoe throughout lateral deployment.
    # Evaluate authored box vertices in the tool-body frame, including the
    # deployment joint's full negative travel, before running any physics.
    cache = UsdGeom.XformCache()
    body_inverse = cache.GetLocalToWorldTransform(stage.GetPrimAtPath("/World/Tool/Body")).GetInverse()

    def body_x_extent(path):
        transform = cache.GetLocalToWorldTransform(stage.GetPrimAtPath(path)) * body_inverse
        values = [transform.Transform(Gf.Vec3d(*corner))[0] for corner in product((-0.5, 0.5), repeat=3)]
        return min(values), max(values)

    rod_min, _ = body_x_extent("/World/Tool/LowerJaw/GuideRod/Shape")
    _, shoe_max = body_x_extent("/World/Tool/Body/UpperSupport/Shape")
    deployment_travel = (
        UsdPhysics.PrismaticJoint(stage.GetPrimAtPath("/World/Tool/deploy")).GetLowerLimitAttr().Get()
    )
    guide_clearance = rod_min + deployment_travel - shoe_max
    assert guide_clearance > 0.001, f"Guide rod/shoe clearance is only {guide_clearance} m"
    tool = cfg.get("tool", {})
    radius = tool.get("torsional_patch_radius_m", 0.0)
    minimum_radius = tool.get("minimum_torsional_patch_radius_m", 0.0)
    assert 0 <= minimum_radius <= radius
    assert radius <= min(c["width_m"], c["length_m"] / c["segments"]) / 2
    if cfg.get("cable_model") == "shell":
        from ffc.isaac_shell import validate_shell

        return {
            **validate_shell(stage, cfg),
            "arm_joints": 7,
            "tool_prismatic_joints": 2,
            "guide_rod_shoe_clearance_m": guide_clearance,
            "slot_width_clearance_m": connector["slot_width_m"] - c["width_m"],
            "slot_height_clearance_m": connector["slot_height_m"] - c["thickness_m"],
        }
    total = 0
    cache = UsdGeom.XformCache()
    for i in range(c["segments"]):
        p = stage.GetPrimAtPath(f"/World/Cable/segment_{i:03d}")
        assert p and p.HasAPI(UsdPhysics.RigidBodyAPI)
        total += UsdPhysics.MassAPI(p).GetMassAttr().Get() * UsdPhysics.GetStageKilogramsPerUnit(stage)
    assert abs(total - c["mass_kg"]) < 1e-8
    errors = []
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.Joint) or not str(prim.GetPath()).startswith("/World/Cable/Joints"):
            continue
        j = UsdPhysics.Joint(prim)
        points = []
        for rel, attr in ((j.GetBody0Rel(), j.GetLocalPos0Attr()), (j.GetBody1Rel(), j.GetLocalPos1Attr())):
            targets = rel.GetTargets()
            assert len(targets) == 1 and stage.GetPrimAtPath(targets[0])
            points.append(
                cache.GetLocalToWorldTransform(stage.GetPrimAtPath(targets[0])).Transform(
                    Gf.Vec3d(attr.Get())
                )
            )
        errors.append((points[0] - points[1]).GetLength())
    assert len(errors) == c["segments"] - 1
    assert max(errors) < 1e-7
    for i in range(1, 8):
        assert stage.GetPrimAtPath(f"/World/FR3/Physics/fr3v2_1_joint{i}")
    for name in ("deploy", "clamp"):
        assert stage.GetPrimAtPath("/World/Tool/" + name).IsA(UsdPhysics.PrismaticJoint)
    return {
        "cable_segments": c["segments"],
        "cable_mass_kg": total,
        "maximum_initial_anchor_error_m": max(errors),
        "slot_width_clearance_m": connector["slot_width_m"] - c["width_m"],
        "slot_height_clearance_m": connector["slot_height_m"] - c["thickness_m"],
        "arm_joints": 7,
        "tool_prismatic_joints": 2,
        "guide_rod_shoe_clearance_m": guide_clearance,
    }
