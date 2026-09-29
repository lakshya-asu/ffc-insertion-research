"""Parametric USD scene; import only after Isaac Sim has initialized its USD plugins."""

from __future__ import annotations

import json
import math
import os
from pathlib import Path

from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade


def pose(prim, position, rotation=None):
    """Set an unscaled rigid transform in metres."""
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*position))
    x.AddOrientOp().Set(rotation or Gf.Quatf(1))


def box(stage, path, position, size, color, mass=None, collision=True):
    """Author a box with an unscaled parent so joint frames remain metric."""
    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    pose(root, position)
    shape = UsdGeom.Cube.Define(stage, path + "/Shape")
    shape.CreateSizeAttr(1)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        contact = PhysxSchema.PhysxCollisionAPI.Apply(shape.GetPrim())
        contact.CreateContactOffsetAttr(0.00002)
        contact.CreateRestOffsetAttr(0.0)
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(root)
        m = UsdPhysics.MassAPI.Apply(root)
        m.CreateMassAttr(mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(0))
        a, b, c = size
        m.CreateDiagonalInertiaAttr(
            Gf.Vec3f(mass * (b * b + c * c) / 12, mass * (a * a + c * c) / 12, mass * (a * a + b * b) / 12)
        )
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(root)
        rb.CreateSolverPositionIterationCountAttr(32)
        rb.CreateSolverVelocityIterationCountAttr(8)
        rb.CreateEnableCCDAttr(True)
    return root


def joint(stage, path, kind, body0, body1, pos0=(0, 0, 0), pos1=(0, 0, 0)):
    """Create a joint with collision disabled only between its connected bodies."""
    j = kind.Define(stage, path)
    if body0:
        j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot0Attr(Gf.Quatf(1))
    j.CreateLocalRot1Attr(Gf.Quatf(1))
    j.CreateCollisionEnabledAttr(False)
    return j


def include_bracket_inertia(stage, body_path, density=2700.0):
    """Combine housing mass with aluminium box brackets, including offset inertia."""
    import numpy as np
    from scipy.spatial.transform import Rotation

    body = stage.GetPrimAtPath(body_path)
    mass_api = UsdPhysics.MassAPI(body)
    parts = [(mass_api.GetMassAttr().Get(), np.zeros(3), np.diag(mass_api.GetDiagonalInertiaAttr().Get()))]
    for child in body.GetChildren():
        shape = child.GetChild("Shape")
        if not shape or not shape.IsA(UsdGeom.Cube):
            continue
        size = np.asarray(UsdGeom.Xformable(shape).GetOrderedXformOps()[0].Get())
        center = np.asarray(UsdGeom.Xformable(child).GetOrderedXformOps()[0].Get())
        mass = float(density * np.prod(size))
        diagonal = mass / 12 * (np.sum(size * size) - size * size)
        parts.append((mass, center, np.diag(diagonal)))
    total = sum(p[0] for p in parts)
    com = sum(m * p for m, p, _ in parts) / total
    inertia = np.zeros((3, 3))
    for mass, center, tensor in parts:
        r = center - com
        inertia += tensor + mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    values, axes = np.linalg.eigh(inertia)
    if np.linalg.det(axes) < 0:
        axes[:, 0] *= -1
    q = Rotation.from_matrix(axes).as_quat()
    mass_api.GetMassAttr().Set(total)
    mass_api.GetCenterOfMassAttr().Set(Gf.Vec3f(*com))
    mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*values))
    mass_api.CreatePrincipalAxesAttr(Gf.Quatf(float(q[3]), Gf.Vec3f(*q[:3])))


def drive(j, axis, stiffness, damping, target, max_force):
    """Author force drive; angular USD values use degrees, not radians."""
    d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), axis)
    d.CreateTypeAttr("force")
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateTargetPositionAttr(target)
    d.CreateMaxForceAttr(max_force)
    return d


def camera(stage, path, eye, target, focal=35.0):
    """Create a pinhole camera; optics are proposed, not calibrated hardware."""
    c = UsdGeom.Camera.Define(stage, path)
    view = Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1))
    c.AddTransformOp().Set(view.GetInverse())
    c.CreateFocalLengthAttr(focal)
    c.CreateHorizontalApertureAttr(36)
    c.CreateVerticalApertureAttr(24)
    c.CreateClippingRangeAttr(Gf.Vec2f(0.0001, 20))
    return c


def make_cable(stage, cfg, root_path="/World/Cable"):
    """Discrete ribbon: soft out-of-plane bend and equal stiffer swing axes.

    Joint X maps to world Y, across the ribbon. PhysX D6 swing drive gains
    must match, so twist and in-plane bend are not independently identified.
    """
    c = cfg["cable"]
    n, length = c["segments"], c["length_m"] / c["segments"]
    sx, sy, sz = c["start_xyz_m"]
    paths = []
    material = UsdShade.Material.Define(stage, "/World/Materials/Cable")
    mat = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    mat.CreateStaticFrictionAttr(0.5)
    mat.CreateDynamicFrictionAttr(0.4)
    mat.CreateRestitutionAttr(0.0)
    for i in range(n):
        p = f"{root_path}/segment_{i:03d}"
        stiff = i >= n - c["stiffener_segments"]
        box(
            stage,
            p,
            (sx + (i + 0.5) * length, sy, sz),
            (length, c["width_m"], c["thickness_m"]),
            (0.08, 0.3, 0.85) if stiff else (0.87, 0.80, 0.58),
            c["mass_kg"] / n,
        )
        PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(p)).GetSolverPositionIterationCountAttr().Set(
            c.get("solver_iterations", 32)
        )
        if c.get("benchmark_drag_per_s", 0):
            body_api = PhysxSchema.PhysxRigidBodyAPI(stage.GetPrimAtPath(p))
            body_api.CreateLinearDampingAttr(c["benchmark_drag_per_s"])
            body_api.CreateAngularDampingAttr(c["benchmark_drag_per_s"])
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(p + "/Shape")).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        if stiff:
            # Visible conductor strips on the bottom face. No electrical physics.
            for k in range(15):
                box(
                    stage,
                    p + f"/contact_{k:02d}",
                    (0, (k - 7) * 0.0005, -c["thickness_m"] / 2 - 0.000003),
                    (length * 0.8, 0.0003, 0.000006),
                    (0.8, 0.56, 0.15),
                    collision=False,
                )
        paths.append(p)
        if i == 0:
            continue
        j = joint(
            stage,
            f"{root_path}/Joints/joint_{i:03d}",
            UsdPhysics.Joint,
            paths[i - 1],
            p,
            (length / 2, 0, 0),
            (-length / 2, 0, 0),
        )
        rotation = Gf.Quatf(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90).GetQuat())
        j.GetLocalRot0Attr().Set(rotation)
        j.GetLocalRot1Attr().Set(rotation)
        for axis in ("transX", "transY", "transZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr(1)
            lim.CreateHighAttr(-1)
        for axis in ("rotX", "rotY", "rotZ"):
            lim = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
            lim.CreateLowAttr(-35 if axis == "rotX" else -10)
            lim.CreateHighAttr(35 if axis == "rotX" else 10)
            k = c["bend_stiffness_nm_per_degree"] if axis == "rotX" else c["swing_stiffness_nm_per_degree"]
            if i > n - c["stiffener_segments"]:
                k *= c["stiffener_gain_multiplier"]
            gain_scale = c.get("angular_drive_gain_scale", 1.0)
            drive(j, axis, k * gain_scale, c["damping_nm_s_per_degree"] * gain_scale, 0, 0.01)
    if c.get("reduced_coordinate", False):
        root = stage.GetPrimAtPath(paths[0])
        UsdPhysics.ArticulationRootAPI.Apply(root)
        articulation = PhysxSchema.PhysxArticulationAPI.Apply(root)
        articulation.CreateEnabledSelfCollisionsAttr(True)
        articulation.CreateSolverPositionIterationCountAttr(c.get("solver_iterations", 128))
        articulation.CreateSolverVelocityIterationCountAttr(c.get("solver_velocity_iterations", 0))
    return paths


def make_connector(stage, cfg):
    """Build an open aperture with sidewalls, roof, floor, stop and hinged latch."""
    c = cfg["connector"]
    x, y, z = c["mouth_xyz_m"]
    w, d, h = c["width_m"], c["depth_m"], c["height_m"]
    sw, sh = c["slot_width_m"], c["slot_height_m"]
    pcb_top = z - 0.001
    box(stage, "/World/PCBStand", (x + 0.05, y, 0.06), (0.10, 0.08, 0.12), (0.25, 0.3, 0.34))
    box(stage, "/World/PCB", (x + 0.045, y, pcb_top - 0.0008), (0.09, 0.065, 0.0016), (0.02, 0.28, 0.12))
    root = "/World/Connector"
    wall = (w - sw) / 2
    for sign, name in ((-1, "left"), (1, "right")):
        box(
            stage,
            root + "/" + name,
            (x + d / 2, y + sign * (sw / 2 + wall / 2), z),
            (d, wall, h),
            (0.82, 0.81, 0.73),
        )
    slab = (h - sh) / 2
    for sign, name in ((-1, "floor"), (1, "roof")):
        box(
            stage,
            root + "/" + name,
            (x + d / 2, y, z + sign * (sh / 2 + slab / 2)),
            (d, sw, slab),
            (0.82, 0.81, 0.73),
        )
    stop = c["insertion_depth_m"]
    box(stage, root + "/backstop", (x + (stop + d) / 2, y, z), (d - stop, sw, sh), (0.4, 0.38, 0.3))
    # Hinge axis across connector width; bar rotates upward toward mouth.
    hinge = (x + d - 0.0004, y, z + h / 2 + 0.0005)
    latch = box(
        stage,
        root + "/Latch",
        (hinge[0] - 0.001, hinge[1], hinge[2]),
        (0.002, w, 0.0007),
        (0.1, 0.1, 0.12),
        0.00005,
    )
    j = joint(
        stage,
        root + "/latch_hinge",
        UsdPhysics.RevoluteJoint,
        None,
        str(latch.GetPath()),
        hinge,
        (0.001, 0, 0),
    )
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(0)
    j.CreateUpperLimitAttr(85)
    drive(j, "angular", 0.0001, 0.00001, c["latch_open_deg"], 0.02)


def make_fixture(stage, cfg):
    """Passive raised shelf with open underside and a rear tail support."""
    f = cfg["fixture"]
    x, y, z = f["center_xyz_m"]
    length, width, thick = f["shelf_length_m"], f["shelf_width_m"], f["shelf_thickness_m"]
    box(stage, "/World/Regrasp/Shelf", (x, y, z - thick / 2), (length, width, thick), (0.25, 0.55, 0.65))
    for dx in (-0.05, 0.02):
        box(
            stage,
            f"/World/Regrasp/Post_{int((dx + 0.1) * 1000)}",
            (x + dx, y, (z - thick) / 2),
            (0.008, width, z - thick),
            (0.3, 0.35, 0.4),
        )
    # Low guide along the back; end remains unobstructed for pinching.
    box(
        stage,
        "/World/Regrasp/BackGuide",
        (x - 0.015, y + width / 2 + 0.001, z + 0.001),
        (length - 0.03, 0.002, 0.005),
        (0.1, 0.32, 0.4),
    )


def make_tool(stage, link7, tool_cfg=None):
    """Concept with independent deployment and clamp axes, attached to FR3."""
    tool_cfg = tool_cfg or {}
    matrix = UsdGeom.XformCache().GetLocalToWorldTransform(link7)
    frame = Gf.Matrix4d().SetTranslate(Gf.Vec3d(0, 0, 0.107)) * matrix
    rotation = Gf.Quatf(frame.ExtractRotationQuat())

    def part(name, xyz, size, color, mass):
        p = box(stage, "/World/Tool/" + name, frame.Transform(Gf.Vec3d(*xyz)), size, color, mass)
        pose(p, frame.Transform(Gf.Vec3d(*xyz)), rotation)
        return str(p.GetPath())

    body = part("Body", (0, 0, 0.025), (0.04, 0.04, 0.05), (0.2, 0.23, 0.27), 0.18)
    joint(
        stage,
        "/World/Tool/flange_mount",
        UsdPhysics.FixedJoint,
        str(link7.GetPath()),
        body,
        (0, 0, 0.107),
        (0, 0, -0.025),
    )
    # Structural stem and short shoe; only the final 12 mm contacts the ribbon.
    box(stage, body + "/Neck", (0, 0.056, 0.04), (0.012, 0.112, 0.015), (0.68, 0.7, 0.74))
    box(stage, body + "/Stem", (0, 0.112, 0.058), (0.01, 0.01, 0.045), (0.68, 0.7, 0.74))
    box(stage, body + "/UpperSupport", (0, 0.1185, 0.085), (0.01, 0.003, 0.014), (0.68, 0.7, 0.74))
    box(stage, body + "/DeployRail", (0, 0.092, 0.070), (0.04, 0.008, 0.008), (0.3, 0.35, 0.4))
    cup = UsdGeom.Cylinder.Define(stage, body + "/VacuumPatch")
    pose(cup.GetPrim(), (0, 0.1195, 0.086))
    cup.CreateAxisAttr("Y")
    cup.CreateRadiusAttr(0.0015)
    cup.CreateHeightAttr(0.001)
    cup.CreateDisplayColorAttr([Gf.Vec3f(0.07, 0.1, 0.12)])
    rear_cup = UsdGeom.Cylinder.Define(stage, body + "/RearVacuumPatch")
    pose(rear_cup.GetPrim(), (0, 0.1195, 0.081))
    rear_cup.CreateAxisAttr("Y")
    rear_cup.CreateRadiusAttr(0.0015)
    rear_cup.CreateHeightAttr(0.001)
    rear_cup.CreateDisplayColorAttr([Gf.Vec3f(0.07, 0.1, 0.12)])
    carriage = part("Carriage", (0.018, 0.097, 0.111), (0.012, 0.01, 0.014), (0.15, 0.45, 0.65), 0.025)
    j = joint(stage, "/World/Tool/deploy", UsdPhysics.PrismaticJoint, body, carriage, (0.018, 0.097, 0.086))
    j.CreateAxisAttr("X")
    j.CreateLowerLimitAttr(-0.018)
    j.CreateUpperLimitAttr(0)
    drive(j, "linear", 1000, 30, 0, 5)
    PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateMaxJointVelocityAttr(0.03)
    jaw = part("LowerJaw", (0.018, 0.128, 0.111), (0.01, 0.003, 0.009), (0.7, 0.72, 0.76), 0.008)
    # Keep the rod's inner face 3 mm outside the upper shoe's side face.
    # The old tangent faces generated large internal contact impulses.
    box(stage, jaw + "/GuideRod", (0.010, -0.020, 0), (0.004, 0.04, 0.005), (0.35, 0.4, 0.45))
    box(stage, jaw + "/RodLug", (0.0075, 0, 0), (0.009, 0.003, 0.005), (0.7, 0.72, 0.76))
    j = joint(stage, "/World/Tool/clamp", UsdPhysics.PrismaticJoint, carriage, jaw, (0, 0.031, 0))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(-0.02)
    j.CreateUpperLimitAttr(0)
    drive(
        j,
        "linear",
        tool_cfg.get("clamp_stiffness_n_per_m", 600),
        15,
        -0.018,
        tool_cfg.get("maximum_clamp_force_n", 2),
    )
    PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateMaxJointVelocityAttr(0.03)
    tip = UsdGeom.Xform.Define(stage, body + "/TipReference")
    pose(tip.GetPrim(), (0, 0.12015, 0.0985))
    include_bracket_inertia(stage, body)
    include_bracket_inertia(stage, jaw)
    patch_radius = tool_cfg.get("torsional_patch_radius_m", 0.0)
    if patch_radius:
        for shape in (body + "/UpperSupport/Shape", jaw + "/Shape"):
            contact = PhysxSchema.PhysxCollisionAPI(stage.GetPrimAtPath(shape))
            contact.CreateTorsionalPatchRadiusAttr(patch_radius)
            contact.CreateMinTorsionalPatchRadiusAttr(tool_cfg["minimum_torsional_patch_radius_m"])
    compliance = tool_cfg.get("contact_compliance")
    if compliance:
        # Effective jaw-facing compliance, not a resolved silicone pad mesh.
        # Values must be identified from force/displacement measurements.
        material = UsdShade.Material.Define(stage, "/World/Materials/JawFacing")
        surface = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
        surface.CreateStaticFrictionAttr(0.5)
        surface.CreateDynamicFrictionAttr(0.4)
        surface.CreateRestitutionAttr(0.0)
        elastic = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
        elastic.CreateCompliantContactStiffnessAttr(compliance["stiffness_n_per_m"])
        elastic.CreateCompliantContactDampingAttr(compliance["damping_n_s_per_m"])
        elastic.CreateCompliantContactAccelerationSpringAttr(False)
        for shape in (body + "/UpperSupport/Shape", jaw + "/Shape"):
            UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(shape)).Bind(
                material, UsdShade.Tokens.weakerThanDescendants, "physics"
            )


def build(stage, project: Path, cfg: dict) -> dict:
    """Build scene in metres, Z up, and return paths needed by the runner."""
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetMetadata("customLayerData", {"ffc_provenance": json.dumps(cfg["provenance"])})
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(9.81)
    phys = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
    phys.CreateEnableCCDAttr(True)
    phys.CreateSolverTypeAttr(cfg.get("physics_solver", "TGS"))
    phys.CreateEnableExternalForcesEveryIterationAttr(cfg.get("external_forces_every_iteration", False))
    phys.CreateTimeStepsPerSecondAttr(round(1 / cfg["physics_dt_s"]))
    if cfg.get("cable_model") == "shell":
        phys.CreateEnableGPUDynamicsAttr(True)
        phys.CreateBroadphaseTypeAttr("GPU")
    box(stage, "/World/Desk", (0.35, 0, -0.025), (1.1, 0.85, 0.05), (0.45, 0.47, 0.49))
    box(stage, "/World/Floor", (0.3, 0, -0.78), (5, 5, 0.1), (0.15, 0.17, 0.2))
    asset = project / "third_party/franka_isaac/fr3v2_1/fr3v2_1.usda"
    robot = UsdGeom.Xform.Define(stage, "/World/FR3").GetPrim()
    robot.GetReferences().AddReference(os.path.relpath(asset, Path(stage.GetRootLayer().realPath).parent))
    articulation = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/World/FR3/Geometry/base"))
    articulation.CreateEnabledSelfCollisionsAttr(True)
    articulation.CreateSolverPositionIterationCountAttr(32)
    articulation.CreateSolverVelocityIterationCountAttr(8)
    links = [p for p in Usd.PrimRange(robot) if p.GetName() == "fr3v2_1_link7"]
    if len(links) != 1:
        raise RuntimeError("Expected exactly one FR3 link7; check the referenced asset")
    for i, home in enumerate(cfg["robot_home_rad"], 1):
        j = UsdPhysics.RevoluteJoint.Get(stage, f"/World/FR3/Physics/fr3v2_1_joint{i}")
        if not j:
            raise RuntimeError(f"Missing FR3 joint {i}")
        controller = cfg.get("robot_controller", {})
        gain = controller.get("position_gain_scale", 5.0)
        drive(
            j,
            "angular",
            gain * (40 if i < 5 else 15),
            gain * (4 if i < 5 else 1.5),
            math.degrees(home),
            87 if i < 5 else 12,
        )
    make_tool(stage, links[0], cfg.get("tool"))
    if cfg.get("cable_model") == "shell":
        from ffc.isaac_shell import make_shell

        cables = [make_shell(stage, cfg)]
    else:
        cables = make_cable(stage, cfg)
    make_connector(stage, cfg)
    make_fixture(stage, cfg)
    light = UsdLux.DomeLight.Define(stage, "/World/Lighting/Dome")
    light.CreateIntensityAttr(700)
    area = UsdLux.RectLight.Define(stage, "/World/Lighting/Key")
    area.AddTranslateOp().Set(Gf.Vec3d(0.4, 0, 1.5))
    area.CreateIntensityAttr(1600)
    area.CreateWidthAttr(1.0)
    area.CreateHeightAttr(1.0)
    camera(stage, "/World/Cameras/Overview", (1.35, -1.3, 1.05), (0.3, 0, 0.25))
    camera(stage, "/World/Cameras/Overhead", (0.4, -0.001, 0.95), (0.4, 0, 0))
    x, y, z = cfg["connector"]["mouth_xyz_m"]
    camera(stage, "/World/Cameras/ConnectorMacro", (x - 0.035, y - 0.022, z + 0.018), (x + 0.002, y, z), 45)
    if "solver_velocity_iterations" in cfg:
        velocity_iterations = cfg["solver_velocity_iterations"]
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateSolverVelocityIterationCountAttr(
                    velocity_iterations
                )
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                PhysxSchema.PhysxArticulationAPI.Apply(prim).CreateSolverVelocityIterationCountAttr(
                    velocity_iterations
                )
    if cfg.get("disable_stabilization", False):
        for prim in stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                api = PhysxSchema.PhysxRigidBodyAPI.Apply(prim)
                api.CreateSleepThresholdAttr(0)
                api.CreateStabilizationThresholdAttr(0)
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                api.CreateSleepThresholdAttr(0)
                api.CreateStabilizationThresholdAttr(0)
    mass_scale = cfg.get("mass_unit_scale", 1.0)
    if mass_scale != 1:
        if cfg.get("cable_model") == "shell":
            raise ValueError("Mass-unit conversion currently covers rigid models only")
        UsdPhysics.SetStageKilogramsPerUnit(stage, 1 / mass_scale)
        for prim in stage.Traverse():
            for attr in prim.GetAttributes():
                name, value = attr.GetName(), attr.Get()
                if value is None:
                    continue
                if name in ("physics:mass", "physics:density") and value > 0:
                    attr.Set(value * mass_scale)
                elif name == "physics:diagonalInertia" and sum(value) > 0:
                    attr.Set(Gf.Vec3f(value) * mass_scale)
                elif name in (
                    "physxMaterial:compliantContactStiffness",
                    "physxMaterial:compliantContactDamping",
                ):
                    attr.Set(value * mass_scale)
                elif name.startswith("drive:") and name.rsplit(":", 1)[-1] in (
                    "stiffness",
                    "damping",
                    "maxForce",
                ):
                    attr.Set(value * mass_scale)
    stage.GetRootLayer().Save()
    return {
        "robot_root": "/World/FR3/Geometry/base",
        "cable_paths": cables,
        "camera_paths": ["/World/Cameras/" + n for n in ("Overview", "Overhead", "ConnectorMacro")],
    }
