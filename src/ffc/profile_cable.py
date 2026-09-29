"""Sectional properties and a free articulated cable for exploratory contact work.

The short homogeneous bending benchmark does not qualify this variable-section,
unanchored contact model. Equivalent material, twist and damping remain assumptions.
"""

import math

from ffc.cable_spec import values, width_at


def sections(spec, count=100):
    if count not in (100, 200):
        raise ValueError("Use 100 or 200 segments for the profile comparison")
    v = values(spec)
    ds = v["length_m"] / count
    result = []
    for i in range(count):
        distance = (i + 0.5) * ds
        width = width_at(spec, distance)
        reinforced = min(distance, v["length_m"] - distance) < v["stiffener_length_m"]
        height = v["header_thickness_m"] if reinforced else v["body_thickness_m"]
        result.append(
            {
                "center_m": distance,
                "length_m": ds,
                "width_m": width,
                "thickness_m": height,
                "reinforced": reinforced,
                "mass_kg": v["equivalent_density_kg_m3"] * width * height * ds,
                "ei_nm2": v["equivalent_young_pa"] * width * height**3 / 12,
            }
        )
    return result


def joint_stiffness_nm_per_degree(left, right):
    compliance = left["length_m"] / (2 * left["ei_nm2"]) + right["length_m"] / (2 * right["ei_nm2"])
    return math.pi / (180 * compliance)


def create_profile_cable(stage, spec, z0, count=100):
    """Create a free cable along world +Y, with no attachment to tool or ground."""
    from pxr import PhysxSchema, UsdPhysics, UsdShade

    from ffc.isaac_scene import box, drive, joint

    v = values(spec)
    profile = sections(spec, count)
    paths = []
    material = UsdShade.Material.Define(stage, "/World/CableMaterial")
    friction = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(v["static_friction"])
    friction.CreateDynamicFrictionAttr(v["dynamic_friction"])
    friction.CreateRestitutionAttr(0)
    for i, section in enumerate(profile):
        path = f"/World/Cable/segment_{i:03d}"
        ds, width, h = section["length_m"], section["width_m"], section["thickness_m"]
        body = box(
            stage,
            path,
            (0, section["center_m"], z0),
            (width, ds, h),
            (0.06, 0.27, 0.75) if section["reinforced"] else (0.035, 0.045, 0.06),
            section["mass_kg"],
        )
        api = PhysxSchema.PhysxRigidBodyAPI(body)
        api.CreateSolverPositionIterationCountAttr(255)
        api.CreateSolverVelocityIterationCountAttr(8)
        api.CreateSleepThresholdAttr(0)
        api.CreateLinearDampingAttr(20)
        api.CreateAngularDampingAttr(20)
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path + "/Shape")).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        distance = section["center_m"]
        end = (
            "mini"
            if distance < v["exposed_length_m"]
            else ("standard" if distance > v["length_m"] - v["exposed_length_m"] else None)
        )
        if end:
            pins = int(v[end + "_contacts"])
            for pin in range(pins):
                box(
                    stage,
                    path + f"/Contact{pin:02d}",
                    ((pin - (pins - 1) / 2) * v[end + "_pitch_m"], 0, h / 2 + 0.000001),
                    (0.0003, ds, 0.000002),
                    (0.88, 0.64, 0.2),
                    collision=False,
                )
        paths.append(path)
        if i:
            j = joint(
                stage,
                f"/World/Cable/Joints/joint_{i:03d}",
                UsdPhysics.Joint,
                paths[i - 1],
                path,
                (0, ds / 2, 0),
                (0, -ds / 2, 0),
            )
            for axis in ("transX", "transY", "transZ"):
                limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
                limit.CreateLowAttr(1)
                limit.CreateHighAttr(-1)
            k = joint_stiffness_nm_per_degree(profile[i - 1], section)
            for axis in ("rotX", "rotY", "rotZ"):
                limit = UsdPhysics.LimitAPI.Apply(j.GetPrim(), axis)
                limit.CreateLowAttr(-35 if axis == "rotX" else -10)
                limit.CreateHighAttr(35 if axis == "rotX" else 10)
                drive(j, axis, k * (1 if axis == "rotX" else 10), k * 0.01 * 1e-5, 0, 0.01)
    root = stage.GetPrimAtPath(paths[0])
    UsdPhysics.ArticulationRootAPI.Apply(root)
    art = PhysxSchema.PhysxArticulationAPI.Apply(root)
    art.CreateSolverPositionIterationCountAttr(255)
    art.CreateSolverVelocityIterationCountAttr(8)
    art.CreateEnabledSelfCollisionsAttr(True)
    art.CreateSleepThresholdAttr(0)
    art.CreateStabilizationThresholdAttr(0)
    return paths, profile
