"""Side-entry reference collision geometry; all undimensioned details explicit.

No state from this model is a controller input. Dynamic contact poses are offline
replay data. Import after Isaac starts (PhysxSchema required).
"""

import math

from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics, UsdShade

from ffc.isaac_scene import box, drive, joint, pose


def build_socket(stage, settings, evidence, z0, lateral=0.0, height=0.0, lead_in=True):
    """Return collider paths, moving contacts and geometric evaluation metadata."""
    cfg, dims = settings, evidence["zero2"]["drawing_dimensions_mm"]
    width, gap = cfg["assumed_throat_width_m"], cfg["assumed_throat_height_m"]
    length = dims["insertion_depth"] / 1000
    lead = cfg["assumed_lead_in_length_m"] if lead_in else 0.0
    expansion = cfg["assumed_lead_in_expansion_each_side_m"] if lead_in else 0.0
    mouth, z = cfg["mouth_y_m"], z0 + height
    paths, moving = [], []
    material = UsdShade.Material.Define(stage, "/World/SocketMaterial")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    fs, fd = cfg["assumed_friction_static_dynamic"]
    api.CreateStaticFrictionAttr(fs)
    api.CreateDynamicFrictionAttr(fd)
    api.CreateRestitutionAttr(0)

    def finish(prim, dynamic=False):
        if not dynamic:
            UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
            PhysxSchema.PhysxRigidBodyAPI(prim).CreateEnableCCDAttr(False)
        shape = stage.GetPrimAtPath(str(prim.GetPath()) + "/Shape")
        collision = PhysxSchema.PhysxCollisionAPI.Apply(shape)
        collision.CreateContactOffsetAttr(cfg["contact_offset_m"])
        collision.CreateRestOffsetAttr(cfg["rest_offset_m"])
        UsdShade.MaterialBindingAPI.Apply(shape).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        paths.append(str(shape.GetPath()))
        return prim

    def block(name, center, size, color=(0.08, 0.09, 0.10)):
        return finish(box(stage, "/World/Slot/" + name, center, size, color, mass=0.0001))

    # Local coordinates: X across cable, insertion -Y, PCB normal +Z.
    # Roof/open slider and floor leave an actual empty cavity between colliders.
    wall = 0.00035
    for sign, name in [(-1, "floor"), (1, "open_slider_roof")]:
        block(
            name,
            (lateral, mouth - (length + lead) / 2, z + sign * (gap / 2 + wall / 2)),
            (width + 2 * wall, length - lead, wall),
        )
    for sign, name in [(-1, "left"), (1, "right")]:
        block(
            name,
            (lateral + sign * (width / 2 + wall / 2), mouth - (length + lead) / 2, z),
            (wall, length - lead, gap + 2 * wall),
        )
    block("backstop", (lateral, mouth - length - wall / 2, z), (width + 2 * wall, wall, gap + 2 * wall))

    # Sloped guide boxes end exactly on the straight throat surface. Thickness
    # is offset outwards, so it does not intrude across that surface.
    if lead:
        angle = math.atan2(expansion, lead)
        for sign, name in [(-1, "lower_guide"), (1, "upper_guide")]:
            center = (
                lateral,
                mouth - lead / 2 + wall / 2 * math.sin(angle),
                z + sign * (gap / 2 + expansion / 2 + wall / 2 * math.cos(angle)),
            )
            prim = block(name, center, (width + 2 * wall, math.hypot(lead, expansion), wall))
            pose(prim, center, Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(sign * math.sin(angle / 2), 0, 0)))
        for sign, name in [(-1, "left_guide"), (1, "right_guide")]:
            center = (
                lateral + sign * (width / 2 + expansion / 2 + wall / 2 * math.cos(angle)),
                mouth - lead / 2 + wall / 2 * math.sin(angle),
                z,
            )
            prim = block(name, center, (wall, math.hypot(lead, expansion), gap + 2 * wall + 2 * expansion))
            pose(prim, center, Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, -sign * math.sin(angle / 2))))

    # A bounded Z spring for each contact nose. Parameters are hypotheses.
    pitch = evidence["zero2"]["pitch_mm"] / 1000
    count = evidence["zero2"]["positions"]
    depth = (dims["insertion_depth"] - dims["length_to_contact_point"]) / 1000
    for i in range(count):
        path = f"/World/Slot/contact_{i:02}"
        nose_h = 0.00010
        center = (
            lateral + (i - (count - 1) / 2) * pitch,
            mouth - depth,
            z + cfg["assumed_contact_rest_top_relative_to_cable_center_m"] - nose_h / 2,
        )
        prim = box(
            stage,
            path,
            center,
            (cfg["assumed_contact_nose_width_m"], cfg["assumed_contact_nose_length_m"], nose_h),
            (0.75, 0.57, 0.16),
            mass=cfg["assumed_contact_mass_kg"],
        )
        # Analytic primitives avoid PhysX convex-cooking expansion at this scale.
        # A flat rear land and inclined front plate form one spring-supported nose.
        half_l = cfg["assumed_contact_nose_length_m"] / 2
        bevel = cfg["assumed_contact_nose_bevel_length_m"]
        nose_w = cfg["assumed_contact_nose_width_m"]
        land = UsdGeom.Cube(stage.GetPrimAtPath(path + "/Shape"))
        xf = UsdGeom.Xformable(land)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(0, -bevel / 2, 0))
        xf.AddScaleOp().Set(Gf.Vec3f(nose_w, 2 * half_l - bevel, nose_h))
        finish(prim, dynamic=True)
        ramp = UsdGeom.Cube.Define(stage, path + "/Ramp")
        ramp.CreateSizeAttr(1)
        slope = math.atan2(nose_h, bevel)
        thickness = nose_h / 2
        ramp.AddTranslateOp().Set(
            Gf.Vec3d(
                0, half_l - bevel / 2 - thickness / 2 * math.sin(slope), -thickness / 2 * math.cos(slope)
            )
        )
        ramp.AddRotateXOp().Set(-math.degrees(slope))
        ramp.AddScaleOp().Set(Gf.Vec3f(nose_w, math.hypot(bevel, nose_h), thickness))
        ramp.CreateDisplayColorAttr([Gf.Vec3f(0.75, 0.57, 0.16)])
        UsdPhysics.CollisionAPI.Apply(ramp.GetPrim())
        collision = PhysxSchema.PhysxCollisionAPI.Apply(ramp.GetPrim())
        collision.CreateContactOffsetAttr(cfg["contact_offset_m"])
        collision.CreateRestOffsetAttr(cfg["rest_offset_m"])
        UsdShade.MaterialBindingAPI.Apply(ramp.GetPrim()).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
        paths.append(str(ramp.GetPath()))
        UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(["/World/Slot/floor"])
        # Disable gravity for spring reference; avoids treating self-weight as
        # unspecified preload. Nose shape remains a box in this first revision.
        PhysxSchema.PhysxRigidBodyAPI(prim).CreateDisableGravityAttr(True)
        j = joint(stage, f"/World/SocketJoints/contact_{i:02}", UsdPhysics.PrismaticJoint, None, path, center)
        j.CreateAxisAttr("Z")
        j.CreateLowerLimitAttr(-cfg["assumed_contact_max_deflection_m"])
        j.CreateUpperLimitAttr(0)
        drive(j, "linear", cfg["assumed_contact_spring_N_m"], cfg["assumed_contact_damping_N_s_m"], 0, 1)
        moving.append(path)
    return (
        paths,
        moving,
        {
            "gap_m": gap,
            "width_m": width,
            "mouth_y_m": mouth,
            "length_m": length,
            "center_x_m": lateral,
            "center_z_m": z,
            "lead_in_m": lead,
            "lead_in_expansion_each_side_m": expansion,
            "contact_count": count,
            "contact_depth_m": depth,
            "status": cfg["scope"],
            "contact_shape": "spring-loaded analytic ramp and land; assumed bevel dimensions",
        },
    )
