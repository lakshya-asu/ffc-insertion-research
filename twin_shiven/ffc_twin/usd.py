"""Emit the same scene as USD for Isaac Lab: connector boxes, a tool articulation with six drives,
the rigid header and the tail as a reduced-coordinate articulation chain.

Conventions follow Lakshya's isaac_scene.py: an unscaled Xform per part with a unit Cube child
scaled to size, metres, Z up, provenance in the layer's customLayerData. Chamfers are rotated boxes.
The scene is a drop-in sublayer candidate; it does not include the FR3.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

from .spec import DEFAULT, Spec


def _physx(prim, api: str, attrs: dict):
    """Author PhysX attributes without the PhysxSchema plugin (usd-core only): add the API name and the attributes by name.
    Isaac Sim resolves these against its registered schemas on load."""
    from pxr import Sdf

    prim.AddAppliedSchema(api)
    for name, (value, vtype) in attrs.items():
        prim.CreateAttribute(name, vtype).Set(value)


def _box(stage, path, pos, size, color, rot_deg=(0, 0, 0), collision=True, mass=None):
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    root = UsdGeom.Xform.Define(stage, path).GetPrim()
    x = UsdGeom.Xformable(root)
    x.ClearXformOpOrder()
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    if any(rot_deg):
        x.AddRotateXYZOp().Set(Gf.Vec3f(*rot_deg))
    shape = UsdGeom.Cube.Define(stage, path + "/Shape")
    shape.CreateSizeAttr(1)
    shape.AddScaleOp().Set(Gf.Vec3f(*size))
    shape.CreateDisplayColorAttr([Gf.Vec3f(*color)])
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        _physx(shape.GetPrim(), "PhysxCollisionAPI", {"physxCollision:contactOffset": (0.00002, Sdf.ValueTypeNames.Float), "physxCollision:restOffset": (0.0, Sdf.ValueTypeNames.Float)})
    if mass is not None:
        UsdPhysics.RigidBodyAPI.Apply(root)
        m = UsdPhysics.MassAPI.Apply(root)
        m.CreateMassAttr(mass)
        a, b, c = size
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(mass * (b * b + c * c) / 12, mass * (a * a + c * c) / 12, mass * (a * a + b * b) / 12))
    return root


def _joint(stage, path, kind, body0, body1, pos0, pos1, axis=None, lower=None, upper=None, stiffness=None, damping=None, max_force=None):
    from pxr import Gf, UsdPhysics

    j = kind.Define(stage, path)
    if body0:
        j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateLocalPos0Attr(Gf.Vec3f(*pos0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*pos1))
    j.CreateLocalRot0Attr(Gf.Quatf(1))
    j.CreateLocalRot1Attr(Gf.Quatf(1))
    j.CreateCollisionEnabledAttr(False)
    if axis:
        j.CreateAxisAttr(axis)
    if lower is not None:
        j.CreateLowerLimitAttr(lower)
        j.CreateUpperLimitAttr(upper)
    if stiffness is not None:
        drive_kind = "angular" if kind is UsdPhysics.RevoluteJoint else "linear"
        d = UsdPhysics.DriveAPI.Apply(j.GetPrim(), drive_kind)
        d.CreateTypeAttr("force")
        d.CreateStiffnessAttr(stiffness)
        d.CreateDampingAttr(damping)
        d.CreateTargetPositionAttr(0.0)
        d.CreateMaxForceAttr(max_force)
    return j


def build_usd(path: str | Path, spec: Spec = DEFAULT) -> str:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

    path = str(path)
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    stage.SetMetadata("customLayerData", {"ffc_twin_spec": json.loads(spec.to_json())})
    scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    g = spec.physics.gravity_in_C; gm = math.sqrt(sum(v * v for v in g))
    scene.CreateGravityDirectionAttr(Gf.Vec3f(*(v / gm for v in g)))
    scene.CreateGravityMagnitudeAttr(gm)
    _physx(scene.GetPrim(), "PhysxSceneAPI", {"physxScene:solverType": ("TGS", Sdf.ValueTypeNames.Token),
                                              "physxScene:timeStepsPerSecond": (120, Sdf.ValueTypeNames.UInt),  # Factory-task recipe
                                              "physxScene:enableCCD": (True, Sdf.ValueTypeNames.Bool)})

    c, k, t = spec.cable, spec.connector, spec.tool
    W, T_HDR, T_BODY = c.width.value, c.header_thickness.value, c.body_thickness.value
    hdr = c.header_length
    slot_w, slot_h, slot_d, wall = k.opening_width.value, k.slot_height.value, k.seat_depth.value, k.wall_thickness.value
    housing = (0.86, 0.84, 0.78)
    # connector, fixed
    _box(stage, "/World/Connector/WallL", (slot_d / 2, slot_w / 2 + wall / 2, 0), (slot_d, wall, slot_h + 2 * wall), housing)
    _box(stage, "/World/Connector/WallR", (slot_d / 2, -(slot_w / 2 + wall / 2), 0), (slot_d, wall, slot_h + 2 * wall), housing)
    _box(stage, "/World/Connector/Roof", (slot_d / 2, 0, slot_h / 2 + wall / 2), (slot_d, slot_w + 2 * wall, wall), housing)
    _box(stage, "/World/Connector/Floor", (slot_d / 2, 0, -(slot_h / 2 + wall / 2)), (slot_d, slot_w + 2 * wall, wall), housing)
    _box(stage, "/World/Connector/Backstop", (slot_d + wall / 2, 0, 0), (wall, slot_w + 2 * wall, slot_h + 2 * wall), (0.45, 0.43, 0.38))
    ch, a, ch_t = k.top_leadin_half_length.value, k.top_leadin_angle.value, 0.05e-3
    ch_cx = ch * math.cos(a) - ch_t * math.sin(a)
    ch_cz = slot_h / 2 + ch * math.sin(a) + ch_t * math.cos(a)
    _box(stage, "/World/Connector/LeadInTop", (-ch_cx, 0, ch_cz), (2 * ch, slot_w + 2 * wall, 2 * ch_t), housing, rot_deg=(0, math.degrees(a), 0))
    _box(stage, "/World/Connector/LeadInBottom", (-ch_cx, 0, -ch_cz), (2 * ch, slot_w + 2 * wall, 2 * ch_t), housing, rot_deg=(0, -math.degrees(a), 0))
    side = k.side_leadin_half_length.value
    if side > 0:
        s_a = k.side_leadin_angle.value
        cx = side * math.cos(s_a) - ch_t * math.sin(s_a)
        cy = slot_w / 2 + side * math.sin(s_a) + ch_t * math.cos(s_a)
        _box(stage, "/World/Connector/LeadInSideL", (-cx, cy, 0), (2 * side, 2 * ch_t, slot_h + 2 * wall), housing, rot_deg=(0, 0, math.degrees(s_a)))
        _box(stage, "/World/Connector/LeadInSideR", (-cx, -cy, 0), (2 * side, 2 * ch_t, slot_h + 2 * wall), housing, rot_deg=(0, 0, -math.degrees(s_a)))
    # spring-supported contact noses, same layout as the MJCF: land + ramp fixed together, on a Z prismatic joint to the world
    if k.contact_count:
        n_l, n_w, n_h, bev = k.contact_nose_length.value, k.contact_nose_width.value, k.contact_nose_height.value, k.contact_bevel_length.value
        top = k.contact_rest_top_rel_center.value
        slope = math.atan2(n_h, bev); ramp_len = math.hypot(bev, n_h); r_t = n_h / 2
        r_cx = -n_l / 2 + bev / 2 - (r_t / 2) * math.sin(slope)
        r_cz = top - n_h / 2 - (r_t / 2) * math.cos(slope)
        gold = (0.75, 0.57, 0.16)
        for i in range(k.contact_count):
            y = (i - (k.contact_count - 1) / 2) * k.contact_pitch.value
            x0 = k.contact_depth_from_mouth.value
            land = _box(stage, f"/World/Connector/Contact_{i:02d}", (x0 + bev / 2, y, top - n_h / 2), (n_l - bev, n_w, n_h), gold, mass=k.contact_mass.value)
            ramp = _box(stage, f"/World/Connector/ContactRamp_{i:02d}", (x0 + r_cx, y, r_cz), (ramp_len, n_w, r_t), gold, rot_deg=(0, -math.degrees(slope), 0), mass=1e-9)
            _joint(stage, f"/World/Connector/contact_{i:02d}_z", UsdPhysics.PrismaticJoint, None, str(land.GetPath()), (x0 + bev / 2, y, top - n_h / 2), (0, 0, 0),
                   "Z", -k.contact_max_deflection.value, 0.0, k.contact_spring.value, k.contact_damping.value, 1.0)
            _joint(stage, f"/World/Connector/contact_{i:02d}_ramp", UsdPhysics.FixedJoint, str(land.GetPath()), str(ramp.GetPath()), (r_cx - bev / 2, 0, r_cz - (top - n_h / 2)), (0, 0, 0))

    # tool articulation: a fixed base, then six single-axis links (x, y, z slides; rx, ry, rz hinges), then the header
    film = t.free_film_length.value if t.grasp_on == "film" else 0.0
    jaw_len = t.jaw_length.value
    tool_x = (-(hdr - jaw_len / 2) if t.grasp_on == "tape" else -hdr - film - jaw_len / 2) - spec.success.start_gap.value
    base = UsdGeom.Xform.Define(stage, "/World/Tool").GetPrim()
    UsdPhysics.ArticulationRootAPI.Apply(base)
    _physx(base, "PhysxArticulationAPI", {"physxArticulation:solverPositionIterationCount": (192, Sdf.ValueTypeNames.Int),
                                          "physxArticulation:solverVelocityIterationCount": (1, Sdf.ValueTypeNames.Int)})
    anchor = _box(stage, "/World/Tool/Anchor", (tool_x, 0, 0), (0.002, 0.002, 0.002), (0.3, 0.3, 0.3), collision=False, mass=0.01)
    _joint(stage, "/World/Tool/fixed_base", UsdPhysics.FixedJoint, None, str(anchor.GetPath()), (0, 0, 0), (0, 0, 0))
    prev = str(anchor.GetPath())
    for name, kind, axis, kp, kv, fmax in (
        ("tx", UsdPhysics.PrismaticJoint, "X", t.position_kp.value, t.position_kv.value, 50.0),
        ("ty", UsdPhysics.PrismaticJoint, "Y", t.position_kp.value, t.position_kv.value, 50.0),
        ("tz", UsdPhysics.PrismaticJoint, "Z", t.position_kp.value, t.position_kv.value, 50.0),
        ("rx", UsdPhysics.RevoluteJoint, "X", t.rotation_kp.value, t.rotation_kv.value, 1.0),
        ("ry", UsdPhysics.RevoluteJoint, "Y", t.rotation_kp.value, t.rotation_kv.value, 1.0),
        ("rz", UsdPhysics.RevoluteJoint, "Z", t.rotation_kp.value, t.rotation_kv.value, 1.0),
    ):
        link = _box(stage, f"/World/Tool/Link_{name}", (tool_x, 0, 0), (0.002, 0.002, 0.002), (0.3, 0.32, 0.36), collision=False, mass=0.005)
        limits = (-0.02, 0.02) if kind is UsdPhysics.PrismaticJoint else (-30.0, 30.0)
        stiffness = kp if kind is UsdPhysics.PrismaticJoint else math.degrees(1) * kp  # USD angular drives are per degree
        damping = kv if kind is UsdPhysics.PrismaticJoint else math.degrees(1) * kv
        _joint(stage, f"/World/Tool/{name}", kind, prev, str(link.GetPath()), (0, 0, 0), (0, 0, 0), axis, *limits, stiffness, damping, fmax)
        prev = str(link.GetPath())
    for sign, name in ((1, "JawTop"), (-1, "JawBottom")):
        _box(stage, f"/World/Tool/{name}", (tool_x, 0, sign * (T_BODY / 2 + 0.0006)), (jaw_len, 0.012, 0.001), (0.30, 0.32, 0.36), collision=False, mass=0.025)
        _joint(stage, f"/World/Tool/{name}_fixed", UsdPhysics.FixedJoint, prev, f"/World/Tool/{name}", (0, 0, sign * (T_BODY / 2 + 0.0006)), (0, 0, 0))
    # grasp: the gripped film may slide along x in the jaws against dry friction 2 mu N (PhysX joint friction, authored by name)
    gripped = _box(stage, "/World/Cable/Gripped", (tool_x, 0, 0), (jaw_len, W, T_BODY), (0.16, 0.17, 0.19), collision=False, mass=c.mass_per_length.value * jaw_len)
    slip = _joint(stage, "/World/Cable/grip_slip", UsdPhysics.PrismaticJoint, prev, str(gripped.GetPath()), (0, 0, 0), (0, 0, 0), "X", -0.01, 0.01)
    _physx(slip.GetPrim(), "PhysxJointAPI", {"physxJoint:jointFriction": (2 * t.pad_friction.value * t.clamp_force.value, Sdf.ValueTypeNames.Float)})
    if t.grasp_on == "film":
        # free film between the jaws and the tape: bend then twist
        film_x = tool_x + jaw_len / 2 + film / 2
        kb = math.degrees(1) * c.bend_stiffness_per_hinge() * c.tail_link_length.value / film
        kt = math.degrees(1) * c.twist_stiffness_per_hinge() * c.tail_link_length.value / film
        pivot = _box(stage, "/World/Cable/FilmPivot", (tool_x + jaw_len / 2, 0, 0), (0.0005, 0.0005, 0.0005), (0.5, 0.5, 0.5), collision=False, mass=1e-6)
        _joint(stage, "/World/Cable/film_bend", UsdPhysics.RevoluteJoint, str(gripped.GetPath()), str(pivot.GetPath()), (jaw_len / 2, 0, 0), (0, 0, 0), "Y", -60.0, 60.0, kb, 0.02 * kb, 0.5)
        free_film = _box(stage, "/World/Cable/FreeFilm", (film_x, 0, 0), (film, W, T_BODY), (0.16, 0.17, 0.19), mass=c.mass_per_length.value * film)
        _joint(stage, "/World/Cable/film_twist", UsdPhysics.RevoluteJoint, str(pivot.GetPath()), str(free_film.GetPath()), (0, 0, 0), (-film / 2, 0, 0), "X", -60.0, 60.0, kt, 0.02 * kt, 0.5)
        header_x = film_x + film / 2 + hdr / 2
        header = _box(stage, "/World/Cable/Header", (header_x, 0, 0), (hdr, W, T_HDR), (0.18, 0.42, 0.85), mass=c.mass_per_length.value * hdr * 2)
        _joint(stage, "/World/Cable/tape_fixed", UsdPhysics.FixedJoint, str(free_film.GetPath()), str(header.GetPath()), (film / 2 + hdr / 2, 0, 0), (0, 0, 0))
    else:
        # tape grip: the jaws hold the rear of the stiffener; the header is fixed to the gripped body
        header_x = tool_x + hdr / 2 - jaw_len / 2
        header = _box(stage, "/World/Cable/Header", (header_x, 0, 0), (hdr, W, T_HDR), (0.18, 0.42, 0.85), mass=c.mass_per_length.value * hdr * 2)
        _joint(stage, "/World/Cable/tape_fixed", UsdPhysics.FixedJoint, str(gripped.GetPath()), str(header.GetPath()), (hdr / 2 - jaw_len / 2, 0, 0), (0, 0, 0))
    # tail chain: revolute bend (Y) then revolute twist (X) per link, articulated
    seg, n = c.tail_link_length.value, c.tail_links
    k_bend = math.degrees(1) * c.bend_stiffness_per_hinge()
    k_twist = math.degrees(1) * c.twist_stiffness_per_hinge()
    prev_path, prev_anchor_x = str(gripped.GetPath()), -jaw_len / 2
    x = tool_x - jaw_len / 2
    for i in range(n):
        x -= seg / 2
        # bend link: a massless-ish pivot body then the segment; simpler: one body with two joints is not expressible, so use a small pivot body
        pivot = _box(stage, f"/World/Cable/Pivot_{i:02d}", (x + seg / 2, 0, 0), (0.0005, 0.0005, 0.0005), (0.5, 0.5, 0.5), collision=False, mass=1e-6)
        _joint(stage, f"/World/Cable/bend_{i:02d}", UsdPhysics.RevoluteJoint, prev_path, str(pivot.GetPath()), (prev_anchor_x, 0, 0), (0, 0, 0), "Y", -60.0, 60.0, k_bend, 0.02 * k_bend, 0.5)
        link = _box(stage, f"/World/Cable/Tail_{i:02d}", (x, 0, 0), (seg, W, T_BODY), (0.16, 0.17, 0.19), mass=c.mass_per_length.value * seg)
        _joint(stage, f"/World/Cable/twist_{i:02d}", UsdPhysics.RevoluteJoint, str(pivot.GetPath()), str(link.GetPath()), (0, 0, 0), (seg / 2, 0, 0), "X", -60.0, 60.0, k_twist, 0.02 * k_twist, 0.5)
        prev_path, prev_anchor_x = str(link.GetPath()), -seg / 2
        x -= seg / 2
    stage.GetRootLayer().Save()
    return path
