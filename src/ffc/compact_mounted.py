"""Compact candidate as an FR3 payload with physical jaw joints.

Guide contacts internal to the purchased actuator are represented by ideal
prismatic joints. External supplier, finger, pad and adapter surfaces collide.
"""

import hashlib
from pathlib import Path

import numpy as np
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

from ffc.isaac_scene import drive, joint, pose


def mount(stage, project: Path, link_path, local, world_pose, closing=0.0045):
    cad = project / "outputs/compact-fingers-002/compact-fingers.usda"
    adapter = project / "outputs/compact-mount-001/adapter.usda"
    bodies, pad_paths, supplier = {}, [], {}
    rotation = Gf.Quatf(Gf.Matrix3d(*world_pose[:3, :3].T.flatten().tolist()).ExtractRotation().GetQuat())
    for name, mass in [("fixed", 0.16), ("lower", 0.02), ("upper", 0.02)]:
        offset = closing if name == "lower" else (-closing if name == "upper" else 0)
        path = "/World/Tool/" + name
        body = UsdGeom.Xform.Define(stage, path).GetPrim()
        position = world_pose[:3, 3] + world_pose[:3, :3] @ [0, 0, offset]
        pose(body, position.tolist(), rotation)
        UsdPhysics.RigidBodyAPI.Apply(body)
        inertia = UsdPhysics.MassAPI.Apply(body)
        inertia.CreateMassAttr(mass)
        inertia.CreateCenterOfMassAttr(Gf.Vec3f(0, -0.03 if name == "fixed" else -0.004, 0))
        inertia.CreateDiagonalInertiaAttr(Gf.Vec3f(0.00004 if name == "fixed" else 0.000002))
        PhysxSchema.PhysxRigidBodyAPI.Apply(body).CreateEnableCCDAttr(True)
        visual = UsdGeom.Xform.Define(stage, path + "/Visual")
        visual.AddTranslateOp().Set(Gf.Vec3d(-0.0102985785382651, 0.2482103195204206, -0.0143601205939444))
        group = stage.DefinePrim(path + "/Visual/CAD")
        group.GetReferences().AddReference(str(cad), "/CompactTool/" + name)
        bodies[name] = path
        supplier[name] = []
        for mesh in Usd.PrimRange(group):
            if not mesh.IsA(UsdGeom.Mesh) or mesh.GetName().endswith("_envelope"):
                continue
            UsdPhysics.CollisionAPI.Apply(mesh)
            UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("convexHull")
            contact = PhysxSchema.PhysxCollisionAPI.Apply(mesh)
            contact.CreateContactOffsetAttr(0.00002)
            contact.CreateRestOffsetAttr(0)
            if mesh.GetName().endswith("_pad"):
                pad_paths.append(str(mesh.GetPath()))
            if mesh.GetName().startswith("supplier_"):
                supplier[name].append(mesh)
    # Actuator guide pieces interleave; convex hulls would fill their running
    # clearances. Filter only those internal guide pairs, not pads/environment.
    for left in supplier["lower"]:
        pairs = UsdPhysics.FilteredPairsAPI.Apply(left).CreateFilteredPairsRel()
        pairs.SetTargets([p.GetPath() for p in supplier["upper"]])
    bracket = stage.DefinePrim(bodies["fixed"] + "/Adapter")
    bracket.GetReferences().AddReference(str(adapter))
    for mesh in Usd.PrimRange(bracket):
        if mesh.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(mesh)
            UsdPhysics.MeshCollisionAPI.Apply(mesh).CreateApproximationAttr("convexHull")
    attach = joint(
        stage, "/World/Tool/Attach", UsdPhysics.FixedJoint, link_path, bodies["fixed"], tuple(local[:3, 3])
    )
    attach.GetLocalRot0Attr().Set(
        Gf.Quatf(Gf.Matrix3d(*local[:3, :3].T.flatten().tolist()).ExtractRotation().GetQuat())
    )
    for name, sign in [("lower", 1), ("upper", -1)]:
        jaw = joint(
            stage, "/World/Tool/" + name + "_travel", UsdPhysics.PrismaticJoint, bodies["fixed"], bodies[name]
        )
        jaw.CreateAxisAttr("Z")
        jaw.CreateLowerLimitAttr(0 if sign == 1 else -0.005)
        jaw.CreateUpperLimitAttr(0.005 if sign == 1 else 0)
        drive(jaw, "linear", 10000, 2, sign * closing, 0.8)
    material = UsdShade.Material.Define(stage, "/World/PadMaterial")
    physics = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics.CreateStaticFrictionAttr(0.5)
    physics.CreateDynamicFrictionAttr(0.4)
    physics.CreateRestitutionAttr(0)
    for path in pad_paths:
        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            material, UsdShade.Tokens.weakerThanDescendants, "physics"
        )
    if len(pad_paths) != 2:
        raise ValueError("Expected two actual CAD pad surfaces")
    return pad_paths, {
        "cad_sha256": hashlib.sha256(cad.read_bytes()).hexdigest(),
        "adapter_sha256": hashlib.sha256(adapter.read_bytes()).hexdigest(),
        "tool_local_link7": np.asarray(local).tolist(),
        "mass_model": "Assumed 160 g fixed body/adapter and 20 g per jaw; approximate inertia",
        "external_collision": "Per-component convex hulls; pads included, screw clearance envelopes excluded",
        "internal_collision_filter": "Connected guide bodies and lower/upper supplier guide pieces only",
    }
