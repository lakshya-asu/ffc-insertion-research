"""CAD-backed empty-tool dynamics. Assumed drives/inertia; not a grasp model.

All transforms are initialization only. The two prismatic joints become part of
FR3's articulation. Motor feedback is measured joint position/velocity in SI.
"""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics

from ffc.isaac_scene import box, drive, joint, pose


def mount(stage, cad: Path):
    manifest = json.loads((cad / "assembly.json").read_text())
    link = str(
        UsdPhysics.RevoluteJoint.Get(stage, "/World/FR3/Physics/fr3v2_1_joint7").GetBody1Rel().GetTargets()[0]
    )
    link_t = UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(link))
    # Provisional adapter top coincides with the link8 flange plane, 107 mm from link7.
    rotation = Gf.Rotation(Gf.Vec3d(1, 0, 0), 180)
    anchor = Gf.Vec3d(-0.021, -0.091, 0.152)
    translation = Gf.Vec3d(0, 0, 0.107) - rotation.TransformDir(anchor)
    local = Gf.Matrix4d(1)
    local.SetRotate(rotation)
    local.SetTranslateOnly(translation)
    global_t = local * link_t
    orientation = Gf.Quatf(global_t.ExtractRotationQuat())
    offsets = {"fixed": (0, 0, 0), "carriage": (-0.018, 0, 0), "jaw": (-0.018, 0, 0.008)}
    bodies = {}
    mass_parts = {g: [] for g in offsets}
    collisions = []
    for group, offset in offsets.items():
        path = "/World/Tool/" + group
        prim = UsdGeom.Xform.Define(stage, path).GetPrim()
        pose(prim, global_t.Transform(Gf.Vec3d(*offset)), orientation)
        UsdPhysics.RigidBodyAPI.Apply(prim)
        PhysxSchema.PhysxRigidBodyAPI.Apply(prim).CreateEnableCCDAttr(True)
        bodies[group] = path
    for part in manifest["parts"]:
        group = "carriage" if part["group"] == "deploy_rod" else part["group"]
        raw = (cad / "parts" / (part["name"] + ".stl")).read_bytes()
        count = int.from_bytes(raw[80:84], "little")
        dtype = np.dtype([("normal", "<f4", (3,)), ("v", "<f4", (3, 3)), ("attr", "<u2")])
        vertices = np.frombuffer(raw, dtype=dtype, count=count, offset=84)["v"].reshape(-1, 3) / 1000
        mesh = UsdGeom.Mesh.Define(stage, bodies[group] + "/" + part["name"])
        mesh.CreatePointsAttr(vertices.tolist())
        mesh.CreateFaceVertexCountsAttr([3] * count)
        mesh.CreateFaceVertexIndicesAttr(list(range(3 * count)))
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*part["color"])])
        if part["kind"] == "envelope":
            continue  # Visual reservation is not a force sensor or payload component.
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        collision = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        collision.CreateContactOffsetAttr(0.00002)
        collision.CreateRestOffsetAttr(0)
        collisions.append(str(mesh.GetPath()))
        bounds = np.asarray(part["bounds_mm"]) / 1000
        center, size = bounds.mean(axis=0), bounds[1] - bounds[0]
        # Published total actuator mass split is an assumption, not supplier inertia.
        if "actuator" in part["name"]:
            mass = 0.016 if "body" in part["name"] else 0.003
        else:
            density = {"metal": 2700, "printed": 1240, "soft": 1100, "purchased": 7800}[part["kind"]]
            mass = part["volume_mm3"] * 1e-9 * density
        mass_parts[group].append((mass, center, np.diag(mass / 12 * (np.sum(size * size) - size * size))))
    # Adapter is a structural envelope, not a drilled/qualified manufacturing part.
    box(
        stage,
        bodies["fixed"] + "/ProvisionalAdapter",
        (-0.021, -0.091, 0.145),
        (0.06, 0.06, 0.014),
        (0.48, 0.52, 0.56),
    )
    size = np.array([0.06, 0.06, 0.014])
    mass = float(np.prod(size) * 2700)
    mass_parts["fixed"].append(
        (mass, np.array([-0.021, -0.091, 0.145]), np.diag(mass / 12 * (np.sum(size * size) - size * size)))
    )
    inertia_report = {}
    for group, parts in mass_parts.items():
        total = sum(m for m, _, _ in parts)
        com = sum(m * c for m, c, _ in parts) / total
        tensor = sum(
            i + m * (np.dot(c - com, c - com) * np.eye(3) - np.outer(c - com, c - com)) for m, c, i in parts
        )
        values, axes = np.linalg.eigh(tensor)
        if np.linalg.det(axes) < 0:
            axes[:, 0] *= -1
        matrix = Gf.Matrix3d(*axes.T.flatten().tolist())
        api = UsdPhysics.MassAPI.Apply(stage.GetPrimAtPath(bodies[group]))
        api.CreateMassAttr(total)
        api.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        api.CreateDiagonalInertiaAttr(Gf.Vec3f(*values))
        api.CreatePrincipalAxesAttr(Gf.Quatf(matrix.ExtractRotation().GetQuat()))
        inertia_report[group] = dict(mass_kg=total, com_m=com.tolist(), inertia_kg_m2=tensor.tolist())
    attachment = joint(
        stage,
        "/World/Tool/Attach",
        UsdPhysics.FixedJoint,
        link,
        bodies["fixed"],
        (0, 0, 0.107),
        tuple(anchor),
    )
    attachment.GetLocalRot0Attr().Set(Gf.Quatf(rotation.GetQuat()))
    deployment = joint(
        stage,
        "/World/Tool/deploy",
        UsdPhysics.PrismaticJoint,
        bodies["fixed"],
        bodies["carriage"],
        offsets["carriage"],
    )
    clamp = joint(
        stage,
        "/World/Tool/clamp",
        UsdPhysics.PrismaticJoint,
        bodies["carriage"],
        bodies["jaw"],
        (0, 0, 0.008),
    )
    for j, axis, lo, hi in [(deployment, "X", 0, 0.02), (clamp, "Z", -0.02, 0)]:
        j.CreateAxisAttr(axis)
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        drive(j, "linear", 8000, 80, 0, 5)
        PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateMaxJointVelocityAttr(0.008)
    return dict(
        manifest_sha256=hashlib.sha256((cad / "assembly.json").read_bytes()).hexdigest(),
        mass_model=(
            "CAD volume x assumed material density; bounding-box part inertia; "
            "PQ12 19 g split 16/3 g; no hoses/electronics"
        ),
        payload_mass_kg=sum(x["mass_kg"] for x in inertia_report.values()),
        bodies=inertia_report,
        drive=dict(stiffness_N_m=8000, damping_N_s_m=80, max_force_N=5, max_velocity_m_s=0.008),
        collisions=(
            "Per-part convex hulls; adjacent connected-body collisions filtered by joint; "
            "no cable/contact qualification"
        ),
        collision_shapes=len(collisions) + 1,
        mount=(
            "Provisional 60 x 60 x 14 mm aluminium adapter; flange offset 107 mm; "
            "no hole/fastener/stiffness qualification"
        ),
        limits="No motor electrical, backlash, friction, duty-cycle, vacuum, tactile or pad compliance model",
    )
