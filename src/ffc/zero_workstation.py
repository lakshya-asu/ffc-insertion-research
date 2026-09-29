"""Pi Zero CAD and socket reference registration for the controlled cell.

Scene authoring only. The nominal registration is not a perception observation.
"""

import json

import numpy as np
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics

from ffc.isaac_scene import box, pose
from ffc.socket_reference import build_socket


def add_workstation(stage, root):
    mouth = np.array([0.478, 0.0105, 0.290])
    board_translation = mouth - [0.0333, -0.0008, 0.0022]
    path = "/World/Zero2W"
    board = UsdGeom.Xform.Define(stage, path)
    board.GetPrim().GetReferences().AddReference(str(root / "third_party/raspberry_pi/zero/zero2w.usdc"))
    board.AddTranslateOp().Set(Gf.Vec3d(*board_translation))
    # Replace the community socket mesh with the separately parameterized model.
    old = stage.GetPrimAtPath(path + "/Components/Part_0010")
    if not old:
        raise ValueError("Expected community connector component absent")
    old.SetActive(False)
    for prim in Usd.PrimRange(board.GetPrim()):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateMinThicknessAttr(0.00001)
            api = PhysxSchema.PhysxCollisionAPI.Apply(prim)
            api.CreateContactOffsetAttr(0.00002)
            api.CreateRestOffsetAttr(0)
    cfg = json.loads((root / "config/connectors/zero-reference-contact-v1.json").read_text())
    evidence = json.loads((root / "config/connectors/pi-socket-evidence-v1.json").read_text())
    colliders, moving, geometry = build_socket(stage, cfg, evidence, 0.055)
    rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), -90).GetQuat()
    r = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    translation = mouth - r @ np.array([0, cfg["mouth_y_m"], 0.055])
    for prim in stage.GetPrimAtPath("/World/Slot").GetChildren():
        xf = UsdGeom.Xformable(prim)
        matrix = xf.GetLocalTransformation()
        point = r @ np.asarray(matrix.ExtractTranslation()) + translation
        orientation = Gf.Quatf(rotation * matrix.ExtractRotationQuat())
        pose(prim, point.tolist(), orientation)
    for prim in stage.GetPrimAtPath("/World/SocketJoints").GetChildren():
        j = UsdPhysics.Joint(prim)
        j.GetLocalPos0Attr().Set(Gf.Vec3f(*(r @ np.asarray(j.GetLocalPos0Attr().Get()) + translation)))
        j.GetLocalRot0Attr().Set(Gf.Quatf(rotation))
    box(
        stage,
        "/World/ZeroFixture",
        (board_translation[0], board_translation[1], 0.2239),
        (0.06, 0.026, 0.1278),
        (0.16, 0.18, 0.2),
    )
    return {
        "board_translation_m": board_translation.tolist(),
        "nominal_mouth_m": mouth.tolist(),
        "colliders": colliders,
        "moving_contacts": moving,
        "socket_geometry": geometry,
        "scope": "Authoring registration, community CAD and reference socket; calibration unqualified",
    }
