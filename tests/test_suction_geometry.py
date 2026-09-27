"""Vacuum anchors must pull an actual cable surface through the capture gap."""

import numpy as np
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from ffc.isaac_suction import attach


def test_capture_gap_does_not_become_a_floating_body_anchor():
    stage = Usd.Stage.CreateInMemory()
    body = UsdGeom.Xform.Define(stage, "/World/Tool/Body")
    rotation = Rotation.from_matrix([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]).as_quat()
    body.AddOrientOp().Set(Gf.Quatf(float(rotation[3]), Gf.Vec3f(*rotation[:3])))
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    for index, x in enumerate((0.081, 0.086)):
        cable = UsdGeom.Xform.Define(stage, f"/World/Cable/segment_{index:03d}")
        cable.AddTranslateOp().Set(Gf.Vec3d(x, 0, -0.12035))
        UsdPhysics.RigidBodyAPI.Apply(cable.GetPrim())
        shape = UsdGeom.Cube.Define(stage, str(cable.GetPath()) + "/Shape")
        shape.CreateSizeAttr(1)
        shape.AddScaleOp().Set(Gf.Vec3f(0.005, 0.008, 0.0003))
    joints = attach(stage, "/World/Cable/segment_001")
    cache = UsdGeom.XformCache()
    for path in joints:
        joint = UsdPhysics.Joint(stage.GetPrimAtPath(path))
        local_anchor = joint.GetLocalPos1Attr().Get()
        assert np.isclose(local_anchor[2], 0.00015, atol=1e-9)
        endpoints = []
        for relation, local in (
            (joint.GetBody0Rel(), joint.GetLocalPos0Attr()),
            (joint.GetBody1Rel(), joint.GetLocalPos1Attr()),
        ):
            endpoints.append(
                cache.GetLocalToWorldTransform(stage.GetPrimAtPath(relation.GetTargets()[0])).Transform(
                    Gf.Vec3d(local.Get())
                )
            )
        assert np.isclose((endpoints[0] - endpoints[1]).GetLength(), 0.0002, atol=1e-8)
