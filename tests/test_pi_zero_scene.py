import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

from ffc.pi_zero_scene import make_zero_cable, set_probe

CFG = json.loads((Path(__file__).parents[1] / "config/pi-zero-task.json").read_text())


def test_asymmetric_cable_has_correct_end_widths_pitch_and_face():
    stage = Usd.Stage.CreateInMemory()
    make_zero_cable(stage, CFG)
    path = "/World/Hardware/ZeroCable"
    points = np.array(UsdGeom.Mesh(stage.GetPrimAtPath(path + "/Body")).GetPointsAttr().Get())
    assert np.isclose(np.ptp(points[:, 0]), 0.2)
    assert np.isclose(np.ptp(points[:4, 1]), 0.0115)
    assert np.isclose(np.ptp(points[-4:, 1]), 0.016)
    for end, count, pitch, sign in [("Mini", 22, 0.0005, -1), ("Standard", 15, 0.001, 1)]:
        pins = list(stage.GetPrimAtPath(path + "/" + end + "Contacts").GetChildren())
        assert len(pins) == count
        locations = np.array(
            [UsdGeom.Xformable(p).GetLocalTransformation().ExtractTranslation() for p in pins]
        )
        np.testing.assert_allclose(np.diff(locations[:, 1]), pitch, atol=1e-9)
        assert np.all(locations[:, 2] * sign > 0)
    assert not any("Physics" in str(p.GetAppliedSchemas()) for p in stage.Traverse())


def test_sideways_probes_keep_height_and_stay_outside_mouth():
    stage = Usd.Stage.CreateInMemory()
    make_zero_cable(stage, CFG)
    a = set_probe(stage, CFG, "approach")
    b = set_probe(stage, CFG, "near")
    movement = np.array(b["tip_m"]) - a["tip_m"]
    np.testing.assert_allclose(movement, [-0.012, 0, 0], atol=1e-10)
    assert np.dot(movement, CFG["board_normal_world"]) == 0
    assert b["tip_m"][0] > CFG["nominal_mouth_review_point_m"][0]
    np.testing.assert_allclose(
        UsdGeom.Xformable(stage.GetPrimAtPath("/World/Hardware/ZeroCable"))
        .GetLocalTransformation()
        .ExtractTranslation(),
        b["tip_m"],
    )


def test_pickup_camera_contains_both_loose_cable_ends():
    from pxr import Gf

    from ffc.camera_optics import apply_reference_optics

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    cam = UsdGeom.Camera.Define(stage, "/Camera")
    view = next(v for v in CFG["cameras"] if v["id"] == "overview")
    eye, target = np.array(view["eye_m"]), np.array(view["target_m"])
    cam.AddTransformOp().Set(
        Gf.Matrix4d().SetLookAt(Gf.Vec3d(*eye), Gf.Vec3d(*target), Gf.Vec3d(0, 0, 1)).GetInverse()
    )
    profile = json.loads((Path(__file__).parents[1] / "config/arducam-b0498.json").read_text())
    apply_reference_optics(stage, cam, profile, float(np.linalg.norm(eye - target)))
    frustum = cam.GetCamera().frustum
    transform = frustum.ComputeViewMatrix() * frustum.ComputeProjectionMatrix()
    # Include the wider far-end corners, with a five-percent half-frame margin.
    for x in [0.34, 0.54]:
        for y in [-0.238, -0.222]:
            projected = transform.Transform(Gf.Vec3d(x, y, 0.0003))
            assert abs(projected[0]) < 0.95 and abs(projected[1]) < 0.95
