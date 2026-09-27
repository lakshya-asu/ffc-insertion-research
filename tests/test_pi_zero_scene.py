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
