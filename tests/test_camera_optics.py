import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom

from ffc.camera_optics import apply_reference_optics

PROFILE = json.loads((Path(__file__).parents[1] / "config/arducam-b0498.json").read_text())


@pytest.mark.parametrize("meters_per_unit", [1.0, 0.01])
def test_physical_units_and_projected_field(meters_per_unit):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, meters_per_unit)
    cam = UsdGeom.Camera.Define(stage, "/Camera")
    result = apply_reference_optics(stage, cam, PROFILE, 0.32)
    physical_focal_m = cam.GetFocalLengthAttr().Get() * 0.1 * meters_per_unit
    assert physical_focal_m == pytest.approx(0.016)
    assert cam.GetFocusDistanceAttr().Get() * meters_per_unit == pytest.approx(0.32)
    # Independently read USD's projection window at the authored focus distance.
    frustum = cam.GetCamera().frustum
    corners = frustum.ComputeCornersAtDistance(0.32 / meters_per_unit)
    width_m = (corners[1] - corners[0]).GetLength() * meters_per_unit
    assert width_m == pytest.approx(0.22272, rel=1e-5)
    assert result["K"][0][0] == pytest.approx(5517.241379)
    assert cam.GetFStopAttr().Get() == 0


def test_preview_preserves_field_and_rejects_stretched_sensor():
    stage = Usd.Stage.CreateInMemory()
    cam = UsdGeom.Camera.Define(stage, "/Camera")
    native = apply_reference_optics(stage, cam, PROFILE, 0.32)
    preview = apply_reference_optics(stage, cam, PROFILE, 0.32, (960, 540))
    assert preview["horizontal_fov_mm"] == native["horizontal_fov_mm"]
    assert preview["K"][0][0] == native["K"][0][0] / 4
    with pytest.raises(ValueError, match="aspect"):
        apply_reference_optics(stage, cam, PROFILE, 0.32, (960, 640))
