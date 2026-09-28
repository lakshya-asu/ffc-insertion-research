import json
from pathlib import Path

import pytest
from pxr import Usd, UsdGeom

from ffc.macro_camera import add_macro_camera, optical_budget

ROOT = Path(__file__).resolve().parents[1]


def test_macro_projection_matches_published_magnification():
    profile = json.loads((ROOT / "config/macro-basler-kowa.json").read_text())
    budget = optical_budget(profile)
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    cam, _, report = add_macro_camera(stage, profile, [0.6333, -0.1408, 0.0322])
    # USD lens/aperture quantities are tenths of a scene unit, not millimetres.
    fov = (
        cam.GetHorizontalApertureAttr().Get()
        / cam.GetFocalLengthAttr().Get()
        * cam.GetFocusDistanceAttr().Get()
        * 1000
    )
    assert fov == pytest.approx(17.6513684, rel=1e-6)
    assert budget["object_sampling_um_per_pixel"] == pytest.approx(7.2105263)
    assert budget["approx_total_dof_one_pixel_coc_mm"] < 0.5
    assert report["lens_front_m"][2] > 0.05
    assert profile["runtime"]["motion_permitted"] is False
    assert profile["runtime"]["model_compatible"] is False
