"""Geometry checks that catch invisible sliders and accidentally filled apertures."""

import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

from ffc.entrance_features import make_socket, set_slider

ROOT = Path(__file__).resolve().parents[1]


def test_front_rims_leave_declared_opening_and_slider_is_exposed():
    spec = json.loads((ROOT / "config/zero-feature-v1.json").read_text())
    stage = Usd.Stage.CreateInMemory()
    make_socket(stage, "/Socket", spec)
    upper = np.array(UsdGeom.Mesh(stage.GetPrimAtPath("/Socket/Upper/Front")).GetPointsAttr().Get())
    lower = np.array(UsdGeom.Mesh(stage.GetPrimAtPath("/Socket/Lower/Front")).GetPointsAttr().Get())
    assert np.isclose(upper[:, 2].min() - lower[:, 2].max(), 0.0005)
    # Both positions must be visible above the housing, not buried in its solid roof.
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default"])
    for opened in [False, True]:
        set_slider(stage, "/Socket", spec, opened)
        cache.Clear()
        bounds = cache.ComputeWorldBound(stage.GetPrimAtPath("/Socket/Slider")).ComputeAlignedRange()
        assert bounds.GetMin()[2] >= upper[:, 2].max() - 1e-9
        assert bounds.GetMax()[2] <= 0.000600001
    # Rim planes are actual boundaries of solid boxes; front labels aren't a floating marker.
    for name in ["Upper", "Lower"]:
        front = UsdGeom.Mesh(stage.GetPrimAtPath(f"/Socket/{name}/Front"))
        ids = np.array(front.GetFaceVertexIndicesAttr().Get())
        vertices = np.array(front.GetPointsAttr().Get())[ids]
        assert np.allclose(vertices[:, 0], 0)
        assert len(front.GetFaceVertexCountsAttr().Get()) == 1
