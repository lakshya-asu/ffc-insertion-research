"""Dimension and face-orientation contracts for the new cable asset."""

import json
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

from ffc.raspberry_pi_scene import cable_centerline, make_camera_cable

ROOT = Path(__file__).resolve().parents[1]


def make_scene():
    stage = Usd.Stage.CreateInMemory()
    spec = json.loads((ROOT / "config/raspberry-pi-task.json").read_text())
    result = make_camera_cable(stage, spec)
    return stage, result


def points(stage, name):
    return np.asarray(UsdGeom.Mesh.Get(stage, "/World/Hardware/CameraCable/" + name).GetPointsAttr().Get())


def test_cable_arc_length_is_200_mm_not_projected_span():
    center = cable_centerline()
    arc = np.linalg.norm(np.diff(center, axis=0), axis=1).sum()
    assert abs(arc - 0.2) < 1e-10
    assert center[-1, 0] - center[0, 0] < 0.2


def test_metre_scale_width_and_fifteen_contact_pitch():
    stage, result = make_scene()
    body = points(stage, "Body")
    assert abs(np.ptp(body[:, 1]) - 0.016) < 1e-7
    y = [points(stage, f"ContactsA/Pin_{i:02d}")[:, 1].mean() for i in range(1, 16)]
    np.testing.assert_allclose(np.diff(y), 0.001, atol=1e-7)
    assert result["contacts_per_end"] == 15
    assert not stage.GetPrimAtPath("/World/Hardware/CameraCable/ContactsA/Pin_16")


def test_support_and_contact_faces_are_opposite():
    stage, _ = make_scene()
    # Endpoint cross-sections are nearly horizontal; compare local normal offsets.
    a = points(stage, "ContactsA/Pin_08")[:4, 2]
    a_support = points(stage, "TipASupport")[:4, 2]
    b = points(stage, "ContactsB/Pin_08")[-4:, 2]
    b_support = points(stage, "TipBSupport")[-4:, 2]
    assert a_support.max() < a.min()
    assert b_support.min() > b.max()
    assert abs(a.max() - a_support.min() - 0.000309) < 1e-7
    assert abs(b_support.max() - b.min() - 0.000309) < 1e-7


def test_randomized_bows_preserve_length_and_face_flip_keeps_cable_above_desk():
    for height, lateral in [(0.001, -0.012), (0.014, 0.025)]:
        center = cable_centerline(height_m=height, lateral_m=lateral)
        assert abs(np.linalg.norm(np.diff(center, axis=0), axis=1).sum() - 0.2) < 1e-10
        stage = Usd.Stage.CreateInMemory()
        spec = json.loads((ROOT / "config/raspberry-pi-task.json").read_text())
        spec["cable"].update(face_a_up=False, curve_height_m=height, lateral_bow_m=lateral)
        make_camera_cable(stage, spec)
        for prim in stage.Traverse():
            if prim.IsA(UsdGeom.Mesh):
                assert np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())[:, 2].min() > 0
        assert points(stage, "ContactsA/Pin_08")[:4, 2].max() < points(stage, "TipASupport")[:4, 2].min()
        assert points(stage, "ContactsB/Pin_08")[-4:, 2].min() > points(stage, "TipBSupport")[-4:, 2].max()
