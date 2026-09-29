import json
import math

import mujoco
import numpy as np
import pytest

from ffc_twin.mjcf import build_mjcf
from ffc_twin.spec import DEFAULT, LEVELS, Spec
from ffc_twin.usd import build_usd


def test_placeholders_are_listed_and_measured_values_are_not():
    ph = DEFAULT.placeholders()
    names = [p.split(" = ")[0] for p in ph]
    assert "connector.slot_height" in names
    assert "cable.modulus" in names
    assert "connector.opening_width" not in names
    assert "connector.seat_depth" not in names


def test_spec_json_round_trips_the_te_numbers():
    d = json.loads(DEFAULT.to_json())
    assert d["connector"]["opening_width"]["value"] == pytest.approx(16.40e-3)
    assert d["connector"]["seat_depth"]["value"] == pytest.approx(3.45e-3)
    assert d["cable"]["header_thickness"]["value"] == pytest.approx(0.309e-3)
    assert set(d["levels"]) == {"L0", "L1"}


def test_mjcf_loads_and_geometry_matches_spec():
    model = mujoco.MjModel.from_xml_string(build_mjcf())
    wl = model.geom("wall_l")
    wr = model.geom("wall_r")
    opening = (wl.pos[1] - wl.size[1]) - (wr.pos[1] + wr.size[1])
    assert opening == pytest.approx(DEFAULT.connector.opening_width.value, abs=1e-9)
    stop = model.geom("backstop").pos[0] - model.geom("backstop").size[0]
    assert stop == pytest.approx(DEFAULT.connector.seat_depth.value, abs=1e-9)
    hdr = model.geom("hdr")
    assert 2 * hdr.size[2] == pytest.approx(DEFAULT.cable.header_thickness.value)
    assert model.nv == 6 + 1 + 2 * DEFAULT.cable.tail_links  # tool, grip slip, tail (tape grip: no free film)


def test_mjcf_scripted_push_seats_from_zero_error():
    model = mujoco.MjModel.from_xml_string(build_mjcf())
    data = mujoco.MjData(model)
    tip = model.site("tip").id
    sub = int(round(1 / DEFAULT.physics.control_hz.value / DEFAULT.physics.timestep.value))
    peak = 0.0
    for _ in range(120):
        data.ctrl[0] += 0.2e-3
        for _ in range(sub):
            mujoco.mj_step(model, data)
        peak = max(peak, float(np.linalg.norm(data.sensordata[:3])))
        if data.site_xpos[tip][0] > DEFAULT.connector.seat_depth.value - 0.05e-3:
            break
    assert data.site_xpos[tip][0] > DEFAULT.connector.seat_depth.value - DEFAULT.success.corner_depth_tolerance.value
    assert peak < DEFAULT.success.peak_force_limit.value


def test_usd_composes_with_the_same_geometry(tmp_path):
    from pxr import Usd, UsdGeom, UsdPhysics

    path = build_usd(tmp_path / "twin.usda")
    stage = Usd.Stage.Open(path)
    assert UsdGeom.GetStageMetersPerUnit(stage) == 1
    wl = stage.GetPrimAtPath("/World/Connector/WallL/Shape")
    wr = stage.GetPrimAtPath("/World/Connector/WallR/Shape")
    pl = UsdGeom.Xformable(wl.GetParent()).GetOrderedXformOps()[0].Get()
    pr = UsdGeom.Xformable(wr.GetParent()).GetOrderedXformOps()[0].Get()
    sl = UsdGeom.Xformable(wl).GetOrderedXformOps()[0].Get()
    opening = (pl[1] - sl[1] / 2) - (pr[1] + sl[1] / 2)
    assert opening == pytest.approx(DEFAULT.connector.opening_width.value, abs=1e-9)
    assert stage.GetPrimAtPath("/World/Tool").HasAPI(UsdPhysics.ArticulationRootAPI)
    joints = [p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint) and "/Cable/" in str(p.GetPath())]
    assert len(joints) == 2 * DEFAULT.cable.tail_links  # tape grip: no film hinges
    meta = stage.GetMetadata("customLayerData")["ffc_twin_spec"]
    assert meta["connector"]["seat_depth"]["value"] == pytest.approx(3.45e-3)


def test_reset_levels_are_ordered():
    assert LEVELS["L1"].lateral > LEVELS["L0"].lateral and LEVELS["L1"].yaw > LEVELS["L0"].yaw
    assert LEVELS["L0"].yaw == pytest.approx(math.radians(2))


def test_grasp_slips_when_axial_load_exceeds_friction():
    """Push the header against the backstop harder than 2 mu N: the film must slide in the jaws."""
    model = mujoco.MjModel.from_xml_string(build_mjcf())
    data = mujoco.MjData(model)
    slip = model.joint("grip_slip").qposadr[0]
    for _ in range(400):
        data.ctrl[0] += 0.2e-3
        for _ in range(50):
            mujoco.mj_step(model, data)
    limit = 2 * DEFAULT.tool.pad_friction.value * DEFAULT.tool.clamp_force.value
    assert abs(data.qpos[slip]) > 0.5e-3, "expected the grip to slip once the backstop load exceeded %.2f N" % limit
