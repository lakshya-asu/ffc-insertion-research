import numpy as np
import pytest

from ffc.camera_observations import CameraFrame, CameraGate, CameraSpec
from ffc.held_end_inspection import InspectionConfig, compare, inspect


def frame(rgb, sequence=1):
    h, w, _ = rgb.shape
    return CameraFrame("held", sequence, sequence * 0.1, w, h, "test", rgb.tobytes())


def image():
    rgb = np.zeros((100, 160, 3), dtype=np.uint8)
    rgb[30:50, 25:65] = [30, 90, 220]
    rgb[60:80, 70:120] = [220, 180, 30]
    return rgb


def test_visible_regions_do_not_authorize_motion_or_define_leading_edge():
    result, _, _ = inspect(frame(image()), InspectionConfig("test"))
    assert result["terminal_candidate"]["area_px"] == 800
    assert result["leading_edge"] is None
    assert result["slip_state"] == "unverified"
    assert result["motion_permitted"] is False


def test_absent_and_ambiguous_terminal_abstain():
    rgb = image()
    rgb[30:50, 25:65] = 0
    assert "terminal_not_resolved" in inspect(frame(rgb), InspectionConfig("test"))[0]["reasons"]
    rgb = image()
    rgb[5:25, 100:140] = [30, 90, 220]
    assert "terminal_ambiguous_or_clipped" in inspect(frame(rgb), InspectionConfig("test"))[0]["reasons"]


def test_shared_image_translation_is_not_slip():
    cfg = InspectionConfig("test")
    old = inspect(frame(image()), cfg)[0]
    new = inspect(frame(np.roll(image(), 5, axis=1), 2), cfg)[0]
    result = compare(old, new)
    assert np.allclose(result["relative_image_shift_px"], [0, 0])
    assert result["slip_state"] == "unverified"
    with pytest.raises(ValueError):
        compare(new, old)


def test_ingress_rejects_truth_and_stale_images():
    gate = CameraGate({"held": CameraSpec(160, 100, "test")})
    packet = vars(frame(image())).copy()
    with pytest.raises(ValueError):
        gate.accept(packet | {"cable_pose": [0, 0, 0]}, 0.1)
    with pytest.raises(ValueError):
        gate.accept(packet, 1)
    with pytest.raises(ValueError):
        inspect(frame(image()), InspectionConfig("different"))


def test_morphology_does_not_hide_image_clipping():
    rgb = image()
    rgb[30:50, :65] = [30, 90, 220]
    result = inspect(frame(rgb), InspectionConfig("test"))[0]
    assert "terminal_ambiguous_or_clipped" in result["reasons"]
