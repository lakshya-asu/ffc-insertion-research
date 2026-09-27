from copy import deepcopy

import numpy as np
import pytest

from ffc.cable_perception import detect_cable
from ffc.camera_observations import CameraGate, CameraSpec


def packet():
    return dict(
        camera_id="global",
        sequence=0,
        timestamp_s=1.0,
        width=8,
        height=6,
        calibration_id="cal-v1",
        rgb=bytes(8 * 6 * 3),
    )


def gate():
    return CameraGate({"global": CameraSpec(8, 6, "cal-v1")})


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("true_tip", [1, 2]),
        ("width", 7),
        ("height", True),
        ("calibration_id", "other"),
        ("rgb", bytes(8)),
        ("rgb", bytearray(8 * 6 * 3)),
        ("sequence", -1),
        ("timestamp_s", float("nan")),
        ("timestamp_s", 2.0),
        ("timestamp_s", 0.1),
        ("camera_id", "debug_follow_tip"),
    ],
)
def test_bad_image_packets(key, value):
    p = packet()
    p[key] = value
    with pytest.raises(ValueError):
        gate().accept(p, 1.01)


def test_replay_and_missing_frame_are_rejected():
    g = gate()
    p = packet()
    g.accept(p, 1.01)
    with pytest.raises(ValueError):
        g.accept(p, 1.02)
    with pytest.raises(ValueError):
        g.accept({}, 1.03)
    with pytest.raises(ValueError):
        g.accept(p, 2.0)
    p.update(sequence=1, timestamp_s=1.04)
    # Same pixels in a static scene are legitimate if delivery advances.
    assert g.accept(p, 1.05).rgb == bytes(144)


def test_bad_packet_does_not_advance_sequence():
    g = gate()
    p = packet()
    bad = deepcopy(p)
    bad["rgb"] = b""
    with pytest.raises(ValueError):
        g.accept(bad, 1.1)
    assert g.accept(p, 1.0).sequence == 0


def test_detector_empty_and_blue_distractor():
    rgb = np.full((100, 200, 3), 100, dtype=np.uint8)
    assert not detect_cable(rgb)[0]["detected"]
    rgb[45:55, 20:180] = [20, 70, 210]
    assert not detect_cable(rgb)[0]["detected"]


def test_detector_returns_unordered_endpoints_without_motion_authority():
    rgb = np.full((100, 200, 3), 100, dtype=np.uint8)
    rgb[45:55, 20:180] = [220, 200, 140]
    prediction, mask = detect_cable(rgb)
    assert prediction["detected"] and mask.sum() == 1600
    assert prediction["contact_face"] == "unknown" and not prediction["motion_permitted"]
    assert np.allclose(sorted(p[0] for p in prediction["unordered_endpoints_uv"]), [20, 179])


def test_partial_view_does_not_claim_two_physical_endpoints():
    rgb = np.full((100, 200, 3), 100, dtype=np.uint8)
    rgb[45:55, :180] = [220, 200, 140]
    result, _ = detect_cable(rgb)
    assert result["detected"] and result["partial_view"]
    assert result["unordered_endpoints_uv"] is None


@pytest.mark.parametrize("coefficients", [[1], [float("nan")] * 10])
def test_invalid_learned_model(coefficients):
    with pytest.raises(ValueError):
        detect_cable(np.zeros((100, 200, 3), dtype=np.uint8), {"coefficients": coefficients})


def test_large_non_ribbon_does_not_hide_a_smaller_ribbon():
    rgb = np.full((200, 300, 3), 100, dtype=np.uint8)
    rgb[20:100, 20:100] = [220, 200, 140]
    rgb[140:150, 50:250] = [220, 200, 140]
    result, _ = detect_cable(rgb)
    assert result["detected"]
    assert 140 < result["center_uv"][1] < 150


def test_multiple_similar_ribbons_require_reobservation():
    rgb = np.full((200, 300, 3), 100, dtype=np.uint8)
    rgb[40:50, 50:250] = [220, 200, 140]
    rgb[140:150, 50:250] = [220, 200, 140]
    result, _ = detect_cable(rgb)
    assert not result["detected"] and result["reason"] == "ambiguous_multiple_ribbons"
