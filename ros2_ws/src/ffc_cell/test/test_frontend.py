import cv2
import numpy as np
import pytest
from ffc_cell.cv_frontend import Frontend
from ffc_cell.guard import FrameGuard


def rgb_pattern():
    x = np.indices((2048, 2448)).sum(0) % 64 * 4
    return np.stack([x, np.roll(x, 8, 0), np.roll(x, 16, 1)], -1).astype(np.uint8)


def test_macro_resize_preserves_full_field_and_pixel_centres():
    raw = rgb_pattern()
    k = np.array([[17627.7, 0, 1224.0], [0, 17627.7, 1024.0], [0, 0, 1]])
    image, new_k, edges, metrics = Frontend().process(raw, k, np.zeros(5))
    assert image.shape == (1024, 1232, 3) and edges.shape == (1024, 1232)
    assert np.array_equal(image[:, 4:-4], cv2.resize(raw, (1224, 1024), interpolation=cv2.INTER_AREA))
    assert not image[:, :4].any() and not image[:, -4:].any()
    assert new_k[0, 2] == pytest.approx(615.75) and new_k[1, 2] == pytest.approx(511.75)
    assert metrics["std_luma"] > 1


def test_invalid_calibration_and_blank_frame_rejected():
    raw = rgb_pattern()
    f = Frontend()
    k = np.array([[1000.0, 0, 1224], [0, 1000, 1024], [0, 0, 1]])
    with pytest.raises(ValueError, match="Uncalibrated"):
        f.process(raw, np.zeros((3, 3)), np.zeros(5))
    with pytest.raises(ValueError, match="Blank"):
        f.process(np.zeros_like(raw), k, np.zeros(5))
    with pytest.raises(ValueError, match="native"):
        f.process(raw[:100], k, np.zeros(5))


def test_timestamp_and_calibration_faults():
    g = FrameGuard()
    g.check(1_000_000_000, 1_100_000_000, "a")
    g.accept(1_000_000_000, "a")
    cases = [
        (0, 1_100_000_000, "a", "missing"),
        (1_000_000_000, 1_100_000_000, "a", "duplicate"),
        (2_000_000_000, 1_100_000_000, "a", "future"),
        (1_010_000_000, 2_000_000_000, "a", "stale"),
        (1_100_000_000, 1_200_000_000, "b", "calibration changed"),
    ]
    for stamp, now, calib, reason in cases:
        with pytest.raises(ValueError, match=reason):
            g.check(stamp, now, calib)
