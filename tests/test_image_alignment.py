import json
from pathlib import Path

import numpy as np
import pytest

from ffc.image_alignment import estimate_alignment

CONFIG = json.loads((Path(__file__).resolve().parents[1] / "config/image-alignment-v1.json").read_text())


def fixture():
    mask = np.zeros((1024, 1232), np.uint8)
    mask[290:301, 200:1001] = 1
    mask[320:331, 200:1001] = 2
    mask[400:416, 220:1021] = 3
    return mask


def test_straight_geometry_known_offset():
    result = estimate_alignment(fixture(), CONFIG)
    assert result["status"] == "image_measurement"
    assert result["measurement"]["projected_normal_separation_px"] == pytest.approx(90)
    assert result["measurement"]["apparent_rim_gap_px"] == pytest.approx(20)
    assert result["measurement"]["relative_image_angle_deg"] == pytest.approx(0, abs=1e-9)
    assert result["calibrated_uncertainty"] is None
    assert not result["motion_permitted"]


@pytest.mark.parametrize("fault", ["absent", "fragment", "curve", "border", "crossed"])
def test_abstains_on_missing_or_ambiguous_geometry(fault):
    mask = fixture()
    if fault == "absent":
        mask[mask == 3] = 0
    elif fault == "fragment":
        mask[:, 580:620] = 0
    elif fault == "border":
        mask[400:416, :220] = 3
    elif fault == "crossed":
        mask[mask == 2] = 0
        mask[270:281, 200:1001] = 2
    else:
        mask[mask == 3] = 0
        for x in range(220, 1021):
            y = 400 + int(0.0002 * (x - 620) ** 2)
            mask[y : y + 16, x] = 3
    result = estimate_alignment(mask, CONFIG)
    assert result["status"] == "abstain" and result["reasons"]
    assert result["measurement"] is None


def test_tilt_is_in_image_degrees():
    mask = fixture()
    mask[mask == 3] = 0
    for x in range(220, 1021):
        y = 400 + round(0.05 * (x - 620))
        mask[y : y + 16, x] = 3
    result = estimate_alignment(mask, CONFIG)
    assert result["measurement"]["relative_image_angle_deg"] == pytest.approx(
        np.degrees(np.arctan(0.05)), abs=0.02
    )
