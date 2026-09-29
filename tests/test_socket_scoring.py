"""The offline scorer must evaluate the selected fixture, including offsets."""

import json
import subprocess
import sys
from pathlib import Path

import pytest


@pytest.mark.parametrize(
    "x, y, expected, overrun",
    [(0.001, -0.0023, True, 0), (0, -0.0023, False, 0), (0.001, -0.002504, False, 0.000004)],
)
def test_socket_frame_and_clearance(tmp_path, x, y, expected, overrun):
    fixture = {
        "mouth_y_m": -0.001,
        "length_m": 0.0025,
        "width_m": 0.01165,
        "gap_m": 0.0004,
        "center_x_m": 0.001,
        "center_z_m": 0.06,
    }
    command = {"state": "travel_complete_unverified"}
    records = {
        "report.json": {
            "fixture": fixture,
            "socket_reference": True,
            "socket_settings": {
                "assumed_contact_rest_top_relative_to_cable_center_m": -0.00017,
                "assumed_contact_max_deflection_m": 0.0001,
            },
            "case": "open",
            "final": {"feed_command": command},
        },
        "sections.json": [{"length_m": 0.002, "width_m": 0.0115, "thickness_m": 0.0003}],
        "offline-cable-trace.json": [
            {"time_s": 1, "positions_m": [[x, y, 0.06]], "quaternions_wxyz": [[1, 0, 0, 0]]}
        ],
        "sensor-trace.json": [
            {"time_s": 1, "feed_command": command, "feed_observation": {"fixture_force_n": 0.1}}
        ],
    }
    for name, record in records.items():
        (tmp_path / name).write_text(json.dumps(record))
    score = tmp_path / "score.json"
    script = Path(__file__).resolve().parents[1] / "scripts/score_insertion_bench.py"
    subprocess.run([sys.executable, str(script), str(tmp_path), str(score)], check=True, capture_output=True)
    result = json.loads(score.read_text())
    assert result["tip_entered_assumed_channel"] is expected
    assert result["seating_verified"] is False
    assert result["final_backstop_overrun_m"] == pytest.approx(overrun)
    assert result["tip_reached_review_depth"] is True
