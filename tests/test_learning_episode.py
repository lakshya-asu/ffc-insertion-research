import copy

import pytest

from ffc.learning_episode import export_episode


def sample():
    return {
        "time_s": 0.1,
        "observation": {
            "stamp_s": 0.1,
            "lower_travel_m": 0.004,
            "upper_travel_m": 0.004,
            "lift_m": 0.0,
            "lower_force_n": 0.1,
            "upper_force_n": 0.1,
        },
        "command": {"state": "close", "closing_travel_m": 0.004, "lift_m": 0.0, "reason": ""},
    }


def test_sensor_boundary():
    row = sample()
    row["offline_cable_pose"] = [1, 2, 3]
    episode = export_episode([row], "run")
    assert "offline_cable_pose" not in str(episode)
    assert episode["camera_streams"] == []
    row["observation"]["socket_pose"] = [1, 2, 3]
    with pytest.raises(ValueError, match="Unknown"):
        export_episode([row], "run")


@pytest.mark.parametrize("failure", ["nan", "stale", "mismatch"])
def test_bad_timing_and_values(failure):
    a = sample()
    b = copy.deepcopy(a)
    if failure == "nan":
        a["observation"]["lower_force_n"] = float("nan")
    if failure == "mismatch":
        a["observation"]["stamp_s"] = 0.2
    with pytest.raises(ValueError):
        export_episode([a, b], "run")
