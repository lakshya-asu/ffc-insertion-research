"""Strict sensor-only export of the existing pinch trace, not vision training data."""

import math

OBSERVATIONS = frozenset(
    {"stamp_s", "lower_travel_m", "upper_travel_m", "lift_m", "lower_force_n", "upper_force_n"}
)
COMMANDS = frozenset({"state", "closing_travel_m", "lift_m", "reason"})


def export_episode(trace: list[dict], run_id: str) -> dict:
    if not trace or not run_id:
        raise ValueError("Episode requires an identity and observations")
    rows, previous = [], -math.inf
    for sequence, row in enumerate(trace):
        obs, command = row["observation"], row["command"]
        if set(obs) != OBSERVATIONS or set(command) != COMMANDS:
            raise ValueError("Unknown or missing sensor/action fields")
        stamp = row["time_s"]
        values = [stamp, *obs.values(), command["closing_travel_m"], command["lift_m"]]
        if any(
            isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values
        ):
            raise ValueError("Nonfinite or nonnumeric measurement")
        if stamp <= previous or abs(obs["stamp_s"] - stamp) > 1e-9:
            raise ValueError("Nonmonotonic or mismatched acquisition time")
        if not isinstance(command["state"], str) or not isinstance(command["reason"], str):
            raise ValueError("Invalid controller state/reason")
        previous = stamp
        rows.append(
            {
                "sequence": sequence,
                "acquisition_s": stamp,
                "action_issue_s": stamp,
                "action_application_s": None,
                "observation": dict(obs),
                "action": dict(command),
            }
        )
    return {
        "schema": "ffc.pinch-episode.v1",
        "run_id": run_id,
        "clock": "simulation_seconds",
        "scope": "Ideal pad magnitude and encoder-equivalent commissioning; no camera training",
        "camera_streams": [],
        "offline_labels_included": False,
        "samples": rows,
    }
