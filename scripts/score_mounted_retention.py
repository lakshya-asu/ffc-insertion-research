"""Offline material-point retention and stopping audit; never a policy input."""

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def score(run):
    report = json.loads((run / "report.json").read_text())
    frames = json.loads((run / "offline-cable-trace.json").read_text())
    trace = json.loads((run / "sensor-trace.json").read_text())
    lift_rows = [r for r in trace if r["command"]["state"] in {"lift", "hold", "contact_hold_complete"}]
    lift_time = lift_rows[0]["time_s"] if lift_rows else None
    coords = []
    world = []
    for frame in frames:
        if not frame["positions_m"] or (lift_time is not None and frame["time_s"] < lift_time):
            continue
        if len(frame["positions_m"]) != 100:
            raise ValueError("Audit requires the captured 100-section profile")
        body = frame["tool_bodies"]["fixed"]
        q = body["quaternion_wxyz"]
        rotation = Rotation.from_quat([*q[1:], q[0]])
        xyz = np.mean(np.array(frame["positions_m"], dtype=float)[5:7], axis=0)
        coords.append(rotation.inv().apply(xyz - body["position_m"]))
        world.append(xyz)
    fault = [r for r in trace if r["command"]["state"] == "fault"]
    result = {
        "run_id": run.name,
        "final_command": report["final"]["command"],
        "measured_tool_lift_m": report["final"]["observation"]["lift_m"],
        "material_point_definition": "Midpoint of section centers at 11 and 13 mm from mini end",
        "sample_origin": "First captured frame after lift qualification, when available",
        "material_point_local_start_m": coords[0].tolist() if coords else None,
        "material_point_local_end_m": coords[-1].tolist() if coords else None,
        "local_displacement_m": (coords[-1] - coords[0]).tolist() if coords else None,
        "max_local_displacement_norm_m": float(np.max(np.linalg.norm(np.array(coords) - coords[0], axis=1)))
        if coords
        else None,
        "material_point_vertical_displacement_m": float(world[-1][2] - world[0][2]) if world else None,
        "post_fault_lift_drift_m": trace[-1]["observation"]["lift_m"] - fault[0]["observation"]["lift_m"]
        if fault
        else None,
        "source_sha256": {
            name: hashlib.sha256((run / name).read_bytes()).hexdigest()
            for name in ["report.json", "offline-cable-trace.json", "sensor-trace.json"]
        },
        "scope": "Offline simulation geometry audit. Relative motion can include bending, not only slip. "
        "No visual retention, material damage or insertion claim.",
    }
    return result


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    with args.output.open("x") as f:
        json.dump(score(args.run), f, indent=2)
