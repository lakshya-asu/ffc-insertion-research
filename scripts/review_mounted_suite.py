"""Independent review of one frozen positive/empty/dropout commissioning suite."""

import argparse
import json
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("positive", type=Path)
p.add_argument("empty", type=Path)
p.add_argument("dropout", type=Path)
p.add_argument("output", type=Path)
a = p.parse_args()
runs = [a.positive, a.empty, a.dropout]
reports = [json.loads((r / "report.json").read_text()) for r in runs]
traces = [json.loads((r / "sensor-trace.json").read_text()) for r in runs]
hashes = [json.loads((r / "source-hashes.json").read_text()) for r in runs]
checks = {
    "same_frozen_sources": hashes[0] == hashes[1] == hashes[2],
    "same_timestep": len({r["dt_s"] for r in reports}) == 1,
    "same_servo_gain": len({r["joint_integral_gain_per_s"] for r in reports}) == 1,
    "same_tool_assets": all(r["tool"] == reports[0]["tool"] for r in reports),
    "no_cable_pose_control_or_attachment": all(
        not r["cable_pose_control"] and not r["grasp_attachment"] for r in reports
    ),
    "no_reported_unintended_contacts": all(not r["unintended_contacts"] for r in reports),
}
positive, empty, dropout = traces
checks["positive_contact_hold_complete"] = positive[-1]["command"]["state"] == "contact_hold_complete"
checks["positive_lift_within_existing_0p1mm_tolerance"] = (
    abs(positive[-1]["observation"]["lift_m"] - 0.01) < 0.0001
)
checks["empty_rejected"] = empty[-1]["command"]["reason"] == "empty_or_too_thin"
checks["empty_never_commanded_lift"] = all(r["command"]["lift_m"] == 0 for r in empty)
fault = [r for r in dropout if r["command"]["state"] == "fault"]
checks["dropout_contact_loss_detected"] = bool(fault) and fault[0]["command"]["reason"] == "contact_lost"
checks["dropout_arm_target_latched_to_fault_measurement"] = bool(fault) and all(
    np.array_equal(row["arm_reference_rad"], fault[0]["arm_q_rad"]) for row in fault
)
checks["dropout_integral_cleared"] = bool(fault) and all(
    np.all(np.asarray(row["joint_integral_rad"]) == 0) for row in fault
)
result = {
    "run_ids": [r.name for r in runs],
    "checks": checks,
    "controller_checks_passed": all(checks.values()),
    "positive_measured_lift_m": positive[-1]["observation"]["lift_m"],
    "dropout_measured_lift_drift_m": dropout[-1]["observation"]["lift_m"] - fault[0]["observation"]["lift_m"]
    if fault
    else None,
    "dropout_joint_drift_rad": reports[-1]["post_fault_joint_drift_rad"],
    "scope": "One presented-cable commissioning configuration, three trials; no reliability rate. "
    "Independent material-point review is separate. No vision, hardware, damage or insertion validation.",
}
with a.output.open("x") as f:
    json.dump(result, f, indent=2)
print(json.dumps(result))
if not result["controller_checks_passed"]:
    raise SystemExit(1)
