"""Audit tip stability relative to the gripper before connector contact."""

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
from pxr import Usd

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from ffc.isaac_kinematics import from_stage  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = json.loads((args.run / "results.json").read_text())
    samples = json.loads((args.run / "trajectory.json").read_text())
    phases = json.loads((args.run / "experiment.phases.json").read_text())
    events = json.loads((args.run / "episode-events.json").read_text())
    if not any(e["event"] == "phase_completed" and e["phase"] == "pinch" for e in events):
        report = {"grip_stability_qualified": False, "reason": "No completed pinch phase to audit"}
        (args.run / "grip-audit.json").write_text(json.dumps(report, indent=2))
        print(json.dumps(report))
        return
    pinch_index = next(i for i, p in enumerate(phases) if p["phase"] == "pinch")
    start = phases[pinch_index + 1]["first_simulation_time_s"]
    dt = result["configuration"]["physics_dt_s"]
    reference_sample = max((s for s in samples if s["step"] * dt < start), key=lambda s: s["step"])
    chain = from_stage(Usd.Stage.Open(str(args.run / "workcell.usda")))
    names = result["dof_names"]
    indices = [names.index(f"fr3v2_1_joint{i}") for i in range(1, 8)]
    reconstruction_errors = []

    def local_tip(sample):
        pose = chain.forward(np.asarray(sample["joint_positions"])[indices])
        patch = np.asarray(sample["tool_patch_xyz_m"])
        predicted_patch = pose[:3, 3] + pose[:3, :3] @ [0, 0.120, 0.218]
        reconstruction_errors.append(float(np.linalg.norm(predicted_patch - patch)))
        return pose[:3, :3].T @ (np.asarray(sample["cable_tip_xyz_m"]) - patch)

    reference = local_tip(reference_sample)
    selected = {
        "release_after_pinch",
        "raise_after_pinch",
        "lift_after_regrasp",
        "raise_after_fixture",
        "transport_to_pcb",
        "align_connector",
        "approach_slot",
    }
    measurements = {}
    for index, phase in enumerate(phases):
        if phase["phase"] not in selected:
            continue
        end = phases[index + 1]["first_simulation_time_s"] if index + 1 < len(phases) else float("inf")
        drifts = [
            float(np.linalg.norm(local_tip(s) - reference))
            for s in samples
            if phase["first_simulation_time_s"] <= s["step"] * dt < end
        ]
        if drifts:
            measurements[phase["phase"]] = max(drifts)
    maximum = max(measurements.values(), default=None)
    reconstruction_ok = max(reconstruction_errors) < 0.0001
    report = {
        "grip_stability_qualified": maximum is not None and maximum <= 0.001 and reconstruction_ok,
        "maximum_tip_drift_m": maximum,
        "declared_drift_limit_m": 0.001,
        "per_phase_maximum_tip_drift_m": measurements,
        "tool_rotation_source": "Vendor forward kinematics reconstructed from recorded joint positions",
        "maximum_forward_kinematics_patch_error_m": max(reconstruction_errors),
        "reference_time_s": reference_sample["step"] * dt,
        "sampling_hz": 10,
        "note": "Retrospective sampled check; cannot bound motion between samples or certify hardware grip",
        "audit_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    (args.run / "grip-audit.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
