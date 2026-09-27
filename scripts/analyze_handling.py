"""Plot measured handling trajectories and rigid contact loads for one completed run."""

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def contact_kind(a, b):
    for cable, other in ((a, b), (b, a)):
        if "/Cable/" in cable:
            if "/Connector/" in other:
                return "Cable–connector"
            if "/Tool/" in other:
                return "Cable–tool"
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    result = json.loads((args.run / "results.json").read_text())
    samples = json.loads((args.run / "trajectory.json").read_text())
    events = json.loads((args.run / "episode-events.json").read_text())
    phases = json.loads((args.run / "experiment.phases.json").read_text())
    contacts = json.loads((args.run / "rigid-contacts.json").read_text())
    peak_path = args.run / "contact-peaks.json"
    peaks = json.loads(peak_path.read_text()) if peak_path.exists() else contacts
    dt = result["configuration"]["physics_dt_s"]
    time = np.array([s["step"] for s in samples]) * dt
    patch = np.array([s["tool_patch_xyz_m"] for s in samples])
    end = np.array([s["cable_xyz_m"][-1] for s in samples])
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True, layout="constrained")
    axes[0].plot(time, patch[:, 2] * 1000, label="Tool contact plane")
    axes[0].plot(time, end[:, 2] * 1000, label="Cable end-segment center")
    axes[0].set_ylabel("Height above desk (mm)")
    axes[0].legend(loc="upper right")
    axes[1].plot(time, (end[:, 0] - patch[:, 0]) * 1000, color="#986844")
    axes[1].set_ylabel("End center − tool X (mm)")
    axes[1].set_title(
        "Longitudinal position relative to tool; includes cable rotation as well as slip", fontsize=10
    )
    sampled = {kind: {} for kind in ("Cable–connector", "Cable–tool")}
    for item in contacts:
        kind = contact_kind(item["collider0"], item["collider1"])
        if kind and item["step"] >= 0:
            step = item["step"]
            sampled[kind][step] = max(sampled[kind].get(step, 0), item["sum_contact_force_magnitudes_n"])
    maximums = {kind: 0.0 for kind in sampled}
    unintended_arm_tool = []
    unintended_tool_contacts = []
    for item in peaks:
        a, b = item["collider0"], item["collider1"]
        if (
            item["step"] >= 0
            and a.startswith("/World/Tool/")
            and b.startswith("/World/Tool/")
            and item["sum_contact_force_magnitudes_n"] > 0.1
        ):
            unintended_tool_contacts.append(item)
        for arm, tool in ((a, b), (b, a)):
            if (
                item["step"] >= 0
                and arm.startswith("/World/FR3/")
                and tool.startswith("/World/Tool/")
                and "fr3v2_1_link7" not in arm
                and item["sum_contact_force_magnitudes_n"] > 0.1
            ):
                unintended_arm_tool.append(item)
        kind = contact_kind(a, b)
        if kind and item["step"] >= 0:
            maximums[kind] = max(maximums[kind], item["sum_contact_force_magnitudes_n"])
    for kind, series in sampled.items():
        if series:
            t = np.array(sorted(series))
            axes[2].plot(t * dt, [series[step] for step in t], label=kind, linewidth=1)
    axes[2].axhline(0.5, color="#b44444", linestyle="--", label="Connector force stop")
    axes[2].set(xlabel="Audited simulation time (s)", ylabel="Max contact-pair load (N)")
    axes[2].legend(loc="upper right")
    important = {"lift", "pinch", "release_after_pinch", "release_suction", "lift_after_regrasp", "insert"}
    for phase in phases:
        if phase["phase"] in important:
            t = phase["first_simulation_time_s"]
            for ax in axes:
                ax.axvline(t, color="#888", alpha=0.4, linewidth=0.7)
            axes[0].text(
                t,
                0.02,
                phase["phase"].replace("_", " "),
                transform=axes[0].get_xaxis_transform(),
                rotation=90,
                va="bottom",
                fontsize=8,
            )
    for ax in axes:
        ax.spines[["top", "right"]].set_visible(False)
        ax.grid(alpha=0.15)
    fig.suptitle(f"{args.run.name} — experiment {result['status']}", fontsize=14)
    fig.savefig(args.run / "handling-metrics.png", dpi=180)
    fig.savefig(args.run / "handling-metrics.svg")
    seating = [event for event in events if event["event"] == "geometric_seating"]
    summary = {
        "experiment_status": result["status"],
        "completed_phases": [e["phase"] for e in events if e["event"] == "phase_completed"],
        "seating": seating[-1] if seating else None,
        "maximum_contact_pair_load_n": maximums,
        "peak_source": "all-step per-pair maxima" if peak_path.exists() else "recorded contact samples",
        "electrical_connection_verified": False,
        "unintended_arm_tool_contacts_over_0_1_n": unintended_arm_tool,
        "unintended_tool_self_contacts_over_0_1_n": unintended_tool_contacts,
        "analysis_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    (args.run / "handling-analysis.json").write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
