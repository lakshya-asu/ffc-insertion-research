"""Compare measured phases of two completed, independently audited trials."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("direct", type=Path)
    parser.add_argument("fixture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    groups = ["Desk acquisition", "Grip transfer / regrasp", "Transport / alignment", "Insertion / hold"]
    rows = []
    for label, run in (("Direct", args.direct), ("Fixture", args.fixture)):
        result = json.loads((run / "results.json").read_text())
        phases = json.loads((run / "experiment.phases.json").read_text())
        grip = json.loads((run / "grip-audit.json").read_text())
        seating = json.loads((run / "seating-audit.json").read_text())
        total = result["physics"]["simulated_time_s"]
        durations = dict.fromkeys(groups, 0.0)
        for i, phase in enumerate(phases):
            name = phase["phase"]
            if name in ("settle", "approach_pick", "hover_pick", "descend_pick", "acquire_suction", "lift"):
                group = groups[0]
            elif name in ("transport_to_pcb", "align_connector", "approach_slot"):
                group = groups[2]
            elif name in ("insert", "verify_seating"):
                group = groups[3]
            else:
                group = groups[1]
            end = phases[i + 1]["first_simulation_time_s"] if i + 1 < len(phases) else total
            durations[group] += end - phase["first_simulation_time_s"]
        rows.append(
            {
                "strategy": label,
                "run": str(run),
                "geometric_task_qualified": seating["geometric_task_qualified"],
                "simulated_time_s": total,
                "recorded_stages": len(phases),
                "maximum_sampled_pre_insertion_grip_drift_um": grip["maximum_tip_drift_m"] * 1e6,
                "final_tip_depth_mm": seating["tip_geometry"]["depth_m"] * 1000,
                "phase_group_durations_s": durations,
            }
        )
    args.output.mkdir(parents=True, exist_ok=True)
    note = (
        "One deterministic trial per strategy; no reliability estimate or hardware cycle-time claim. "
        "Phase boundaries are sampled at the video frame rate. Materials and contact are uncalibrated."
    )
    (args.output / "comparison.json").write_text(json.dumps({"trials": rows, "limits": note}, indent=2))
    fig, ax = plt.subplots(figsize=(10, 3.7), layout="constrained")
    left = [0.0, 0.0]
    for group, color in zip(groups, ("#557b83", "#bd8952", "#65768e", "#778a61"), strict=True):
        values = [row["phase_group_durations_s"][group] for row in rows]
        ax.barh([row["strategy"] for row in rows], values, left=left, label=group, color=color, height=0.5)
        left = [x + y for x, y in zip(left, values, strict=True)]
    for i, row in enumerate(rows):
        qualified = "qualified" if row["geometric_task_qualified"] else "not qualified"
        ax.text(left[i] + 0.6, i, f"{row['simulated_time_s']:.2f} s\n{qualified}", va="center", fontsize=9)
    ax.set(
        xlim=(0, max(left) * 1.25),
        xlabel="Measured simulated time (s)",
        title="Direct and fixture sequence comparison",
    )
    ax.invert_yaxis()
    ax.spines[["top", "right"]].set_visible(False)
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.22), ncol=2, frameon=False)
    fig.suptitle("Single privileged-state trial each · uncalibrated physical parameters", fontsize=9)
    for extension in ("png", "svg"):
        fig.savefig(args.output / f"comparison.{extension}", dpi=180)
    plt.close(fig)
    print(json.dumps(rows, indent=2))


if __name__ == "__main__":
    main()
