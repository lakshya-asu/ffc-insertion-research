"""Publish all four declared fixture-settling trials, including numerical sensitivity."""

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    destination = ROOT / "docs/hardware/handoff"
    destination.mkdir(parents=True, exist_ok=True)
    runs = [
        ("nominal", "fixture-settle-001"),
        ("softer", "fixture-settle-002"),
        ("smaller-timestep", "fixture-settle-003"),
        ("stiffer", "fixture-settle-004"),
    ]
    records = []
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.8), facecolor="#f6f4ef")
    for name, folder in runs:
        source = ROOT / "outputs" / folder
        report = json.loads((source / "report.json").read_text())
        trace = json.loads((source / "offline-trace.json").read_text())
        if report["physics_steps"] != round(2 / report["dt_s"]):
            raise ValueError("Incomplete trial")
        records.append({"name": name, "run": folder, **report})
        for source_name, suffix in [
            ("settle.mp4", ".mp4"),
            ("report.json", ".json"),
            ("latest-frame.jpg", ".jpg"),
        ]:
            shutil.copyfile(source / source_name, destination / (name + suffix))
        pinch_index = int(0.012 / (0.2 / report["segments"]))
        label = f"{report['stiffness_scale']:g}× E, {report['dt_s'] * 1000:g} ms"
        for ax, index, reference in [(axes[0], 0, 0.03015), (axes[1], pinch_index, 0.03007)]:
            ax.plot(
                [t["time_s"] for t in trace],
                [(reference - t["positions_m"][index][2]) * 1000 for t in trace],
                label=label,
            )
    for ax, title in zip(axes, ["Leading segment drop", "Grip-region segment drop"], strict=True):
        ax.set_facecolor("#f6f4ef")
        ax.set(title=title, xlabel="Simulation time (s)", ylabel="Vertical drop (mm)")
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
    fig.suptitle("Fixture settling: material assumptions and numerical sensitivity")
    fig.text(
        0.04,
        0.01,
        "Offline measurements. Unanchored segmented ribbon; no pickup. Material behavior is unqualified.",
        fontsize=9,
    )
    fig.tight_layout(rect=(0, 0.035, 1, 0.94))
    fig.savefig(destination / "settling-comparison.png", dpi=160)
    delta = abs(records[0]["tip_center_drop_mm"] - records[2]["tip_center_drop_mm"])
    summary = {
        "status": "UNQUALIFIED_MATERIAL_MODEL",
        "runs": records,
        "timestep_halving_tip_difference_mm": delta,
        "all_four_trials_included": True,
        "pickup_demonstrated": False,
    }
    (destination / "settling-summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(
        json.dumps(
            {
                "timestep_halving_tip_difference_mm": delta,
                "results": [
                    (r["name"], r["tip_center_drop_mm"], r["pinch_region_center_drop_mm"]) for r in records
                ],
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
