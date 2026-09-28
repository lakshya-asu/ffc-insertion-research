"""Publish matched passing mounted-tool experiments; no CAD surfaces are copied."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cycle", type=Path, required=True)
    parser.add_argument("--arm-cancel", type=Path, required=True)
    parser.add_argument("--tool-cancel", type=Path, required=True)
    parser.add_argument("--destination", type=Path, default=Path("docs/hardware/mounted-tool"))
    args = parser.parse_args()
    reports = {}
    for label, folder in [
        ("cycle", args.cycle),
        ("arm-cancel", args.arm_cancel),
        ("tool-cancel", args.tool_cancel),
    ]:
        report = json.loads((folder / "results.json").read_text())
        if report["status"] != "PASS" or report["camera_control"] or report["contact_qualified"]:
            raise ValueError("Expected passing, explicitly empty-tool commissioning report")
        phases = json.loads((folder / "probe.phases.json").read_text())
        frames = sum(phase["frames"] for phase in phases)
        if frames != report["physics_steps"] // 40:
            raise ValueError("Video frame ledger does not match the physics clock")
        reports[label] = dict(report=report, folder=folder, frames=frames)
    if len({entry["report"]["tool"]["manifest_sha256"] for entry in reports.values()}) != 1:
        raise ValueError("Tool geometry differs between experiments")
    controller_hashes = {
        hashlib.sha256((entry["folder"] / "source" / "joint_trajectory.py").read_bytes()).hexdigest()
        for entry in reports.values()
    }
    if len(controller_hashes) != 1:
        raise ValueError("Controller revisions differ; rerun cancellation on the current controller")
    output = args.destination
    output.mkdir(parents=True, exist_ok=True)
    review = {
        "scope": "Mounted empty-tool commissioning only",
        "controller_sha256": next(iter(controller_hashes)),
        "runs": {},
    }
    for label, entry in reports.items():
        folder = entry["folder"]
        for source, suffix in [
            ("probe.mp4", ".mp4"),
            ("results.json", "-results.json"),
            ("joint-trace.json", "-trace.json"),
            ("probe.phases.json", "-phases.json"),
        ]:
            shutil.copy2(folder / source, output / (label + suffix))
        review["runs"][label] = dict(
            run=folder.name,
            frames=entry["frames"],
            physics_seconds=entry["report"]["physics_seconds"],
            source_hashes=json.loads((folder / "source" / "hashes.json").read_text()),
        )
    shutil.copy2(args.cycle / "latest-frame.jpg", output / "poster.jpg")
    shutil.copytree(args.cycle / "stage-clips", output / "stage-clips", dirs_exist_ok=True)
    rows = json.loads((output / "cycle-trace.json").read_text())
    time = np.array([row["time_s"] for row in rows])
    measured = np.array([row["tool_q_m"] for row in rows]) * 1000
    commanded = np.array([row["tool_reference_m"] for row in rows]) * 1000
    plt.rcParams.update(
        {"font.family": "DejaVu Sans", "font.size": 12, "axes.spines.top": False, "axes.spines.right": False}
    )
    fig, axes = plt.subplots(3, 1, figsize=(14, 11), sharex=True, facecolor="#f8f7f3")
    for i, (label, sign) in enumerate([("Deployment (mm)", 1), ("Clamp extension (mm)", -1)]):
        axes[i].plot(time, commanded[:, i] * sign, color="#999b99", lw=3, label="Commanded reference")
        axes[i].plot(time, measured[:, i] * sign, color="#176b57", lw=1.5, label="Measured joint")
        axes[i].set_ylabel(label)
        axes[i].legend(loc="upper right", frameon=False)
    axes[2].plot(time, measured[:, 0] - commanded[:, 0], color="#176b57", label="Deployment")
    axes[2].plot(time, -measured[:, 1] + commanded[:, 1], color="#b87831", label="Clamp extension")
    axes[2].set_ylabel("Tracking error (mm)")
    axes[2].set_xlabel("Physics time (s)")
    axes[2].legend(frameon=False)
    for ax in axes:
        ax.grid(alpha=0.18)
        ax.set_facecolor("#f8f7f3")
        ax.set_xlim(0, reports["cycle"]["report"]["physics_seconds"])
    fig.suptitle("Mounted CAD gripper · actual Isaac joint feedback", fontsize=22, x=0.1, ha="left", y=0.975)
    fig.text(
        0.1,
        0.935,
        "Free-space mechanism cycle. Assumed drive and inertia model; no cable or grasp forces.",
        fontsize=12,
        color="#626765",
    )
    fig.tight_layout(rect=(0, 0.01, 1, 0.92))
    fig.savefig(output / "tracking.png", dpi=100)
    review["artifact_sha256"] = {
        str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in output.rglob("*")
        if p.is_file() and p.name != "review.json"
    }
    (output / "review.json").write_text(json.dumps(review, indent=2) + "\n")
    print(
        json.dumps(
            {key: {"run": item["folder"].name, "frames": item["frames"]} for key, item in reports.items()},
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
