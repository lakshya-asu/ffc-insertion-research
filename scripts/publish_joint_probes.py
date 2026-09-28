"""Publish measured Isaac commissioning evidence; no reconstructed animation."""

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
output = ROOT / "docs/demo/motion"
output.mkdir(exist_ok=True)
runs = [
    ("wrist", "isaac-joint-probe-002"),
    ("shoulder", "isaac-joint-probe-003"),
    ("cancel", "isaac-joint-cancel-001"),
]
fig, axes = plt.subplots(3, 1, figsize=(10, 8), layout="constrained")
fig.set_facecolor("#f5f4ef")
summary = []
for ax, (name, run_name) in zip(axes, runs, strict=True):
    run = ROOT / "outputs" / run_name
    report = json.loads((run / "results.json").read_text())
    if report["status"] != "PASS":
        raise RuntimeError(f"Cannot publish a failed run as passed: {run_name}")
    rows = json.loads((run / "joint-trace.json").read_text())
    joint = report["joint"] - 1
    t = np.array([r["time_s"] for r in rows])
    t -= t[0]
    actual = np.array([r["q_rad"][joint] for r in rows])
    ref = np.array([r["reference_rad"][joint] for r in rows])
    base = ref[0]
    ax.set_facecolor("#f5f4ef")
    ax.plot(t, (ref - base) * 1000, label="Commanded reference", color="#a15e27", ls="--", lw=2)
    ax.plot(t, (actual - base) * 1000, label="Measured joint state", color="#176756", lw=1.3)
    ax.set(
        title=f"{name.title()} · joint {joint + 1}",
        ylabel="Change (milliradians)",
        xlabel="Simulation time (s)",
    )
    ax.grid(alpha=0.15)
    ax.spines[["top", "right"]].set_visible(False)
    if report.get("cancel_state"):
        ax.axvline(
            report["cancel_state"]["time_s"] - rows[0]["time_s"],
            color="#ae4361",
            ls=":",
            label="Cancel requested",
        )
    ax.legend(loc="upper right", frameon=False)
    shutil.copyfile(run / "probe.mp4", output / f"{name}-probe.mp4")
    shutil.copyfile(run / "results.json", output / f"{name}-results.json")
    summary.append(dict(name=name, run=run_name, **report))
fig.suptitle("Actual Isaac joint response · unloaded FR3, no camera control", fontsize=15)
fig.savefig(output / "tracking.png", dpi=140)
plt.close(fig)
(output / "manifest.json").write_text(json.dumps(summary, indent=2) + "\n")
