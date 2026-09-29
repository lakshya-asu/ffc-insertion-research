"""Plot recorded measured lift/load against issued lift reference; offline review."""

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("run", type=Path)
p.add_argument("output", type=Path)
a = p.parse_args()
rows = json.loads((a.run / "sensor-trace.json").read_text())
t = np.array([r["time_s"] for r in rows])
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, layout="constrained")
fig.patch.set_facecolor("#f4f3ed")
for ax in axes:
    ax.set_facecolor("#f4f3ed")
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(alpha=0.2)
axes[0].plot(
    t, [r["command"]["lift_m"] * 1000 for r in rows], "--", color="#87928a", label="Issued lift reference"
)
axes[0].plot(
    t, [r["observation"]["lift_m"] * 1000 for r in rows], color="#176b57", label="Measured tool lift"
)
axes[0].set_ylabel("Lift (mm)")
axes[0].legend(loc="upper left")
for key, label, color in [
    ("lower_force_n", "Lower pad", "#176b57"),
    ("upper_force_n", "Upper pad", "#ae712d"),
]:
    axes[1].plot(t, [r["observation"][key] for r in rows], color=color, label=label)
axes[1].axhline(0.08, color="#8e524c", ls="--", lw=1, label="Contact threshold (0.08 N)")
axes[1].set_ylabel("Ideal pad load (N)")
axes[1].set_xlabel("Simulation time (s)")
axes[1].legend(loc="lower right")
fault = [r for r in rows if r["command"]["state"] == "fault"]
if fault:
    for ax in axes:
        ax.axvspan(fault[0]["time_s"], t[-1], color="#b9786c", alpha=0.13)
command = rows[-1]["command"]
fig.suptitle(
    a.run.name + "  /  " + (command["reason"] if command["state"] == "fault" else command["state"]),
    fontsize=14,
)
fig.savefig(a.output, dpi=150)
plt.close(fig)
