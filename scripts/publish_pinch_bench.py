"""Publish the finite sensor-pinch bench results, including failures and limits."""

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
DEST = ROOT / "docs/hardware/pinch-bench"
DEST.mkdir(exist_ok=True)
cases = {"hold": "005", "empty": "003", "load-dropout": "004", "tipped-sample": "001"}
summary = {"scope": "Rigid terminal sample on a support; not full cable pickup or insertion", "cases": {}}
fig, axes = plt.subplots(3, 2, figsize=(12, 9), constrained_layout=True)
for name, index in cases.items():
    source = ROOT / ("outputs/pinch-bench-" + index)
    report = json.loads((source / "report.json").read_text())
    trace = json.loads((source / "sensor-trace.json").read_text())
    item = {
        "run": str(source.relative_to(ROOT)),
        "final": report["final"],
        "physics_steps": report["physics_steps"],
        "dt_s": report["physics_dt_s"],
        "post_fault_lift_drift_m": report.get("post_fault_lift_drift_m"),
    }
    if (source / "offline-coupon-trace.json").exists():
        offline = json.loads((source / "offline-coupon-trace.json").read_text())
        if offline:
            item["offline_coupon_lift_mm"] = 1000 * (
                offline[-1]["position_m"][2] - offline[0]["position_m"][2]
            )
    summary["cases"][name] = item
    for file, suffix in [
        ("pinch.mp4", ".mp4"),
        ("report.json", "-report.json"),
        ("sensor-trace.json", "-sensor-trace.json"),
    ]:
        shutil.copyfile(source / file, DEST / (name + suffix))
    Image.open(source / "latest-frame.jpg").save(DEST / (name + ".webp"), quality=90)
    if name == "tipped-sample":
        continue
    row = list(cases).index(name)
    t = [s["time_s"] for s in trace]
    for side in ["lower", "upper"]:
        axes[row, 0].plot(t, [s["observation"][side + "_force_n"] for s in trace], label=side + " pad")
    axes[row, 0].set(title=name + " / reported pad load", ylabel="N", xlabel="simulation seconds")
    axes[row, 0].legend()
    axes[row, 1].plot(
        t, np.array([s["observation"]["lift_m"] for s in trace]) * 1000, label="encoder equivalent"
    )
    axes[row, 1].plot(t, np.array([s["command"]["lift_m"] for s in trace]) * 1000, "--", label="target")
    axes[row, 1].set(title=name + " / tool lift", ylabel="mm", xlabel="simulation seconds")
    axes[row, 1].legend()
    for ax in axes[row]:
        ax.grid(alpha=0.2)
fig.suptitle("Sensor-driven pinch bench — ideal actuator and pad signals; rigid sample")
fig.savefig(DEST / "sensor-traces.png", dpi=140)
plt.close(fig)
assert summary["cases"]["hold"]["final"]["command"]["state"] == "contact_hold_complete"
assert summary["cases"]["empty"]["final"]["command"]["reason"] == "empty_or_too_thin"
assert summary["cases"]["empty"]["final"]["command"]["lift_m"] == 0
assert summary["cases"]["load-dropout"]["final"]["command"]["reason"] == "contact_lost"
(DEST / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
print(json.dumps(summary, indent=2))
