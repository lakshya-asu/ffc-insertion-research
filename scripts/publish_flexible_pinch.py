"""Publish the exploratory flexible-cable motion bench with its negative cases."""

import argparse
import hashlib
import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--positive", type=Path, required=True)
    p.add_argument("--empty", type=Path, required=True)
    p.add_argument("--dropout", type=Path, required=True)
    a = p.parse_args()
    dest = ROOT / "docs/hardware/flexible-motion"
    dest.mkdir(parents=True, exist_ok=True)
    rows, traces = [], []
    for label, run in [("lift", a.positive), ("empty", a.empty), ("dropout", a.dropout)]:
        if (run / "failure.txt").exists():
            raise ValueError(f"Unfinished or failed run: {run}")
        report = json.loads((run / "report.json").read_text())
        trace = json.loads((run / "sensor-trace.json").read_text())
        offline = json.loads((run / "offline-cable-trace.json").read_text())
        final = report["final"]
        if (
            not final
            or abs(report["physics_steps"] * report["physics_dt_s"] - report["physics_elapsed_s"]) > 1e-6
        ):
            raise ValueError("Missing completion or inconsistent physics clock")
        row = {
            "case": label,
            "run": run.name,
            "final_state": final["command"]["state"],
            "reason": final["command"]["reason"],
            "tool_lift_mm": final["observation"]["lift_m"] * 1000,
            "commanded_lift_mm": final["command"]["lift_m"] * 1000,
            "post_fault_lift_drift_mm": None
            if report["post_fault_lift_drift_m"] is None
            else report["post_fault_lift_drift_m"] * 1000,
            "report_sha256": hashlib.sha256((run / "report.json").read_bytes()).hexdigest(),
            "cable_material_point_lift_mm": None,
        }
        if offline:
            profile = json.loads((run / "sections.json").read_text())
            distances = np.array([s["center_m"] for s in profile])
            initial = np.array(offline[0]["positions_m"])
            last = np.array(report["offline_final_positions_m"])
            z_start = np.interp(0.0105, distances, initial[:, 2])
            z_end = np.interp(0.0105, distances, last[:, 2])
            row["cable_material_point_lift_mm"] = float((z_end - z_start) * 1000)
            row["material_point_note"] = "Offline interpolation at the original 10.5 mm material coordinate"
        rows.append(row)
        traces.append(trace)
        for source, target in [
            ("report.json", "report.json"),
            ("sensor-trace.json", "sensor-trace.json"),
            ("offline-cable-trace.json", "offline-cable-trace.json"),
            ("pinch.mp4", "mp4"),
        ]:
            shutil.copy2(
                run / source, dest / f"{label}-{target}" if target != "mp4" else dest / f"{label}.mp4"
            )
    shutil.copy2(a.positive / "latest-frame.jpg", dest / "lift.jpg")
    (dest / "summary.json").write_text(
        json.dumps({"scope": "Exploratory contact motion, not task success", "cases": rows}, indent=2) + "\n"
    )
    fig, axes = plt.subplots(2, 3, figsize=(13, 6), sharex="col", facecolor="#f6f4ef")
    for col, (row, trace) in enumerate(zip(rows, traces, strict=True)):
        times = [r["time_s"] for r in trace]
        top, bottom = axes[:, col]
        top.set_facecolor("#f6f4ef")
        bottom.set_facecolor("#f6f4ef")
        for key, label in [("lower_force_n", "Lower pad"), ("upper_force_n", "Upper pad")]:
            top.plot(times, [r["observation"][key] for r in trace], label=label)
        top.set(title=row["case"].capitalize(), ylabel="Pad load magnitude (N)")
        bottom.plot(times, [r["command"]["lift_m"] * 1000 for r in trace], "--", label="Command")
        bottom.plot(times, [r["observation"]["lift_m"] * 1000 for r in trace], label="Measured tool")
        bottom.set(xlabel="Simulation time (s)", ylabel="Lift (mm)")
        top.grid(alpha=0.15)
        bottom.grid(alpha=0.15)
    axes[0, 0].legend(frameon=False)
    axes[1, 0].legend(frameon=False)
    fig.tight_layout()
    fig.savefig(dest / "sensor-traces.png", dpi=150)
    plt.close(fig)
    positive = rows[0]
    section = f"""<section id="flexible-motion"><p class="eyebrow">Motion pilot / experiment 038</p>
<h2>The fingers lift a flexible cable end.</h2>
<p>The compact tool closes on an insulated part of the presented 200 mm cable, using pad loads
and actuator displacement feedback. Its tail rests on a support. Cable motion comes from joint
forces and contact; there is no attachment joint or scripted cable movement.</p>
<video controls playsinline preload="metadata" poster="flexible-motion/lift.jpg"
aria-label="Actual Isaac flexible-cable contact motion pilot">
<source src="flexible-motion/lift.mp4" type="video/mp4"></video>
<p>Final controller state: <code>{positive["final_state"]}</code>.
Measured tool lift: {positive["tool_lift_mm"]:.3f} mm. The original material point under the pads
rose {positive["cable_material_point_lift_mm"]:.3f} mm in the separate offline check.
That measurement never enters the controller.</p>
<div class="sequence"><div><h3>Presented cable</h3><p>The fingers begin with a 1 mm opening.
The pad footprint is 9–12 mm behind the tip, outside the assumed 4 mm exposed-contact strip.
This is a controlled starting placement, not a camera-guided desk pickup.</p></div>
<div><h3>Empty grasp</h3><p>The controller checks the remaining opening before allowing a lift.</p>
<a href="flexible-motion/empty.mp4">Watch the empty test</a></div>
<div><h3>Lost feedback</h3><p>A separate trial zeros the pad-load signal during lifting.
This tests a sensor fault, not a physical slipping event.</p>
<a href="flexible-motion/dropout.mp4">Watch the stop test</a></div></div>
<figure><img src="flexible-motion/sensor-traces.png" loading="lazy" style="width:100%;height:auto"
alt="Pad loads and commanded versus measured lift in positive, empty and load-dropout trials">
<figcaption>200 Hz local controller; idealized pad-load and encoder signals. No ROS transport
in this bench.</figcaption></figure>
<p>The full cable has 100 articulated sections, variable width and two assumed terminal reinforcements.
This contact pilot uses a 0.125 ms timestep. The short-strip bending benchmark below uses a finer
timestep and does not qualify these full-cable contact dynamics. Friction, damping and actuator response
remain declared assumptions; only the tool pads have collision geometry.</p>
<p>The next integration is the loaded tool on the FR3, with whole-tool clearance and camera evidence
of retention and a visible leading edge. Current completion means bilateral contact was held;
camera verification, suction, desk pickup and insertion are still separate work.</p>
<p><a href="flexible-motion/summary.json">All case results</a> ·
<a href="flexible-motion/lift-report.json">Positive report</a> ·
<a href="flexible-motion/lift-sensor-trace.json">Controller observations and commands</a> ·
<a href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/experiments/038-flexible-cable-motion.md">
Methods and remaining checks</a></p></section>"""
    page = ROOT / "docs/hardware/custom-gripper.html"
    html = page.read_text()
    if '<section id="flexible-motion">' in html:
        start = html.index('<section id="flexible-motion">')
        end = html.index("</section>", start) + len("</section>")
        html = html[:start] + html[end:]
    html = html.replace('<section id="bending-fit">', section + '\n<section id="bending-fit">')
    html = html.replace(
        'href="#bending-fit">New: watch the assumed-world bending fit and validation.',
        'href="#flexible-motion">New: watch the flexible-cable motion pilot.',
    )
    page.write_text(html)
    print(json.dumps(rows))


if __name__ == "__main__":
    main()
