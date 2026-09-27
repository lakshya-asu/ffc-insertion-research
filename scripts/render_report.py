"""Generate a local, self-contained index of measured experiment outputs."""

import html
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "outputs"


def main():
    sections = []
    active = []
    selected = []
    comparison = (
        '<p><a href="strategy-comparison/comparison.png">Measured strategy comparison</a> · '
        '<a href="strategy-comparison/comparison.json">Comparison data and limits</a></p>'
        if (OUT / "strategy-comparison/comparison.png").exists()
        else ""
    )
    for name, label in (
        ("e040-direct-pgs", "Desk pickup and direct insertion"),
        ("e040-fixture-pgs", "Passive fixture and regrasp comparison"),
        ("e039-pgs-transfer", "Suction-to-pinch handoff and raise"),
        ("e043-selected-static", "Full-length static cable validation"),
        ("e044-pgs-tool-smoke", "Workcell and actuator strokes"),
    ):
        result_path = OUT / name / "results.json"
        if not result_path.exists():
            continue
        result = json.loads(result_path.read_text())
        status = result["status"]
        audit_path = OUT / name / "seating-audit.json"
        if audit_path.exists():
            qualified = json.loads(audit_path.read_text())["geometric_task_qualified"]
            status = "Geometric task qualified" if qualified else "Geometric task not qualified"
        selected.append(
            f'<li><a href="#{name}">{label}</a> — {html.escape(status)}</li>'
        )
    for progress in sorted(OUT.glob("*/recording-progress.json")):
        if (progress.parent / "results.json").exists():
            continue
        data = json.loads(progress.read_text())
        name = progress.parent.name
        active.append(
            f"<details open><summary><b>{html.escape(name)}</b> — no final result yet</summary>"
            f"<p>Last recorded phase: {html.escape(data['phase'])}; "
            f"simulation {data['simulation_time_s']:.2f} s.</p>"
            f'<img style="width:100%" src="{name}/latest-frame.jpg" alt="Latest actual rendered frame">'
            f'<p><a href="{name}/recording-progress.json">Progress</a> · '
            f'<a href="{name}/run.log">Log</a></p></details>'
        )
    for path in sorted(OUT.glob("*/results.json")):
        result = json.loads(path.read_text())
        if "status" not in result:
            continue
        name = path.parent.name
        status = result["status"]
        seating_audit_path = path.parent / "seating-audit.json"
        if seating_audit_path.exists():
            seating_audit = json.loads(seating_audit_path.read_text())
            status += (
                " · independent geometric task audit passed"
                if seating_audit["geometric_task_qualified"]
                else " · independent geometric task not qualified"
            )
        grip_audit_path = path.parent / "grip-audit.json"
        if grip_audit_path.exists():
            grip_audit = json.loads(grip_audit_path.read_text())
            if not grip_audit["grip_stability_qualified"]:
                status += " · grip stability audit failed"
        if status == "PASS" and "cantilever" in result:
            status = (
                "Static shape check passed"
                if result["cantilever"].get("static_shape_agreement_qualified", False)
                else "Run completed; static shape not qualified"
            )
        if "timestep_audit" not in result:
            status += " · development diagnostic; timing not audited"
        iterations = result.get("configuration", {}).get("shell", {}).get("solver_iterations", 64)
        if iterations > 255:
            status += " · excluded: out-of-range solver request"
        explanation = result.get("error", "The checks defined for this experiment passed.")
        if "cantilever" in result:
            explanation += (
                " Cantilever PASS records a finite response; inspect numerical agreement separately."
            )
        elif result.get("assembly_success_tested"):
            explanation += (
                " Scripted geometric seating test; no electrical continuity or latch closure verification."
            )
        elif result.get("pinch_benchmark_only"):
            explanation += (
                " Prepositioned mechanical pinch only; no suction acquisition or desk pickup tested."
            )
        elif result.get("transfer_benchmark_only"):
            explanation += " Isolated suction-to-pinch transfer; approach and insertion were not tested."
        elif result.get("grasp_benchmark_only"):
            explanation += (
                " Isolated pickup/lift from a prepositioned tool; approach and insertion were not tested."
            )
        video = path.parent / "experiment.mp4"
        clips = sorted((path.parent / "stage-clips").glob("*.mp4"))
        links = " ".join(
            f'<a href="{html.escape(str(p.relative_to(OUT)))}">{html.escape(p.stem)}</a>' for p in clips
        )
        analysis_link = (
            f'<p><a href="{name}/handling-metrics.png">Measured handling and contact plots</a></p>'
            if (path.parent / "handling-metrics.png").exists()
            else ""
        )
        if (path.parent / "static-shape-comparison.png").exists():
            analysis_link += (
                f'<p><a href="{name}/static-shape-comparison.png">'
                'Static shape versus independent reference</a></p>'
            )
        if (path.parent / "artifact-checks.json").exists():
            analysis_link += (
                f'<p><a href="{name}/artifact-checks.json">Video, source-hash and clock verification</a></p>'
            )
        for filename, label in (
            ("grip-audit.json", "Independent tool-relative grip stability audit"),
            ("seating-audit.json", "Independent full-tip seating audit"),
        ):
            if (path.parent / filename).exists():
                analysis_link += f'<p><a href="{name}/{filename}">{label}</a></p>'
        media = (
            f'<video controls preload="none" src="{name}/experiment.mp4"></video>'
            if video.exists() and video.stat().st_size > 100
            else ""
        )
        metrics = {
            key: result[key]
            for key in ("physics", "tool_strokes_m", "cantilever", "controller_period_s", "timestep_audit")
            if key in result
        }
        sections.append(f'''<details id="{name}"><summary><b>{html.escape(name)}</b> — {status}</summary>
<p>{html.escape(explanation)}</p>{media}{analysis_link}<p class="clips">{links}</p>
<pre>{html.escape(json.dumps(metrics, indent=2))}</pre>
<p><a href="{name}/results.json">Results</a> · <a href="{name}/run.log">Run log</a>
· <a href="{name}/trajectory.json">Trajectory</a></p></details>''')
    document = """<!doctype html><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>FFC simulation experiment log</title><style>
body{font:16px/1.55 system-ui,sans-serif;background:#f5f4ef;color:#20272c;
max-width:1100px;margin:50px auto;padding:0 24px}
h1{font-size:32px;line-height:1.2}details{border-top:1px solid #bbb;padding:16px 0}summary{cursor:pointer}
video{width:100%;background:#111}pre{overflow:auto;font-size:13px}a{color:#185a75}
.clips a{display:inline-block;margin:4px 12px 4px 0}
</style><h1>FFC simulation experiment log</h1>
<p>Actual Isaac Sim runs, rendered frames and measured checks. A PASS applies only to that run's checks.
The cable mechanics, suction model and connector interior are uncalibrated;
these runs do not verify electrical connection.</p>
<p>Expand a run to play its video or inspect individual stage clips.
Failed runs are retained to show the problems found during development.</p>
<p><a href="benchmarks/shell-convergence.png">Native-shell numerical comparison</a> ·
<a href="benchmarks/segment-convergence.png">Segmented-cable numerical comparison</a></p>
<p><a href="e043-selected-static/static-shape-comparison.png">Selected PGS full-length validation</a></p>
""" + "<h2>Selected experiments</h2>" + comparison + "<ul>" + "\n".join(selected) + "</ul>" + "\n".join(
        active + list(reversed(sections))
    ) + """<script>
function revealRun(){const e=document.getElementById(location.hash.slice(1));
if(e && e.tagName==='DETAILS'){e.open=true;e.scrollIntoView();}}
addEventListener('hashchange',revealRun);revealRun();
</script>"""
    (OUT / "index.html").write_text(document)
    print(OUT / "index.html")


if __name__ == "__main__":
    main()
