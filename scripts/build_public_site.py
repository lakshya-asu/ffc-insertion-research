"""Package selected measured results into a static GitHub Pages viewer."""

# HTML templates use long literal lines.
# ruff: noqa: E501
import html
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SITE = ROOT / "docs"
RUNS = [
    (
        "e040-direct-pgs",
        "Direct pickup and insertion",
        "41.76 simulated seconds · 16 stages · 1.32 µm sampled grip drift",
    ),
    (
        "e040-fixture-pgs",
        "Passive fixture and regrasp",
        "66.79 simulated seconds · 23 stages · 1.93 µm sampled grip drift",
    ),
    (
        "e039-pgs-transfer",
        "Suction-to-pinch transfer",
        "Acquisition, pinch, vacuum release and a further raise",
    ),
    (
        "e043-selected-static",
        "Cable mechanics check",
        "46.081 mm sag versus 46.132 mm independent nominal reference",
    ),
    (
        "e044-pgs-tool-smoke",
        "Workcell and actuator checks",
        "Seven recorded stages; both actuator strokes approximately 18 mm",
    ),
    (
        "e023-native-workcell",
        "Native surface-FEM comparison",
        "Settling and tool exercise only; full handling not qualified",
    ),
]
FILES = [
    "experiment.mp4",
    "latest-frame.jpg",
    "results.json",
    "artifact-checks.json",
    "grip-audit.json",
    "seating-audit.json",
    "handling-metrics.png",
    "static-shape-comparison.png",
    "experiment.phases.json",
    "contact-peaks.json",
]
STYLE = """body{font:16px/1.6 system-ui,sans-serif;background:#f5f4ef;color:#20272c;max-width:1080px;margin:40px auto;padding:0 20px}h1{font-size:clamp(30px,6vw,48px);line-height:1.15}h2{margin-top:0}a{color:#165b76}section{border-top:1px solid #b8bcb9;padding:32px 0}video,img{width:100%;max-width:100%;background:#172028}video{aspect-ratio:8/3}nav a{display:inline-block;margin:8px 18px 8px 0}.muted{color:#53616a}li{margin:8px 0}table{border-collapse:collapse;width:100%}td,th{padding:10px;text-align:left;border-bottom:1px solid #ccd0cc}.table{overflow-x:auto}summary{cursor:pointer;font-weight:600}details{margin:18px 0}footer{padding:24px 0}"""


def page(title, body):
    return f'<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>{html.escape(title)}</title><style>{STYLE}</style></head><body>{body}</body></html>'


def main():
    SITE.mkdir(exist_ok=True)
    (SITE / ".nojekyll").touch()
    sections = []
    for name, title, description in RUNS:
        source = ROOT / "outputs" / name
        target = SITE / "outputs" / name
        target.mkdir(parents=True, exist_ok=True)
        for filename in FILES:
            if (source / filename).exists():
                shutil.copy2(source / filename, target / filename)
        clips = sorted((source / "stage-clips").glob("*.mp4"))
        clip_dir = target / "stage-clips"
        clip_dir.mkdir(exist_ok=True)
        links = []
        for clip in clips:
            shutil.copy2(clip, clip_dir / clip.name)
            label = clip.stem.replace("_", " ")
            links.append(
                f'<li><a href="outputs/{name}/stage-clips/{clip.name}">{html.escape(label)}</a></li>'
            )
        local_links = "".join(links).replace(f"outputs/{name}/stage-clips/", "")
        (clip_dir / "index.html").write_text(
            page(
                title + " — stage clips",
                f'<h1>{title}</h1><p><a href="../../../index.html#{name}">Back to experiment</a></p><ol>{local_links}</ol>',
            )
        )
        evidence = " · ".join(
            f'<a href="outputs/{name}/{filename}">{label}</a>'
            for filename, label in [
                ("artifact-checks.json", "Video / clock checks"),
                ("grip-audit.json", "Grip audit"),
                ("seating-audit.json", "Seating audit"),
                ("results.json", "Recorded results"),
                ("handling-metrics.png", "Handling plot"),
                ("static-shape-comparison.png", "Cable validation plot"),
            ]
            if (target / filename).exists()
        )
        sections.append(f'''<section id="{name}"><h2>{title}</h2><p class="muted">{description}</p>
<video controls playsinline preload="none" poster="outputs/{name}/latest-frame.jpg"><source src="outputs/{name}/experiment.mp4" type="video/mp4"><a href="outputs/{name}/experiment.mp4">Download video</a></video>
<p><a href="outputs/{name}/experiment.mp4">Open or download full video</a></p><p>{evidence}</p>
<details><summary>{len(clips)} individual stage clips</summary><ol>{"".join(links)}</ol></details></section>''')
    comparison = SITE / "outputs" / "strategy-comparison"
    comparison.mkdir(parents=True, exist_ok=True)
    for name in ["comparison.png", "comparison.svg", "comparison.json"]:
        shutil.copy2(ROOT / "outputs" / "strategy-comparison" / name, comparison / name)
    stop = SITE / "outputs" / "e030-stop-check02"
    stop.mkdir(parents=True, exist_ok=True)
    shutil.copy2(ROOT / "outputs/e030-stop-check02/artifact-checks.json", stop / "artifact-checks.json")
    body = """<header><p class="muted">FRANKA FR3 · ISAAC SIM 6.1 · EXPERIMENT RECORD</p><h1>From desk pickup to cable insertion</h1>
<p>Both direct and passive-fixture strategies passed the independent geometric insertion and grip audits. Watch the actual simulation recordings below.</p>
<nav><a href="#e040-direct-pgs">Direct video</a><a href="#e040-fixture-pgs">Fixture video</a><a href="#comparison">Comparison</a><a href="https://github.com/lakshya-asu/ffc-insertion-research">Source and Docker setup</a></nav>
<p><strong>Scope:</strong> one deterministic trial per strategy, using exact simulated state and uncalibrated material/contact parameters. The assumed open slot has no terminal-spring resistance. These results do not establish latch closure, electrical continuity, hardware performance or reliability.</p>
<div class="table"><table><tr><th>Verified measure</th><th>Direct</th><th>Fixture</th></tr><tr><td>Full-tip seating depth</td><td>3.80105 mm</td><td>3.80099 mm</td></tr><tr><td>Maximum sampled pre-insertion grip drift</td><td>1.32 µm</td><td>1.93 µm</td></tr><tr><td>Verified stage clips</td><td>16</td><td>23</td></tr></table></div>
<p class="muted">17 regression tests passed. Source hashes, decoded recordings and physics-clock checks accompany the selected runs. Recorded 27 September 2026.</p></header>"""
    body += "".join(sections)
    body += """<section id="comparison"><h2>Measured sequence comparison</h2><img loading="lazy" src="outputs/strategy-comparison/comparison.png" alt="Direct sequence: 41.76 simulated seconds; fixture sequence: 66.79 seconds, primarily due to additional regrasp stages."><p>Direct is the initial control baseline; the fixture branch remains available for future tests with uncertain cable poses. These timings reflect the chosen simulation controllers, not predicted hardware cycle times.</p><a href="outputs/strategy-comparison/comparison.json">Comparison data</a></section><footer><a href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/MORNING_REPORT.md">Engineering report</a> · <a href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/CALIBRATION.md">Physical calibration plan</a></footer>"""
    (SITE / "index.html").write_text(page("FR3 cable insertion — experiment videos", body))
    print(f"Built {SITE}; {sum(p.stat().st_size for p in SITE.rglob('*') if p.is_file()) / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
