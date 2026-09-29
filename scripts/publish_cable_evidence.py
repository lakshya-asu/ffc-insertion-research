"""Publish source profile and every declared thin-body benchmark, including failures."""

import json
import shutil
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]


def main():
    dest = ROOT / "docs/hardware/cable-physics"
    dest.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(10, 4.5), facecolor="#f6f4ef")
    ax.set_facecolor("#f6f4ef")
    rows, links = [], []
    for run in ["thin-bend-001", "thin-bend-002", "thin-bend-003"]:
        folder = ROOT / "outputs" / run
        report = json.loads((folder / "report.json").read_text())
        if report["single_case_reference_agreement"] is not False:
            raise ValueError(
                "This review describes failed cases; update its interpretation before publishing"
            )
        trace = json.loads((folder / "trace.json").read_text())
        label = f"{report['solver']}, {report['dt_s'] * 1000:g} ms"
        ax.plot([p["time_s"] for p in trace], [p["tip_sag_m"] * 1000 for p in trace], label=label)
        shutil.copy2(folder / "report.json", dest / f"{run}.json")
        shutil.copy2(folder / "bending.mp4", dest / f"{run}.mp4")
        rows.append(
            f"<tr><th>{label}</th><td>{report['observed_tip_sag_m'] * 1000:.3f} mm</td>"
            f"<td>{report['relative_error'] * 100:.1f}%</td><td>Failed</td></tr>"
        )
        links.append(
            f'<a href="cable-physics/{run}.mp4">Watch {label}</a> · '
            f'<a href="cable-physics/{run}.json">Report</a>'
        )
    reference = report["reference"]["nonlinear_hinge_chain_prediction_m"] * 1000
    ax.axhline(reference, color="#292929", linestyle="--", label=f"Independent reference: {reference:.3f} mm")
    ax.set(
        xlabel="Simulation time (s)",
        ylabel="Free-end sag (mm)",
        title="Thin-body bending: solver checks have not passed",
    )
    ax.legend(frameon=False)
    ax.grid(alpha=0.15)
    fig.tight_layout()
    fig.savefig(dest / "bending-comparison.png", dpi=180)
    plt.close(fig)
    shutil.copy2(ROOT / "config/cables/rpi-camera-standard-mini-200-rev2.json", dest / "cable-spec.json")
    shutil.copy2(ROOT / "outputs/cable-source-review-001/pickup-release.json", dest / "pickup-release.json")
    fixture = ROOT / "outputs/fixture-profile-001"
    shutil.copy2(fixture / "report.json", dest / "fixture-profile.json")
    shutil.copy2(fixture / "settle.mp4", dest / "fixture-profile.mp4")
    section = (
        """<section id="cable-physics"><p class="eyebrow">Cable evidence / experiment 036</p>
<h2>Match the cable. Then test the physics.</h2>
<p>We have pinned the Raspberry Pi 200 mm Standard–Mini shielded camera cable to its revision-two
outline. The new fixture model includes the 11.5 mm body, the wider 15-pin end and both reinforced ends.
Terminal lengths, thickness, stiffness and friction remain declared assumptions.</p>
<p>A short bending test exposes a problem: the simulated sag depends strongly on solver settings and
exceeds an independent calculation. A steady-looking cable is not enough. These runs do not qualify
pickup or insertion training data.</p>
<figure><img src="cable-physics/bending-comparison.png" loading="lazy" alt="Three simulated bending
traces remain above the independent 0.415 mm sag reference."
style="width:100%;height:auto"><figcaption>Same material and geometry in all three runs. The dashed line
is an independent nonlinear hinge-chain calculation.</figcaption></figure>
<div class="roadmap-table"><table><thead><tr><th>Solver / timestep</th><th>Final sag</th><th>Reference
error</th><th>5% check</th></tr></thead><tbody>"""
        + "".join(rows)
        + """</tbody></table></div>
<p>"""
        + "<br>".join(links)
        + """</p>
<video controls preload="metadata" aria-label="Revised full cable gravity settling, unqualified
mechanics"><source src="cable-physics/fixture-profile.mp4" type="video/mp4"></video>
<p class="small">The full-cable movie checks that the revised profile runs in Isaac. It is gravity
settling on a fixture, with no pickup, suction or controller. The numerical failures above still
apply.</p>
<p>The next dataset will record synchronized camera and measured tool signals, bounded actions and
failures. Simulator geometry stays in offline labels and evaluation. We will establish a sensor-feedback
baseline, then imitation learning, before considering constrained reinforcement learning.</p>
<div class="download-list"><a
href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/research/cables/standard-mini-200.md"><strong>Read
the cable dossier</strong><span>Manufacturer sources, assumptions, mechanics tests and learning
sequence.</span></a><a href="cable-physics/cable-spec.json"><strong>Inspect the parameter
profile</strong><span>Units, evidence, exploratory ranges and exact profile identity.</span></a><a
href="cable-physics/pickup-release.json"><strong>Dataset release: closed</strong><span>Missing
convergence, contact, pickup and sensor-boundary evidence.</span></a><a
href="cable-physics/fixture-profile.json"><strong>Full-cable run report</strong><span>Revised outline and
archived assumptions.</span></a></div></section>"""
    )
    page = ROOT / "docs/hardware/custom-gripper.html"
    html = page.read_text()
    if '<section id="cable-physics">' in html:
        start = html.index('<section id="cable-physics">')
        end = html.index("</section>", start) + len("</section>")
        html = html[:start] + html[end:]
    html = html.replace('<section id="handoff">', section + '\n<section id="handoff">')
    if 'href="#cable-physics"' not in html:
        html = html.replace(
            '<main id="main" class="tool-page">',
            '<main id="main" class="tool-page"><p class="review-note"><a href="#cable-physics">New: cable source dossier and failed bending checks.</a></p>',  # noqa: E501
        )
    page.write_text(html)


if __name__ == "__main__":
    main()
