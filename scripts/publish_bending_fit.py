"""Publish fitted and reserved bending responses without hiding failed cases."""

import argparse
import hashlib
import json
import shutil
import sys
import zipfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def main():
    from ffc.bending_fit import select_gain

    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--fit", type=Path, required=True)
    p.add_argument("--validation", type=Path, required=True)
    p.add_argument("--selection", type=Path, required=True)
    p.add_argument("--audit-dir", type=Path, required=True)
    a = p.parse_args()
    if any((folder / "failure.txt").exists() for folder in [a.fit, a.validation]):
        raise ValueError("A run failed; review it before publishing")
    fit = json.loads((a.fit / "report.json").read_text())
    val = json.loads((a.validation / "report.json").read_text())
    selection = json.loads(a.selection.read_text())
    completion = json.loads((a.validation / "completed.json").read_text())
    if completion["inspection_images"] != len(val["cases"]):
        raise ValueError("Validation review images are incomplete")
    gain = selection["selected_gain"]
    if val["settings"]["gains"] != [gain] or val["settings"]["split"] != "validation":
        raise ValueError("Validation does not match the frozen selection")
    if val["settings"]["selection_sha256"] != hashlib.sha256(a.selection.read_bytes()).hexdigest():
        raise ValueError("Validation selection hash does not match the published decision")
    if fit["settings"]["target_sha256"] != val["settings"]["target_sha256"]:
        raise ValueError("Fitting and validation used different reference archives")
    for label, folder in [("fit", a.fit), ("validation", a.validation)]:
        report_sha = hashlib.sha256((folder / "report.json").read_bytes()).hexdigest()
        for kind in ["shape", "measurements"]:
            audit = json.loads((a.audit_dir / f"{label}-{kind}.json").read_text())
            if audit["sources"]["report.json"] != report_sha:
                raise ValueError("Audit belongs to a different physics run")
    cases = [c for c in fit["cases"] if c["gain"] == gain] + val["cases"]
    dest = ROOT / "docs/hardware/bending-fit"
    dest.mkdir(parents=True, exist_ok=True)
    earlier = ROOT / "outputs/bending-fit-002"
    shutil.copy2(earlier / "report.json", dest / "coarse-fit.json")
    for run in [1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13]:
        source = ROOT / "outputs" / f"gain-fit-{run:03d}"
        shutil.copy2(source / "report.json", dest / f"diagnostic-{run:03d}.json")
        shutil.copy2(source / "bending.mp4", dest / f"diagnostic-{run:03d}.mp4")
    for label, folder in [("fit", a.fit), ("validation", a.validation)]:
        for name, target in [("report.json", f"{label}.json"), ("bending.mp4", f"{label}.mp4")]:
            shutil.copy2(folder / name, dest / target)
    shutil.copy2(a.selection, dest / "selection.json")
    shutil.copy2(a.fit / "targets.json", dest / "targets.json")
    shutil.copy2(a.selection.parent / "fit-scores.json", dest / "fit-scores-original-scorer.json")
    (dest / "fit-scores.json").write_text(json.dumps(select_gain(fit), indent=2) + "\n")
    for name in [
        "fit-shape.json",
        "validation-shape.json",
        "fit-measurements.json",
        "validation-measurements.json",
    ]:
        shutil.copy2(a.audit_dir / name, dest / name)
    for label, run in [("fit", "bending-fit-003"), ("validation", "bending-validation-001")]:
        shutil.copy2(ROOT / "outputs" / run / "report.json", dest / f"{label}-original-scorer.json")
        shutil.copy2(
            a.selection.parent / f"{label}-shape-corrected.json", dest / f"{label}-rescored-shape.json"
        )
    for label, folder in [("fit", a.fit), ("validation", a.validation)]:
        shutil.copy2(folder / "measured-poses.npz", dest / f"{label}-measured-poses.npz")
    shutil.copy2(a.validation / "latest-frame.jpg", dest / "validation.jpg")
    shutil.copytree(a.validation / "cases", dest / "cases", dirs_exist_ok=True)
    with zipfile.ZipFile(dest / "validation-scene.zip", "w", zipfile.ZIP_DEFLATED) as archive:
        for name in [
            "bending.usda",
            "report.json",
            "settings.json",
            "selection.json",
            "completed.json",
            "measured-poses.npz",
        ]:
            archive.write(a.validation / name, name)
        archive.writestr(
            "README.txt",
            "Saved final state of a static bending benchmark, not a robot cell.\n"
            "Red OfflineReference curves are human review guides, not policy inputs.\n"
            "Open bending.usda in Isaac with the timeline paused.\n"
            "See settings.json for the numerical profile; material values are assumptions.\n",
        )
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.6), facecolor="#f6f4ef")
    colors = {0.024: "#366983", 0.032: "#598632", 0.028: "#a75a2b", 0.040: "#8d4274"}
    for ax, e in zip(axes, [0.25, 1, 4], strict=True):
        ax.set_facecolor("#f6f4ef")
        for length, color in colors.items():
            rows = sorted(
                [c for c in cases if c["length_m"] == length and c["modulus_multiplier"] == e],
                key=lambda c: c["uniform_load_multiplier"],
            )
            x = [c["uniform_load_multiplier"] for c in rows]
            ax.plot(
                x,
                [c["reference"]["nonlinear_hinge_chain_prediction_m"] * 1000 for c in rows],
                color=color,
                linestyle="--" if rows[0]["split"] == "validation" else "-",
                label=f"{length * 1000:g} mm / {rows[0]['split']}",
            )
            ax.scatter(x, [c["observed_sag_m"] * 1000 for c in rows], color=color, s=22)
        ax.set(
            title=f"Equivalent stiffness ×{e:g}", xlabel="Uniform load / nominal load", ylabel="Tip sag (mm)"
        )
        ax.grid(alpha=0.15)
    axes[0].legend(frameon=False, fontsize=8)
    fig.suptitle("Lines: frozen reference world · dots: Isaac physics · dashed: reserved spans")
    fig.tight_layout()
    fig.savefig(dest / "response-curves.png", dpi=160)
    plt.close(fig)
    stats = []
    for split in ["fit", "validation"]:
        subset = [c for c in cases if c["split"] == split]
        stats.append(
            {
                "split": split,
                "passed": sum(c["passed"] for c in subset),
                "count": len(subset),
                "max_error_percent": max(c["relative_error"] for c in subset) * 100,
                "max_error_um": max(
                    abs(c["observed_sag_m"] - c["reference"]["nonlinear_hinge_chain_prediction_m"])
                    for c in subset
                )
                * 1e6,
            }
        )
    (dest / "summary.json").write_text(json.dumps({"selection": selection, "stats": stats}, indent=2) + "\n")
    table = "".join(
        f"<tr><th>{s['split'].capitalize()}</th><td>{s['passed']} / {s['count']}</td>"
        f"<td>{s['max_error_percent']:.2f}%</td><td>{s['max_error_um']:.2f} µm</td></tr>"
        for s in stats
    )
    initial_case = max(range(len(val["cases"])), key=lambda i: val["cases"][i]["observed_sag_m"])
    options = "".join(
        f'<option value="{i:03d}" {"selected" if i == initial_case else ""}>'
        f"{c['length_m'] * 1000:g} mm · stiffness ×{c['modulus_multiplier']:g}"
        f" · load ×{c['uniform_load_multiplier']:g} · tip error {c['relative_error'] * 100:.2f}%</option>"
        for i, c in enumerate(val["cases"])
    )
    section = (
        """<section id="bending-fit"><p class="eyebrow">Assumed-world calibration / experiment 037</p>
<h2>Watch the cable follow the reference curves.</h2>
<p>We can build a simulation proof of concept around a declared cable model.
This experiment checks whether Isaac reproduces that model's static bending responses.
The reference properties remain assumptions; they are not measurements of the purchased cable.</p>
<video controls playsinline preload="metadata" poster="bending-fit/validation.jpg"
aria-label="Actual Isaac reserved-span bending batch">
<source src="bending-fit/validation-slow.mp4" type="video/mp4"></video>
<p>The left view shows independent test strips; the right shows a close-up.
The red curve is the reference shape, used for offline inspection only.
Isaac advances the cable under gravity and joint forces. No controller commands its shape.
Replay is slowed sixfold; the experiment covers one simulated second.</p>
<figure><img src="bending-fit/response-curves.png" loading="lazy" style="width:100%;height:auto"
alt="Reference bending curves and simulated points across four spans and three stiffness assumptions">
<figcaption>Five loads per span and stiffness. Solid lines are fitting spans; dashed lines are reserved
validation spans. Every selected-parameter case is shown.</figcaption></figure>
<div class="roadmap-table"><table><thead><tr><th>Cases</th><th>Passed / total</th>
<th>Largest relative error</th><th>Largest absolute error</th></tr></thead><tbody>"""
        + table
        + """</tbody></table></div>
<h3>Inspect each reserved case.</h3><label for="bending-case">Span, stiffness and distributed load</label>
<select id="bending-case">"""
        + options
        + """</select>
<figure><img id="bending-case-image" src="bending-fit/cases/case-INITIAL.jpg" loading="lazy"
style="width:100%;max-width:900px;height:auto" alt="Settled cable from the selected validation case">
<figcaption>Saved Isaac state after one simulated second. The red reference centerline is shifted
sideways beside the ribbon for visibility; its height and length are unchanged.</figcaption></figure>
<p>The acceptance check requires at most 5% tip-sag error and at most 1% of reference sag in
the final 50 ms of tip motion. Passing establishes this static response approximation only.
Full-length deformation, twist, damping transients, desk contact, suction and insertion
remain separate tests.</p>
<p>Both errors are reported because a large percentage can be a sub-micrometre difference
in the smallest bends. The original 5% relative-error criterion is unchanged.
<a href="bending-fit/fit-shape.json">Fitting centerline audit</a> ·
<a href="bending-fit/validation-shape.json">Validation centerline audit</a>.</p>
<p>The earlier gain-only attempts barely changed the error. Reducing numerical joint viscosity made
the static response much closer to the reference. Artificial body drag remains a settling aid;
neither damping term is identified real-cable behavior.</p>
<p>The first broad fit still missed four stiff, lightly loaded cases. Its
<a href="bending-fit/coarse-fit.json">complete results</a> and
<a href="bending-fit/fit-slow.mp4">90-case recording</a> remain available.
The refined run uses a smaller timestep; the acceptance criterion remains 5%.</p>
<h3>A measurement correction, followed by a fresh run.</h3>
<p>The first refined reports showed 29/30 passes in each split. A rotation conversion lost very small
angles when a single-precision quaternion's scalar rounded to one. The corrected endpoint calculation
normalizes the quaternion in double precision and rotates the offset directly. This changes scoring,
not the cable's physics settings. The earlier reports remain available:
<a href="bending-fit/fit-original-scorer.json">fitting</a> and
<a href="bending-fit/validation-original-scorer.json">validation</a>.</p>
<p>The results above come from complete repeat runs with corrected measurements, including a fresh
50 ms settling window. These repeat the same reserved cases; they are not an additional unseen test set.
Raw poses are archived, and a separate SciPy calculation checks every final sag and settling result:
<a href="bending-fit/fit-measurements.json">fitting audit</a> and
<a href="bending-fit/validation-measurements.json">validation audit</a>.</p>
<h3>Next: make the full cable behave under contact.</h3>
<div class="roadmap-table"><table><thead><tr><th>Bench</th><th>What we will check</th></tr></thead><tbody>
<tr><th>Full cable and stiffeners</th><td>200 mm profile, both terminal reinforcements, mass,
resting shape and mesh/timestep sensitivity. Short homogeneous strips are only the first check.</td></tr>
<tr><th>Desk and fingertip contact</th><td>Support height, penetration, force balance and controlled slip
across declared friction assumptions.</td></tr>
<tr><th>Flexible-cable pickup</th><td>Lift with exposed contacts kept clear; compare direct pickup and
presentation-fixture handoff. Detect retention and slip through camera and pad/pressure signals.</td></tr>
<tr><th>Insertion entry</th><td>Only after pickup: leading-edge visibility, bounded alignment,
edge collisions, buckling and stop/retract behavior.</td></tr></tbody></table></div>
<p>We can develop these inside the declared assumed world before buying hardware. Damping,
suction and damage behavior still need explicit models; static bending agreement does not supply them.
<a href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/research/cables/contact-qualification.md">
Read the contact-bench plan.</a></p>
<div class="download-list"><a href="bending-fit/selection.json"><strong>Frozen fitting decision</strong>
<span>Numerical settings and selection rationale, frozen before validation.</span></a>
<a href="bending-fit/validation.json"><strong>Every validation result</strong>
<span>Reserved spans, loads, stiffnesses and settling checks.</span></a>
<a href="bending-fit/fit.mp4"><strong>Watch the fitting batch</strong>
<span>Actual physics for the 30 refined fitting cases.</span></a>
<a href="bending-fit/validation-scene.zip"><strong>Download the Isaac benchmark scene</strong>
<span>Saved final USD state, numerical settings and results. No robot controller.</span></a>
<a href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/experiments/037-assumed-world-fit.md">
<strong>Methods and unsuccessful trials</strong>
<span>What changed, what passed and what remains.</span></a></div>
</section>"""
    )
    section = section.replace("case-INITIAL.jpg", f"case-{initial_case:03d}.jpg")
    page = ROOT / "docs/hardware/custom-gripper.html"
    html = page.read_text()
    if '<section id="bending-fit">' in html:
        start = html.index('<section id="bending-fit">')
        end = html.index("</section>", start) + len("</section>")
        html = html[:start] + html[end:]
    html = html.replace('<section id="cable-physics">', section + '\n<section id="cable-physics">')
    if 'href="#bending-fit"' not in html:
        html = html.replace(
            '<main id="main" class="tool-page">',
            '<main id="main" class="tool-page"><p class="review-note">'
            '<a href="#bending-fit">New: watch the assumed-world bending fit and validation.</a></p>',
        )
    if 'src="bending-review.js"' not in html:
        html = html.replace("</head>", '<script src="bending-review.js" defer></script></head>')
    page.write_text(html)
    print(json.dumps(stats))


if __name__ == "__main__":
    main()
