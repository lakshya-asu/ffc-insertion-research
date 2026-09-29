"""Publish recorded socket reference trials using the existing library layout."""

import argparse
import html
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    library = root / "docs/library"
    notes = {
        "socket-centered-002": "Box contacts: cable caught near their front faces; guard retracted.",
        "socket-offset-001": "Tapered housing guides, box contacts: entry observed in one offset trial.",
        "socket-square-offset-001": "Paired comparison with the housing slopes removed: guard retracted.",
        "socket-tapered-001": "INVALID: convex cooking enlarged the contacts. No feed occurred.",
        "socket-tapered-002": "INVALID: neighbouring convex contacts collided despite the cooking change.",
        "socket-tapered-003": "Analytic ramp/land contacts; empty-socket check passed.",
    }
    articles = []
    for name in args.runs:
        run = library / name
        report = json.loads((run / "report.json").read_text())
        score = json.loads((run / "score.json").read_text())
        rows = [r for r in json.loads((run / "sensor-trace.json").read_text()) if r["feed_observation"]]
        fig, axes = plt.subplots(2, 1, figsize=(10, 5), layout="constrained")
        t = [r["time_s"] for r in rows]
        axes[0].plot(t, [r["feed_observation"]["travel_m"] * 1000 for r in rows], label="Measured feed")
        axes[0].plot(t, [r["feed_command"]["target_m"] * 1000 for r in rows], label="Commanded feed")
        axes[0].set_ylabel("Travel / mm")
        axes[1].plot(t, [r["feed_observation"]["fixture_force_n"] for r in rows], label="Fixture load proxy")
        axes[1].axhline(0.25, color="firebrick", linestyle="--", label="Exploratory stop threshold")
        axes[1].set_ylabel("Force / N")
        axes[1].set_xlabel("Simulation time / s")
        for ax in axes:
            ax.legend()
            ax.grid(alpha=0.15)
        fig.savefig(run / "trace.png", dpi=130)
        plt.close(fig)
        cmd = report["final"].get("feed_command") or report["final"]["command"]
        depth = score["final_geometry"]["tip_depth_range_m"]
        invocation = report["invocation"]
        articles.append(f'''<article><h2>{html.escape(name)}</h2>
<p>{html.escape(notes.get(name, ""))}</p>
<p>Contact shape: {html.escape(report["fixture"]["contact_shape"])}.</p>
<p>Lateral offset: {invocation["socket_offset_mm"]} mm. Height offset:
{invocation["socket_height_mm"]} mm. Tapered entry: {not invocation["square_entry"]}.</p>
<p>Controller: <b>{html.escape(cmd["state"])}</b>. {html.escape(cmd["reason"])}
Leading-edge depth: {depth[0] * 1000:.3f}–{depth[1] * 1000:.3f} mm.
Backstop overrun: {score.get("final_backstop_overrun_m", 0)*1e6:.2f} µm.
Final corners fit the assumed straight throat:
{score["final_geometry"]["tip_cross_section_fits_channel"]}.</p>
<video controls playsinline preload="metadata" poster="{name}/poster.jpg" src="{name}/video.mp4"></video>
<img loading="lazy" src="{name}/trace.png" alt="Feed travel and fixture contact load over simulation time">
<p><a href="{name}/replay.usdz">Isaac replay</a> ·
<a href="{name}/rerun.tar.gz">Frozen physics rerun bundle</a> ·
<a href="{name}/report.json">Assumptions and report</a> · <a href="{name}/score.json">Offline geometry</a> ·
<a href="{name}/manifest.json">Archive hashes</a></p></article>''')
    page = """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Socket entrance trials</title>
<style>body{font:17px/1.65 system-ui;max-width:1100px;margin:auto;padding:40px 24px;background:#f4f3ed;
color:#253c37}h1{font-size:clamp(34px,5vw,56px);line-height:1.1;letter-spacing:-.04em}a{color:#176b57}
article{padding:24px 0;border-top:1px solid #bbc8c0}video,img{width:100%}</style>
<a href="./">← Motion library</a><h1>Into a tighter entrance.</h1>
<p>The cable is pinched on its insulated body and fed toward a side-entry socket reference in Isaac.
The 2.5 mm insertion depth comes from a Molex-family drawing. The model separates the open slider,
housing guides, backstop and 22 spring-supported contacts.</p>
<p>Throat width (11.65 mm), height (0.40 mm), guide length (0.30 mm), guide expansion (0.15 mm each side),
friction and contact springs are explicit assumptions. Early trials used spring-loaded boxes;
the corrected revision uses tapered contact noses. Neither model includes beam FEA or latch preload.
This is a local test fixture, not yet
registered into the Pi board assembly or driven by an FR3.</p>
<p>The controller reads pad loads, an encoder equivalent and a simulated instrumented-fixture load.
It does not read cable poses. Cameras here record the experiment; they do not align the cable.
Only the tool pads have collision shapes. Seating, full-tool clearance and damage are not verified.</p>
<p>Native USD replay includes cable, tool and contact motion with physics disabled. Rerun bundles
contain frozen simulation inputs; a fresh Docker build and independent rerun are not yet verified.</p>
<p><a href="https://www.molex.com/pdm_docs/sd/545482272_sd.pdf">Manufacturer reference drawing</a> ·
<a href="https://github.com/lakshya-asu/ffc-insertion-research/blob/main/research/connectors/pi-socket-contact.md">
Research and part-identity limits</a> · <a href="insertion.html">Previous generic-channel trials</a></p>
<p><a href="socket-diagnostics.json">Empty-socket collision diagnostic</a>: the analytic ramp model
reported no contacts during 200 steps; the rejected convex model produced neighbouring-contact collisions.</p>
<p>The first attempt crashed in native code after its first frame. The failed run is preserved locally;
a successful retry does not establish the cause or resolve runtime reliability.</p>"""
    figure = ""
    if (library / "socket-offset-001/end-1.png").exists():
        figure = (
            '<figure><img src="socket-offset-001/end-1.png" alt="Recorded socket entry">'
            "<figcaption>Native Isaac replay of the offset trial. This is the local socket fixture; "
            "the Pi board has not been integrated into this dynamic bench.</figcaption></figure>"
        )
    (library / "socket.html").write_text(page + figure + "".join(articles) + "</html>")


if __name__ == "__main__":
    main()
