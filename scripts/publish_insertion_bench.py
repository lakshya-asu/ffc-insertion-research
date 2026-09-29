"""Publish completed insertion commissioning evidence without upgrading its claims."""

import argparse
import html
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--library", type=Path, required=True)
    p.add_argument("runs", nargs="+")
    a = p.parse_args()
    cards, summary = [], []
    for name in a.runs:
        directory = a.library / name
        report = json.loads((directory / "report.json").read_text())
        score = json.loads((directory / "score.json").read_text())
        clearance = json.loads((directory / "clearance.json").read_text())
        trace = json.loads((directory / "sensor-trace.json").read_text())
        rows = [r for r in trace if r["feed_observation"]]
        fig, axes = plt.subplots(2, 1, figsize=(10, 6), layout="constrained")
        if rows:
            t = [r["time_s"] for r in rows]
            axes[0].plot(t, [r["feed_command"]["target_m"] * 1000 for r in rows], label="Commanded feed")
            axes[0].plot(
                t, [r["feed_observation"]["travel_m"] * 1000 for r in rows], label="Encoder equivalent"
            )
            axes[1].plot(
                t, [r["feed_observation"]["fixture_force_n"] for r in rows], label="Fixture load proxy"
            )
            axes[1].axhline(0.25, color="darkred", linestyle="--", label="Exploratory stop setting")
        axes[0].set_ylabel("Feed / mm")
        axes[1].set_ylabel("Force / N")
        axes[1].set_xlabel("Simulation time / s")
        for ax in axes:
            ax.legend()
            ax.grid(alpha=0.15)
        fig.savefig(directory / "trace.png", dpi=130)
        plt.close(fig)
        cmd = report["final"].get("feed_command") or report["final"]["command"]
        summary.append(
            {
                "run": name,
                "case": report["case"],
                "orientation": report.get("tool_orientation", "inline"),
                "sampled_bounds_separate": clearance["all_sampled_bounds_separate"],
                "state": cmd["state"],
                "tip_entered_assumed_channel": score["tip_entered_assumed_channel"],
                "seating_verified": False,
            }
        )
        cards.append(f'''<article id="{name}"><h2>{html.escape(report["case"])}</h2>
<p>Tool orientation: {html.escape(report.get("tool_orientation", "inline"))}. 
Sampled CAD/fixture bounds separate: <b>{clearance["all_sampled_bounds_separate"]}</b>. 
This is not swept robot clearance.</p>
<p>Controller: <b>{html.escape(cmd["state"])}</b>. {html.escape(cmd["reason"])}</p>
<p>Offline tip entry into the assumed channel: <b>{score["tip_entered_assumed_channel"]}</b>.
This checks the final leading-edge corners; it does not verify seating or latching.</p>
<video controls playsinline preload="metadata" poster="{name}/poster.jpg" src="{name}/video.mp4"></video>
<img src="{name}/trace.png" alt="Measured feed and fixture load versus time" loading="lazy">
<p><a href="{name}/replay.usdz">Native Isaac replay</a> · <a href="{name}/score.json">Offline score</a> ·
<a href="{name}/sensor-trace.json">Actor observations and commands</a> ·
<a href="{name}/clearance.json">Clearance audit</a> ·
<a href="{name}/rerun.tar.gz">Frozen rerun bundle</a> ·
<a href="{name}/report.json">Run assumptions</a> · <a href="{name}/manifest.json">Archive hashes</a>
</p></article>''')
    (a.library / "insertion-suite.json").write_text(json.dumps(summary, indent=2) + "\n")
    (a.library / "insertion.html").write_text(
        """<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Side-entry contact trials</title>
<style>body{font:17px/1.65 system-ui;max-width:1100px;margin:auto;padding:40px 24px;background:#f4f3ed;
color:#253c37}h1{font-size:clamp(34px,5vw,56px);line-height:1.1;letter-spacing:-.04em}a{color:#176b57}
article{padding:24px 0;border-top:1px solid #bbc8c0}video,img{width:100%}</style>
<a href="./#mounted">← Motion library</a><h1>Testing the sideways feed.</h1>
<p>The flexible cable is pinched, then pushed toward a small channel in Isaac. The controller uses
pad loads, a feed encoder equivalent and an ideal instrumented-fixture load proxy. It never reads
cable pose. The cable has no attachment to the gripper.</p>
<p>This is an isolated contact pilot with a prealigned starting placement. The channel is an exploratory
0.6 mm gap, 12 mm width and 4 mm length. It is not the Raspberry Pi connector model. Only the tool pads
have collision shapes; whole-tool collisions are not enabled. Per-mesh bounds are audited separately.
No FR3, camera servo, ROS motion or latch
is involved. The video cameras are for review and the housing can obscure the contact region.</p>
<p>The first inline setup pointed the cable tip back toward the housing and failed the conservative
clearance review.
<a href="insertion-inline.html">Its recordings and failure evidence are retained separately.</a>
The corrected side grip rotates the tool beside the cable
and grips the insulated body 12 mm behind the tip. The camera view improves, but this does not
qualify a complete FR3 approach.</p>
<h2>What counts as evidence</h2><p>A 4 mm commanded feed is not insertion success. An independent
geometry scorer checks whether the leading-edge corners entered the assumed channel. Even that is
not seating. A 0.25 N load setting and 0.5 mm bounded retract are exploratory controller settings,
not damage limits. Missing grasp feedback stops the feed rather than opening the tool.</p>
<p>These cases test open entry, a blocked mouth and injected pad-signal loss. They are commissioning
trials, not a reliability estimate. Videos show actual physics; downloaded USDZ files replay recorded
state with physics disabled.</p>"""
        + "".join(cards)
        + """<h2>Connecting this to the Pi task</h2>
<p>Next come contact-model convergence and socket-specific collision geometry, with the full tool
included in swept clearance checks. Then we connect a verified camera alignment estimate to short FR3
corrections and the tested feed guards. Insertion completion must have independent visual/contact
seating evidence; latch operation remains its own skill.</p></html>"""
    )

    primary = a.runs[0]
    if (a.library / primary / "end-1.png").exists():
        page = a.library / "insertion.html"
        heading = "<h1>Testing the sideways feed.</h1>"
        figure = (
            f'<figure><img src="{primary}/end-1.png" alt="Side grip holding a cable end inside the '
            'assumed channel"><figcaption>Recorded-state replay in Isaac. The white channel is an '
            "exploratory contact fixture, not the Pi socket.</figcaption></figure>"
        )
        page.write_text(page.read_text().replace(heading, heading + figure))


if __name__ == "__main__":
    main()
