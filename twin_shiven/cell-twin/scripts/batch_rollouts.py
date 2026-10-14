"""Track-mode free-plug replay of the decisively labelled policy rollouts.

The sim arms follow the recorded measured joints; the plug and socket interact.
A good twin seats the plug on the 'satisfied' rollouts and not on the 'failed' ones.
writes out/batch_rollouts.tsv
"""
from __future__ import annotations
import json, subprocess, sys
from pathlib import Path
ROOT = Path(__file__).resolve().parent.parent
EPS = ROOT / "inputs" / "nuc" / "rollouts_as_episodes"
PY = ROOT / ".venv" / "bin" / "python"
labels = sys.argv[1:] or ["satisfied", "failed"]
rows = []
for d in sorted(EPS.iterdir()):
    meta = json.load(open(d / "source.json"))
    if meta["judgment"] not in labels:
        continue
    out = subprocess.run([str(PY), str(ROOT / "scripts" / "replay.py"), d.name, "--mode", "track", "--free-plug",
                          "--eps-root", str(EPS)], capture_output=True, text=True).stdout
    seat = next((l for l in out.splitlines() if l.startswith("SEAT:")), "SEAT: ? max depth nan mm of 7.0  final depth nan mm")
    toks = seat.split()
    rows.append((d.name[:8], meta["checkpoint_id"][:13], meta["judgment"], meta["frames"], toks[1], toks[4], toks[toks.index("final") + 2]))
    print("\t".join(map(str, rows[-1])), flush=True)
with open(ROOT / "out" / "batch_rollouts.tsv", "w") as f:
    f.write("rollout\tcheckpoint\tjudgment\tframes\tsim_seated\tmax_depth_mm\tfinal_depth_mm\n")
    for r in rows:
        f.write("\t".join(map(str, r)) + "\n")
tp = sum(1 for r in rows if r[2] == "satisfied" and r[4] == "yes"); fn = sum(1 for r in rows if r[2] == "satisfied" and r[4] == "no")
tn = sum(1 for r in rows if r[2] == "failed" and r[4] == "no"); fp = sum(1 for r in rows if r[2] == "failed" and r[4] == "yes")
print(f"satisfied: sim seated {tp}, not {fn}   |   failed: sim not seated {tn}, seated {fp}")
