"""Free-plug dynamic replay of every pulled episode; one row per episode.

writes out/batch_seat.tsv: episode, kind, frames, real_seated, sim_seated, max_depth_mm, final_depth_mm
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
PY = ROOT / ".venv" / "bin" / "python"
PREINSERTION2 = "552d4ea6-1f26-4783-8244-c8ba3eef00fa"


def kind_of(meta):
    if meta["title"].startswith("Correction"):
        return "corr"
    return "t5" if meta.get("starting_pose") == PREINSERTION2 else "miss"


def main():
    rows = []
    for ep in sorted(p.name for p in EPS.iterdir()):
        meta = json.load(open(EPS / ep / "source.json"))
        kind = kind_of(meta)
        out = subprocess.run([str(PY), str(ROOT / "scripts" / "replay.py"), ep, "--mode", "dynamic", "--free-plug"],
                             capture_output=True, text=True).stdout
        seat = [l for l in out.splitlines() if l.startswith("SEAT:")]
        line = seat[0] if seat else "SEAT: ? max depth nan mm of 7.0  final depth nan mm"
        toks = line.split()
        sim_seated = toks[1]
        max_depth = toks[4]
        final_depth = toks[toks.index("final") + 2]
        real_seated = "yes" if kind == "t5" else ("no" if kind == "miss" else "corr")
        rows.append((ep[:8], kind, meta.get("frames"), real_seated, sim_seated, max_depth, final_depth))
        print("\t".join(map(str, rows[-1])), flush=True)
    with open(ROOT / "out" / "batch_seat.tsv", "w") as f:
        f.write("episode\tkind\tframes\treal_seated\tsim_seated\tmax_depth_mm\tfinal_depth_mm\n")
        for r in rows:
            f.write("\t".join(map(str, r)) + "\n")
    agree = sum(1 for r in rows if r[3] in ("yes", "no") and r[3] == r[4])
    scored = sum(1 for r in rows if r[3] in ("yes", "no"))
    print(f"agreement on seated/not: {agree}/{scored}")


if __name__ == "__main__":
    main()
