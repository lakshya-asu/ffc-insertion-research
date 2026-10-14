"""Per-tick 6-class contact labels from the truth files (spec B4, report section 4.2). Offline only.

usage: python scripts/label_contact.py out/episodes/<run> [more runs]

Writes <id>.labels.npy (int8, one label per obs row, T + 1 entries) next to each <id>.truth.npz and prints class
counts. Classes (training_config.CONTACT_CLASSES), first rule that matches wins:
  seated   depth >= 0.9 x INSERT_DEPTH and every pin within 0.6 mm of a cavity centre (Bench.seated)
  free     no header-housing contact
  beside   in contact and the worst pin is outside the housing footprint (worst > HOUSING_T/2 - 0.2 mm, the
           F-miss rule of Bench.outcome)
  surface  in contact with depth at or above the chamfer bottom (pins on the housing top or in the entry chamfer)
  wedged   depth past the chamfer, not seated, |depth change| under WEDGE_V per tick for the last 5 ticks
  entry    depth past the chamfer and moving
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
from cell_twin import scene as S  # noqa: E402
from cell_twin.training_config import CONTACT_CLASSES  # noqa: E402

FREE, SURFACE, BESIDE, ENTRY, WEDGED, SEATED = range(6)
WEDGE_V = 0.00002          # 0.02 mm per tick = 0.6 mm/s
WEDGE_TICKS = 5
assert CONTACT_CLASSES == ["free", "surface", "beside", "entry", "wedged", "seated"]


def label_truth(T) -> np.ndarray:
    depth = T["depth"].astype(float); pins = T["pins_in"].astype(bool); worst = T["worst"].astype(float)
    contact = T["n_contact_socket"] > 0
    n = len(depth)
    dv = np.abs(np.diff(depth, prepend=depth[0]))
    still = np.zeros(n, bool)
    for t in range(WEDGE_TICKS, n):
        still[t] = (dv[t - WEDGE_TICKS + 1:t + 1] < WEDGE_V).all()
    lab = np.full(n, ENTRY, np.int8)
    past = depth > S.CAVITY_CHAMFER
    lab[past & still] = WEDGED
    lab[~past] = SURFACE
    lab[worst > S.HOUSING_T / 2 - 0.0002] = BESIDE
    lab[~contact] = FREE
    lab[(depth >= 0.9 * S.INSERT_DEPTH) & pins] = SEATED
    return lab


def main():
    runs = [Path(p) for p in sys.argv[1:]] or sorted((ROOT / "out" / "episodes").iterdir())
    counts = np.zeros(6, int); n_ep = 0
    for run in runs:
        for tf in sorted(run.glob("ep*.truth.npz")):
            T = np.load(tf)
            if "depth" not in T:          # exception episode
                continue
            lab = label_truth(T)
            np.save(tf.with_name(tf.name.replace(".truth.npz", ".labels.npy")), lab)
            counts += np.bincount(lab, minlength=6); n_ep += 1
    print(f"labelled {n_ep} episodes, {counts.sum()} ticks")
    for c, k in zip(CONTACT_CLASSES, counts):
        print(f"  {c:8s} {k:8d}  {100 * k / max(counts.sum(), 1):5.1f} %")


if __name__ == "__main__":
    main()
