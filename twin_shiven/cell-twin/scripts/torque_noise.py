"""Torque sensing noise from static holds in the recorded episodes.

A static hold is a run of at least MIN_TICKS ticks in which every arm joint moves less than SPEED_MAX rad per tick.
Inside each hold the effort std per joint, after removing a straight-line trend (the slow load ramp of a push held
against contact), is the sensing noise. The pooled std per joint is sqrt(mean of the per-window variances), weighted
by window length; this is `std_nm`, used by the bench. Also written: the raw std (no detrend, an upper bound) and the
tick-to-tick estimate std(diff)/sqrt(2) (a lower bound: repeated frames in the log deflate it).

usage: python scripts/torque_noise.py [--min-ticks 15] [--speed 0.002]
writes out/torque_noise.json
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
EPS = ROOT / "inputs" / "nuc" / "episodes" / "objects"
ARMS = {"left": slice(0, 6), "right": slice(7, 13)}     # columns of observation.state / observation.effort


def hold_windows(q: np.ndarray, speed_max: float, min_ticks: int) -> list[tuple[int, int]]:
    """[start, end) tick ranges where every joint's per-tick speed stays under speed_max for >= min_ticks ticks."""
    still = np.all(np.abs(np.diff(q, axis=0)) < speed_max, axis=1)    # still[t] covers ticks t -> t+1
    out, t = [], 0
    while t < len(still):
        if still[t]:
            s = t
            while t < len(still) and still[t]:
                t += 1
            if t - s + 1 >= min_ticks:
                out.append((s, t + 1))
        else:
            t += 1
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--min-ticks", type=int, default=15)
    ap.add_argument("--speed", type=float, default=0.002)
    a = ap.parse_args()
    files = sorted(EPS.glob("*/frames.npz"))
    result = {"episodes": len(files), "speed_max_rad_per_tick": a.speed, "min_ticks": a.min_ticks}
    for arm, cols in ARMS.items():
        sq, sq_raw, sq_diff, n_ticks, n_win, med = np.zeros(6), np.zeros(6), np.zeros(6), 0, 0, []
        for f in files:
            z = np.load(f)
            q, e = z["observation.state"][:, cols], z["observation.effort"][:, cols]
            for s, t in hold_windows(q, a.speed, a.min_ticks):
                w = e[s:t]
                x = np.vstack([np.arange(len(w)), np.ones(len(w))]).T
                r = w - x @ np.linalg.lstsq(x, w, rcond=None)[0]
                sq += r.var(axis=0) * len(w); sq_raw += w.var(axis=0) * len(w)
                sq_diff += np.diff(w, axis=0).var(axis=0) / 2 * len(w)
                n_ticks += len(w); n_win += 1
                med.append(r.std(axis=0))
        k = max(n_ticks, 1)
        std = np.sqrt(sq / k)
        result[arm] = {"std_nm": std.round(4).tolist(), "raw_std_nm": np.sqrt(sq_raw / k).round(4).tolist(),
                       "diff_std_nm": np.sqrt(sq_diff / k).round(4).tolist(),
                       "median_window_std_nm": np.median(med, axis=0).round(4).tolist(),
                       "windows": n_win, "ticks": n_ticks}
        print(f"{arm}: {n_win} windows, {n_ticks} ticks, detrended std (Nm) {std.round(3)}, raw {np.sqrt(sq_raw / k).round(3)}, median window std {np.median(med, axis=0).round(3)}")
    (ROOT / "out").mkdir(exist_ok=True)
    json.dump(result, open(ROOT / "out" / "torque_noise.json", "w"), indent=1)
    print("wrote out/torque_noise.json")


if __name__ == "__main__":
    main()
