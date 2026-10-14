"""Datasets over episodes written by EpisodeWriter (spec B5 input side).

Windows are gathered lazily: all observed rows of all episodes are concatenated (plus one zero row used as padding)
and each sample stores the WINDOW row indices of its window, so 15k episodes fit in memory. Only sensor arrays
(obs_row, clean_action, tau_th) and the offline labels are read; the truth file is read only for the per-episode
outcome used in offline metrics.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .training_config import WINDOW

ROW_DIM = 24


@dataclass
class EpisodeSet:
    rows: np.ndarray          # (R + 1, 24) float32, last row zeros (padding)
    win_idx: np.ndarray       # (N, WINDOW) int64 indices into rows, oldest first
    episode: np.ndarray       # (N,) episode number of each sample
    tick: np.ndarray          # (N,) row index within the episode
    tau_th: np.ndarray        # (N,) float32
    action: np.ndarray | None  # (N, 5) clean action chosen from this window (None for the last row)
    label: np.ndarray | None  # (N,) int64 contact class of the window's last row
    ids: list
    outcomes: list
    lengths: np.ndarray       # rows per episode


def list_episodes(run_dirs) -> list[Path]:
    eps = []
    for r in run_dirs:
        for h in sorted(Path(r).glob("ep*.json")):
            if (h.with_suffix(".npz")).exists():
                eps.append(h.with_suffix(""))
    return eps


def split_episodes(n: int, held_out: float = 0.2, seed: int = 0):
    perm = np.random.default_rng(seed).permutation(n)
    k = int(round(n * held_out))
    return np.sort(perm[k:]), np.sort(perm[:k])


def load(eps: list[Path], purpose: str = "bc") -> EpisodeSet:
    """purpose 'bc': one sample per action (window ending at the row the action was chosen from).
    purpose 'clf': one sample per row with a label (needs <id>.labels.npy from scripts/label_contact.py)."""
    rows, win, epn, ticks, th, act, lab, ids, outs, lens = [], [], [], [], [], [], [], [], [], []
    off = 0
    for k, base in enumerate(eps):
        E = np.load(str(base) + ".npz")
        r = E["obs_row"]; T = len(r)
        n = len(E["clean_action"]) if purpose == "bc" else T
        t = np.arange(n)
        w = t[:, None] + np.arange(-WINDOW + 1, 1)[None, :]
        idx = np.where(w >= 0, w + off, -1)
        rows.append(r); win.append(idx); epn.append(np.full(n, k)); ticks.append(t)
        th.append(np.full(n, float(E["tau_th"]), np.float32))
        if purpose == "bc":
            act.append(E["clean_action"][:n])
        else:
            lab.append(np.load(str(base) + ".labels.npy")[:n].astype(np.int64))
        tr = np.load(str(base) + ".truth.npz")
        outs.append(str(tr["outcome"])); ids.append(base.name); lens.append(T)
        off += T
    R = np.concatenate(rows + [np.zeros((1, ROW_DIM), np.float32)]).astype(np.float32)
    W = np.concatenate(win); W[W < 0] = len(R) - 1
    return EpisodeSet(R, W.astype(np.int64), np.concatenate(epn), np.concatenate(ticks), np.concatenate(th),
                      np.concatenate(act).astype(np.float32) if act else None,
                      np.concatenate(lab) if lab else None, ids, outs, np.asarray(lens))


def row_stats(rows: np.ndarray):
    """Per-channel mean and std over the real rows (the padding row excluded)."""
    r = rows[:-1]
    return r.mean(0).astype(np.float32), (r.std(0) + 1e-6).astype(np.float32)


def headers(eps: list[Path]) -> list[dict]:
    return [json.load(open(str(b) + ".json")) for b in eps]
