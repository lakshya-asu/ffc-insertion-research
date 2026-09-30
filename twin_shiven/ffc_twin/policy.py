"""Learned controller, rung 1: behaviour cloning of the expert from its recorded packets.

The expert is a small state machine (align, approach, advance, retract), so its next move depends on the recent
past, not only on the current packet. The network therefore sees the last HISTORY packets stacked (31 x HISTORY
numbers) and outputs the six action values in [-1, 1]. Nothing else enters: no phase label, no truth.
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

HISTORY = 8
OBS_DIM = 31
ACT_DIM = 6


def stack_history(obs_seq: np.ndarray, k: int = HISTORY) -> np.ndarray:
    """(T, 31) -> (T, 31*k): row t holds packets t-k+1 .. t, with the first packet repeated before the start."""
    T = obs_seq.shape[0]
    pad = np.repeat(obs_seq[:1], k - 1, axis=0)
    padded = np.concatenate([pad, obs_seq], axis=0)
    return np.stack([padded[t : t + k].reshape(-1) for t in range(T)], axis=0)


class HistoryBuffer:
    """Keeps the last k packets at run time, mirroring stack_history."""

    def __init__(self, k: int = HISTORY):
        self.k, self.buf = k, []

    def reset(self):
        self.buf = []

    def push(self, vec: np.ndarray) -> np.ndarray:
        if not self.buf:
            self.buf = [vec.copy()] * self.k
        self.buf.append(vec.copy()); self.buf = self.buf[-self.k :]
        return np.concatenate(self.buf)


def build_mlp(in_dim: int, hidden: int = 256, layers: int = 3):
    import torch.nn as nn

    mods, d = [], in_dim
    for _ in range(layers):
        mods += [nn.Linear(d, hidden), nn.GELU()]
        d = hidden
    mods += [nn.Linear(d, ACT_DIM), nn.Tanh()]
    return nn.Sequential(*mods)


class BCPolicy:
    """Runtime wrapper: load weights + normalisation, act on the observation dict like the expert does."""

    def __init__(self, model_dir: str | Path, device: str = "cpu"):
        import torch

        d = Path(model_dir)
        meta = json.loads((d / "meta.json").read_text())
        self.mean = np.asarray(meta["obs_mean"], np.float32); self.std = np.asarray(meta["obs_std"], np.float32)
        self.k = meta["history"]
        self.net = build_mlp(OBS_DIM * self.k, meta["hidden"], meta["layers"])
        self.net.load_state_dict(torch.load(d / "weights.pt", map_location=device)); self.net.eval()
        self.torch, self.device = torch, device
        self.hist = HistoryBuffer(self.k)
        self.retries = 0   # so capture_region can report it like the expert's
        self.phase = "bc"

    def reset(self):
        self.hist.reset()

    def act(self, obs: dict) -> np.ndarray:
        from .env import TwinEnv

        x = (self.hist.push(TwinEnv.vector(obs).astype(np.float32)) - np.tile(self.mean, self.k)) / np.tile(self.std, self.k)
        with self.torch.no_grad():
            a = self.net(self.torch.as_tensor(x, device=self.device)[None])[0].cpu().numpy()
        return np.clip(a, -1, 1)
