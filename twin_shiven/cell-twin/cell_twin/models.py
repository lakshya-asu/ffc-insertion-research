"""Models (spec B5). Both carry their input normalisation as buffers, so they take raw env observations.

TorqueMLP   flattened 16-tick window (OBS_DIM) + tau_th -> 3 x 256 MLP -> ACTION_DIM, tanh (action units)
ContactCNN  16-tick window as 24 channels x 16 ticks -> 3 x Conv1d(64, k=3) -> global average pool -> 6 logits
"""
from __future__ import annotations

import torch
from torch import nn

from .training_config import ACTION_DIM, CONTACT_CLASSES, OBS_DIM, WINDOW

ROW_DIM = OBS_DIM // WINDOW


class TorqueMLP(nn.Module):
    def __init__(self, hidden: int = 256, layers: int = 3):
        super().__init__()
        self.register_buffer("row_mean", torch.zeros(ROW_DIM)); self.register_buffer("row_std", torch.ones(ROW_DIM))
        dims = [OBS_DIM + 1] + [hidden] * layers
        mods = []
        for a, b in zip(dims[:-1], dims[1:]):
            mods += [nn.Linear(a, b), nn.ReLU()]
        self.net = nn.Sequential(*mods, nn.Linear(hidden, ACTION_DIM))

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        w = obs[:, :OBS_DIM].view(-1, WINDOW, ROW_DIM)
        w = ((w - self.row_mean) / self.row_std).reshape(-1, OBS_DIM)
        th = (obs[:, OBS_DIM:OBS_DIM + 1] - 5.0) / 0.6
        return torch.tanh(self.net(torch.cat([w, th], 1)))


class ContactCNN(nn.Module):
    def __init__(self, ch: int = 64, k: int = 3):
        super().__init__()
        self.register_buffer("row_mean", torch.zeros(ROW_DIM)); self.register_buffer("row_std", torch.ones(ROW_DIM))
        self.conv = nn.Sequential(nn.Conv1d(ROW_DIM, ch, k, padding=k // 2), nn.ReLU(),
                                  nn.Conv1d(ch, ch, k, padding=k // 2), nn.ReLU(),
                                  nn.Conv1d(ch, ch, k, padding=k // 2), nn.ReLU())
        self.head = nn.Linear(ch, len(CONTACT_CLASSES))

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        w = obs[:, :OBS_DIM].view(-1, WINDOW, ROW_DIM)
        w = ((w - self.row_mean) / self.row_std).transpose(1, 2)
        return self.head(self.conv(w).mean(-1))


def set_norm(model: nn.Module, mean, std):
    model.row_mean.copy_(torch.as_tensor(mean)); model.row_std.copy_(torch.as_tensor(std))


def save(model: nn.Module, path, **meta):
    torch.save({"kind": type(model).__name__, "state_dict": model.state_dict(), "meta": meta}, path)


def load(path) -> nn.Module:
    ck = torch.load(path, map_location="cpu", weights_only=False)
    m = {"TorqueMLP": TorqueMLP, "ContactCNN": ContactCNN}[ck["kind"]]()
    m.load_state_dict(ck["state_dict"]); m.eval()
    return m
