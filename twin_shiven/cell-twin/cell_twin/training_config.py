"""Shared training interface (spec docs/SPEC-training-v1.md). Owned by agent A; agent B imports from here.

Everything randomised is drawn once per episode by `Randomisation.sample` and recorded with `to_dict()` in the
episode header. Nothing here reads simulator truth.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent

TICK_HZ = 30
WINDOW = 16                      # ticks of history in the observation
ACTION_BOUNDS = dict(lateral_m=0.0003, push_m=0.0005, rot_rad=np.radians(0.5))   # per tick, tanh-bounded
OBS_LAYOUT = [                   # per tick, in this order; the window is the last WINDOW ticks flattened, oldest first
    ("tool_pose_rel_onset", 6),  # xyz (m) + rotvec (rad) of the measured tool pose relative to the pose at contact onset
    ("tau_ext", 6),              # measured torque minus model torque (gravity feedforward + PD on commanded error)
    ("tracking_err", 6),         # q_cmd - q_measured
    ("last_action", 5),          # the previous tick's action in ACTION_BOUNDS units
    ("time_since_onset", 1),     # seconds, 0 before onset
]
OBS_DIM = 24 * WINDOW
ACTION_DIM = 5                   # dx, dy, dz, droll, dpitch in the tool frame (no yaw: the pick error has none)
CONTACT_CLASSES = ["free", "surface", "beside", "entry", "wedged", "seated"]
OUTCOMES = ["S", "F-partial", "F-jam", "F-miss", "X"]

# scale of one action unit per component, in the order of the action vector (m, m, m, rad, rad)
ACTION_SCALE = np.array([ACTION_BOUNDS["lateral_m"], ACTION_BOUNDS["lateral_m"], ACTION_BOUNDS["push_m"],
                         ACTION_BOUNDS["rot_rad"], ACTION_BOUNDS["rot_rad"]])

# curriculum stages: (header tilt range deg, header slip range mm)
CURRICULUM = {1: ((0.0, 4.0), (0.0, 2.0)), 2: ((0.0, 8.0), (0.0, 4.0)), 3: ((0.0, 10.0), (0.0, 6.0))}

PROXIMAL = slice(0, 3)
WRIST = slice(3, 6)
TORQUE_NOISE_FILE = ROOT / "out" / "torque_noise.json"
TORQUE_NOISE_FALLBACK_NM = 0.05


def measured_torque_noise(arm: str = "right") -> np.ndarray:
    """Per-joint torque noise std (Nm) measured by scripts/torque_noise.py; 0.05 Nm each if the file is absent."""
    if TORQUE_NOISE_FILE.exists():
        return np.asarray(json.load(open(TORQUE_NOISE_FILE))[arm]["std_nm"], dtype=float)
    return np.full(6, TORQUE_NOISE_FALLBACK_NM)


def _per_joint(rng, prox, wrist):
    """Uniform per joint: proximal joints 1-3 in `prox`, wrist joints 4-6 in `wrist`."""
    return np.concatenate([rng.uniform(*prox, 3), rng.uniform(*wrist, 3)])


@dataclass
class Randomisation:
    """One episode's draw. Defaults are the nominal bench (no randomisation)."""
    kp_scale: np.ndarray = field(default_factory=lambda: np.ones(6))
    kd_scale: np.ndarray = field(default_factory=lambda: np.ones(6))
    torque_scale: np.ndarray = field(default_factory=lambda: np.ones(6))
    torque_bias: np.ndarray = field(default_factory=lambda: np.zeros(6))
    torque_noise_std: np.ndarray = field(default_factory=lambda: measured_torque_noise())
    obs_latency_ticks: int = 1
    act_latency_ticks: int = 0
    part_friction: float = 0.4
    contact_stiffness_scale: float = 1.0
    female_tilt_deg: float = 0.0       # signed, about the housing's pinch axis
    header_tilt_deg: float = 0.0       # signed, about the header's pinch axis (tool x)
    header_slip_mm: float = 0.0        # signed, across the row (tool y)
    torque_dropout: bool = False
    curriculum_stage: int = 0          # 0 = not drawn from a curriculum

    @classmethod
    def sample(cls, rng: np.random.Generator, stage: int = 2) -> "Randomisation":
        (t_lo, t_hi), (s_lo, s_hi) = CURRICULUM[stage]
        sign = lambda: float(rng.choice([-1.0, 1.0]))
        return cls(
            kp_scale=_per_joint(rng, (0.9, 1.1), (0.8, 1.2)),
            kd_scale=_per_joint(rng, (0.9, 1.1), (0.8, 1.2)),
            torque_scale=rng.uniform(0.85, 1.15, 6),
            torque_bias=_per_joint(rng, (-0.2, 0.2), (-0.05, 0.05)),
            torque_noise_std=measured_torque_noise() * rng.uniform(0.5, 2.0, 6),
            obs_latency_ticks=int(rng.integers(0, 3)),
            act_latency_ticks=int(rng.integers(0, 3)),
            part_friction=float(rng.uniform(0.2, 0.8)),
            contact_stiffness_scale=float(rng.uniform(0.7, 1.3)),
            female_tilt_deg=sign() * float(rng.uniform(0.0, 2.0)),
            header_tilt_deg=sign() * float(rng.uniform(t_lo, t_hi)),
            header_slip_mm=sign() * float(rng.uniform(s_lo, s_hi)),
            torque_dropout=bool(rng.random() < 0.2),
            curriculum_stage=stage,
        )

    def to_dict(self) -> dict:
        return {k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in asdict(self).items()}

    @classmethod
    def from_dict(cls, d: dict) -> "Randomisation":
        arrays = ("kp_scale", "kd_scale", "torque_scale", "torque_bias", "torque_noise_std")
        return cls(**{k: (np.asarray(v, dtype=float) if k in arrays else v) for k, v in d.items()})
