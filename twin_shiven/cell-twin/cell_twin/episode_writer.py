"""EpisodeWriter (spec B2): one episode = three files under out/episodes/<run>/

  <id>.npz         per-tick sensor and command arrays (what a policy may learn from)
  <id>.json        header: randomisation sample, curriculum stage, controller, seed, bench git hash, end reason
  <id>.truth.npz   per-tick truth (depth, pins_in, worst lateral pin error, plug-housing contact count and normal force,
                   centre pin lateral position in the housing frame) and the outcome. Grading and offline labels only.
  <id>.frames.npz  optional: the three cameras at 320 x 240 every k ticks (--frames k)

Index convention: row arrays (obs_row, q, q_cmd, tau, tau_model, tau_ext, onset, detector, travel and every truth
array) have T + 1 entries, entry 0 measured after reset. Action arrays (action, clean_action, applied_action, reward)
have T entries; action i was chosen from the observation window ending at obs_row i (`obs_index[i] = i`).
"""
from __future__ import annotations

import json
import os
import subprocess
from pathlib import Path

import mujoco
import numpy as np

from .env import InsertionEnv

ROOT = Path(__file__).resolve().parent.parent
CAMERAS = ("scene_cam", "left_wrist_cam", "right_wrist_cam")


def git_hash() -> str:
    try:
        h = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "cell_twin"], capture_output=True,
                               text=True).stdout.strip()
        return h + ("-dirty" if dirty else "")
    except OSError:
        return "unknown"


class EpisodeWriter:
    def __init__(self, run_dir: Path, frames_every: int = 0, size=(320, 240)):
        self.run_dir = Path(run_dir); self.run_dir.mkdir(parents=True, exist_ok=True)
        self.frames_every = frames_every
        self.size = size
        self._renderer = None
        self.frames: dict[str, list] = {}
        self.frame_ticks: list[int] = []

    def exists(self, ep_id: str) -> bool:
        return (self.run_dir / f"{ep_id}.json").exists()

    def on_tick(self, env: InsertionEnv):
        """Call after reset and after every step if frames are wanted."""
        if not self.frames_every or env.t % self.frames_every:
            return
        if self._renderer is None or self._renderer_model is not env.bench.m:
            self._renderer = mujoco.Renderer(env.bench.m, self.size[1], self.size[0]); self._renderer_model = env.bench.m
        for cam in CAMERAS:
            self._renderer.update_scene(env.bench.d, camera=cam)
            self.frames.setdefault(cam, []).append(self._renderer.render().copy())
        self.frame_ticks.append(env.t)

    def write(self, ep_id: str, env: InsertionEnv, header: dict, clean_actions=None, outcome: str | None = None):
        L, T = env.log, env.truth_log
        n = len(L["action"])
        act = np.asarray(L["action"], np.float32).reshape(n, -1)
        clean = np.asarray(clean_actions if clean_actions is not None else L["action"], np.float32).reshape(n, -1)
        arrays = dict(
            obs_row=np.asarray(env.rows, np.float32), obs_index=np.arange(n, dtype=np.int32),
            q=np.asarray(L["q"], np.float32), q_cmd=np.asarray(L["q_cmd"], np.float32),
            tau=np.asarray(L["tau"], np.float32), tau_model=np.asarray(L["tau_model"], np.float32),
            tau_ext=np.asarray(L["tau_ext"], np.float32), action=act, clean_action=clean,
            applied_action=np.asarray(L["applied_action"], np.float32).reshape(n, -1),
            reward=np.asarray(L["reward"], np.float32), onset=np.asarray(L["onset"], bool),
            detector=np.asarray(L["detector"], bool), travel=np.asarray(L["travel"], np.float32),
            tau_th=np.float32(env.tau_th))
        tmp = self.run_dir / f".{ep_id}.tmp.npz"
        np.savez_compressed(tmp, **arrays); os.replace(tmp, self.run_dir / f"{ep_id}.npz")
        truth = env.truth()
        outcome = outcome or truth["outcome"]
        tarr = dict(depth=np.asarray(T["depth"], np.float32), pins_in=np.asarray(T["pins_in"], bool),
                    worst=np.asarray(T["worst"], np.float32), n_contact_socket=np.asarray(T["n_contact_socket"], np.int16),
                    contact_force=np.asarray(T["contact_force"], np.float32), plug_lat=np.asarray(T["plug_lat"], np.float32),
                    outcome=np.array(outcome), final_depth=np.float32(truth["depth"]))
        tmp = self.run_dir / f".{ep_id}.truth.tmp.npz"
        np.savez_compressed(tmp, **tarr); os.replace(tmp, self.run_dir / f"{ep_id}.truth.npz")
        if self.frames:
            np.savez_compressed(self.run_dir / f"{ep_id}.frames.npz", ticks=np.asarray(self.frame_ticks),
                                **{c: np.asarray(v, np.uint8) for c, v in self.frames.items()})
            self.frames, self.frame_ticks = {}, []
        hdr = dict(header, id=ep_id, randomisation=env.rand.to_dict(), curriculum_stage=env.stage, seed=env.seed,
                   tau_th=env.tau_th, ticks=n, onset_tick=env.onset_tick, end_reason=env.end_reason,
                   bench_git=header.get("bench_git") or git_hash(), schema="episode-v1")
        tmp = self.run_dir / f".{ep_id}.tmp.json"
        json.dump(hdr, open(tmp, "w"), indent=1); os.replace(tmp, self.run_dir / f"{ep_id}.json")   # header last = complete
        return hdr
