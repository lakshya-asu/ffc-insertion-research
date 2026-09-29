"""Sensor-driven scripted expert (rung 0). Reads only the observation packet.

align: cancel the estimated error in y, z, roll, pitch, yaw (bounded per tick)
approach: advance along x until the wrench reports contact
advance: 0.2 mm per tick under a force ceiling
retract and shift: on a stall (force up, estimated depth not increasing) back off and move away from the wall
"""
from __future__ import annotations

import math

import numpy as np

from .env import ACTION_POS, ACTION_ROT
from .spec import DEFAULT, Spec


class Expert:
    def __init__(self, spec: Spec = DEFAULT, force_ceiling: float = 1.5, max_retries: int = 8):
        self.spec = spec
        self.force_ceiling = force_ceiling
        self.max_retries = max_retries
        self.reset()

    def reset(self):
        self.phase = "align"
        self.align_ticks = 0
        self.retries = 0
        self.stall = 0
        self.last_depth = -1.0
        self.retract_left = 0
        self.shift_dir = 0.0
        self.contacted = False
        self.est_f = None
        self.settled_ticks = 0

    def act(self, obs: dict) -> np.ndarray:
        est = obs["tip_estimate"]
        w = obs["fixture_wrench_n_nm"]
        visible = obs["tip_visible"][0] > 0.5
        fn = float(np.linalg.norm(w[:3]))
        a = np.zeros(6)
        if self.phase == "align":
            # proportional correction at 40 percent per tick: the estimate arrives 2 to 3 ticks late and is noisy,
            # so a full-gain correction hunts. Smooth the estimate and stop after it has settled for three ticks.
            self.est_f = est.copy() if self.est_f is None else 0.5 * self.est_f + 0.5 * est
            err = np.r_[0.0, self.est_f[1], self.est_f[2], self.est_f[3], self.est_f[4], self.est_f[5]]
            a[1:3] = np.clip(-0.4 * err[1:3] / ACTION_POS, -1, 1)
            a[3:6] = np.clip(-0.4 * err[3:6] / ACTION_ROT, -1, 1)
            self.align_ticks += 1
            settled = np.all(np.abs(err[1:3]) < 0.06e-3) and np.all(np.abs(err[3:6]) < math.radians(0.3))
            self.settled_ticks = self.settled_ticks + 1 if settled else 0
            if self.settled_ticks >= 3 or self.align_ticks > 40:
                self.phase = "approach"
            return a
        if self.phase == "retract":
            a[0] = -1.0
            a[1] = self.shift_dir
            self.retract_left -= 1
            if self.retract_left <= 0:
                self.phase = "advance"
                self.stall = 0
            return a
        # approach and advance share the forward motion; approach also keeps a light lateral servo while the tip is visible
        if fn > 0.3:
            self.contacted = True
        if self.phase == "approach":
            if visible:
                a[1] = float(np.clip(-est[1] / ACTION_POS, -0.5, 0.5))
                a[5] = float(np.clip(-est[5] / ACTION_ROT, -0.5, 0.5))
            if self.contacted:
                self.phase = "advance"
        depth = float(est[0])
        slipping = obs["tactile"][3] > 0.5 if "tactile" in obs else False
        stalled = (fn > 1.0 and (depth - self.last_depth) < 0.02e-3) or slipping
        self.last_depth = max(self.last_depth, depth)
        if self.phase == "advance" and stalled:
            self.stall += 1
            if self.stall >= 2 and self.retries < self.max_retries:
                self.retries += 1
                self.phase = "retract"
                self.retract_left = 2
                self.shift_dir = float(np.sign(w[1])) * 0.6 if abs(w[1]) > 0.2 else 0.0
                self.stall = 0
                a[0] = -1.0
                a[1] = self.shift_dir
                return a
        else:
            self.stall = 0
        a[0] = 1.0 if (fn < self.force_ceiling and not slipping) else 0.0   # never push against a slipping grip
        return a
