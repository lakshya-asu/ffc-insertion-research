"""InsertionEnv: the bench wrapped as a gym-style environment for the contact-phase policy (spec B1).

Sensor-only rule. The observation, the reward, contact onset, the seat detector and termination use only what the
cell measures: right joint encoders (and the tool pose from forward kinematics of them), the measured motor torques
from `Bench.torques()`, the commanded joint targets and the policy's own actions. Simulator truth (seat depth, pin
positions, contact pairs, the sampled pick error, the outcome) goes into `info["truth"]` and is never read by the
env's own decisions. `Bench.reset` uses truth once for the start alignment; that is the bench's documented exception.

Observation: the last WINDOW feature rows (OBS_LAYOUT, 24 per tick) flattened oldest first, then one extra entry,
the episode's force threshold tau_th (Nm), so obs has OBS_DIM + 1 entries. Rows pass through an observation latency
queue of `obs_latency_ticks`; actions pass through an action latency queue of `act_latency_ticks`.

Expert driving: `ExpertProxy` exposes the subset of the Bench API the controllers use (step_tool, torques,
tool_pose, tool_travel, mark_start, hold, seated) and turns every step_tool call into one or more bounded env ticks,
so a monolithic controller runs through the same observation, reward and termination path as a learned policy.
`seated()` on the proxy is the env's seat detector, not truth.
"""
from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np

from . import scene as S
from . import controllers as C
from .bench import Bench, PickError
from .training_config import ACTION_DIM, ACTION_SCALE, OBS_DIM, WINDOW, Randomisation

ROOT = Path(__file__).resolve().parent.parent
START_EP = "09710851-e26c-43ef-b7ed-0a684c28e4a4"
ROW_DIM = 24

CONTACT_RISE_NM = 0.6        # same rule as controllers.CONTACT_RISE_NM: any joint's torque rise above the baseline
SEAT_DEPTH_M = S.INSERT_DEPTH
# travel since onset beyond the hand-built detector's seat band + 2 mm: pins are past the housing. The band is wide
# (onset = leaf engagement plus grasp compliance), so a tighter bound would end real seats.
OVER_TRAVEL_M = getattr(C, "SEAT_BAND", (0.0, SEAT_DEPTH_M + 0.0045))[1] + 0.002
OVER_FORCE_NM = 12.0         # safety bound on norm(tau_ext)
MAX_TICKS = 900           # 30 s: stage-2 searches that were still going at 600 ticks were mostly right, only slow (agent A report)

# stand-in seat detector (until controllers.seat_detector exists): the align_by_contact stall rule plus a travel band
STANDIN_BAND_M = (0.0070, 0.0105)   # travel since onset at true seats: 7.4 to 10.4 mm in the n=40 probe (provisional)
STANDIN_STALL_M = 0.0004            # travel over the last 8 ticks below this = stalled (align_by_contact rule)
STANDIN_RISE_NM = 1.5               # and the push is resisted (align_by_contact's wedge-check level)


def start_state() -> np.ndarray:
    """The seated last frame of the reference demo (the bench's default start)."""
    return np.load(ROOT / "inputs" / "nuc" / "episodes" / "objects" / START_EP / "frames.npz")["observation.state"][-1]


def rotvec(R: np.ndarray) -> np.ndarray:
    ang = math.acos(float(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    if ang < 1e-9:
        return np.zeros(3)
    ax = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return ax / (np.linalg.norm(ax) + 1e-12) * ang


def window(rows: np.ndarray, i: int) -> np.ndarray:
    """Rows i-WINDOW+1 .. i (zero rows before the episode start), flattened oldest first."""
    lo = i - WINDOW + 1
    w = rows[max(lo, 0):i + 1]
    if lo < 0:
        w = np.concatenate([np.zeros((-lo, rows.shape[1]), rows.dtype), w])
    return w.reshape(-1)


class EpisodeOver(Exception):
    """Raised inside a controller driven through ExpertProxy once the env terminates or truncates."""


class InsertionEnv:
    def __init__(self, start14: np.ndarray | None = None, max_ticks: int = MAX_TICKS, seat_detector=None,
                 k_progress: float = 1000.0, beta_force: float = 0.05, c_smooth: float = 0.01):
        self.start14 = start_state() if start14 is None else start14
        self.max_ticks = max_ticks
        # factory(bench) -> object with step(tau) -> bool, stepped once per tick (controllers.SeatDetector by default);
        # False = the stand-in stall rule below
        self.seat_detector = getattr(C, "SeatDetector", False) if seat_detector is None else seat_detector
        self.k_progress, self.beta_force, self.c_smooth = k_progress, beta_force, c_smooth
        self.bench: Bench | None = None
        self.on_tick = None                    # optional callback(env) after reset and every step (frame capture)

    # ---------------- gym API ----------------
    def reset(self, seed: int | None = None, randomisation: Randomisation | None = None, curriculum_stage: int = 1):
        self.rng = np.random.default_rng(seed)
        self.seed = seed
        self.rand = randomisation if randomisation is not None else Randomisation.sample(self.rng, curriculum_stage)
        self.stage = curriculum_stage
        r = self.rand
        self.pick = dict(header=PickError(skew_deg=float(r.header_tilt_deg), lateral_mm=(0.0, float(r.header_slip_mm))),
                         housing=PickError(skew_deg=float(r.female_tilt_deg)))
        # agent A's Bench applies the randomisation before its reset (gains, torque sensing, friction, leaf stiffness)
        # and owns the torque observation latency and the action latency queue; the env then only delays the
        # non-torque channels. Without Bench.apply the env applies both latencies itself.
        self.bench_latency = hasattr(Bench, "apply")
        kw = {"rand": r} if self.bench_latency else {}
        self.bench = Bench(self.start14, self.pick["header"], self.pick["housing"],
                           rng=np.random.default_rng(self.rng.integers(1 << 31)), **kw)
        self.tau_th = float(self.rng.uniform(4.0, 6.0))
        b = self.bench
        self.p0, R0 = b.tool_pose(); self.axis0 = R0[:, 2].copy(); self.R0 = R0
        self.ref_p, self.ref_R = self.p0.copy(), R0.copy()   # pose reference: start until onset, then onset pose
        taus = []
        for _ in range(5):                         # baseline: hold still five ticks and average the measured torque
            b.hold(); taus.append(b.torques())
        self.base = np.mean(taus, axis=0)
        self.tau_last = taus[-1]
        self.t = 0
        self.onset_tick = -1
        self.onset_travel = 0.0
        self.max_travel_onset = 0.0
        self.travel_hist = []
        self.detector_on = False
        self.last_action = np.zeros(ACTION_DIM)
        self.act_queue = [np.zeros(ACTION_DIM)] * (0 if self.bench_latency else int(r.act_latency_ticks))
        self.row_queue = []
        self.tau_queue = []
        self.rows = []                             # observed rows (after the latency queue)
        self.log = {k: [] for k in ("q", "q_cmd", "tau", "tau_model", "tau_ext", "action", "applied_action",
                                    "onset", "detector", "reward", "travel")}
        self.truth_log = {k: [] for k in ("depth", "pins_in", "worst", "n_contact_socket", "contact_force", "plug_lat")}
        self.done = False
        self.end_reason = ""
        self.det = self.seat_detector(b) if self.seat_detector else None
        self._measure(np.zeros(ACTION_DIM))
        if self.on_tick:
            self.on_tick(self)
        return self._obs(), self._info()

    def step(self, action):
        assert not self.done, "step after episode end"
        a = np.clip(np.asarray(action, dtype=float).reshape(ACTION_DIM), -1.0, 1.0)
        self.act_queue.append(a)
        applied = self.act_queue.pop(0)
        d = applied * ACTION_SCALE
        if hasattr(self.bench, "step_action"):
            self.bench.step_action(applied)
        else:
            self.bench.step_tool([d[0], d[1], d[2]], [d[3], d[4], 0.0])
        self.t += 1
        self.log["action"].append(a); self.log["applied_action"].append(applied)
        tau_ext = self._measure(a)
        # ---- reward and termination: observables only ----
        travel_onset = self.travel - self.onset_travel if self.onset_tick >= 0 else 0.0
        r = -0.01
        if self.onset_tick >= 0:
            new_max = min(max(self.max_travel_onset, travel_onset), SEAT_DEPTH_M)
            r += self.k_progress * (new_max - self.max_travel_onset)
            self.max_travel_onset = new_max
        tn = float(np.linalg.norm(tau_ext))
        r -= self.beta_force * max(0.0, tn - self.tau_th)
        r -= self.c_smooth * float(np.linalg.norm(a - self.last_action))
        self.last_action = a
        terminated = truncated = False
        if self.detector_on:
            r += 10.0; terminated = True; self.end_reason = "seat_detector"
        elif tn > OVER_FORCE_NM:
            r -= 5.0; terminated = True; self.end_reason = "over_force"
        elif self.onset_tick >= 0 and travel_onset > OVER_TRAVEL_M:
            r -= 5.0; terminated = True; self.end_reason = "over_travel"
        elif self.t >= self.max_ticks:
            truncated = True; self.end_reason = "truncated"
        self.log["reward"].append(r)
        self.done = terminated or truncated
        if self.on_tick:
            self.on_tick(self)
        return self._obs(), r, terminated, truncated, self._info()

    # ---------------- internals ----------------
    def _measure(self, a):
        b = self.bench
        tau = np.asarray(b.torques(), dtype=float)
        self.tau_last = tau
        self.tau_queue.append(tau)
        q = b.q_right(); q_cmd = np.asarray(b.target_q, dtype=float).copy()
        dofs = [b.dof[("right", i)] for i in range(6)]
        # model torque: what the PD must supply in free motion at the measured joints = gravity + Coriolis of the arm
        # model minus the gravity the YAM's feedforward already cancels (Menagerie models it as body gravcomp)
        tau_model = (b.d.qfrc_bias[dofs] - b.d.qfrc_gravcomp[dofs]).copy()
        tau_ext = tau - tau_model
        p, R = b.tool_pose()
        self.travel = float((p - self.p0) @ self.axis0)
        self.travel_hist.append(self.travel)
        self.detector_on = self._detect(tau)
        # contact onset: the seat detector's ContactMonitor (controllers' matched-filter torque rule plus slowed-by-contact
        # travel) when present, else the 0.6 Nm rise rule. The first onset fixes the pose reference for the episode.
        on = (self.det.mon.onset_travel is not None) if self.det is not None and hasattr(self.det, "mon") \
            else np.abs(tau - self.base).max() > CONTACT_RISE_NM
        if self.onset_tick < 0 and on:
            self.onset_tick = self.t; self.onset_travel = self.travel
            self.ref_p, self.ref_R = p.copy(), R.copy()
        dp = self.ref_R.T @ (p - self.ref_p)
        dr = rotvec(self.ref_R.T @ R)
        tso = (self.t - self.onset_tick) / 30.0 if self.onset_tick >= 0 else 0.0
        row = np.concatenate([dp, dr, tau_model, q_cmd - q, a, [tso]]).astype(np.float32)
        self.row_queue.append(row)
        lat = int(self.rand.obs_latency_ticks)
        obs = (self.row_queue[-1 - lat] if len(self.row_queue) > lat else self.row_queue[0]).copy()
        # tau_ext channel: delayed measured torque minus the model torque of the same (delayed) tick. With the
        # bench's latency the measured torque is already lat ticks old; without it, delay it here.
        tau_d = tau if self.bench_latency else (self.tau_queue[-1 - lat] if len(self.tau_queue) > lat else self.tau_queue[0])
        obs[6:12] = 0.0 if self.rand.torque_dropout else tau_d - obs[6:12]
        self.rows.append(obs)
        for k, v in (("q", q), ("q_cmd", q_cmd), ("tau", tau), ("tau_model", tau_model), ("tau_ext", tau_ext),
                     ("onset", self.onset_tick >= 0), ("detector", self.detector_on), ("travel", self.travel)):
            self.log[k].append(v)
        self._log_truth()
        return tau_ext

    def _detect(self, tau) -> bool:
        if self.det is not None:
            return bool(self.det.step(tau))
        if self.onset_tick < 0 or len(self.travel_hist) < 8:
            return False
        tr = self.travel - self.onset_travel
        stalled = (self.travel_hist[-1] - self.travel_hist[-8]) < STANDIN_STALL_M
        return STANDIN_BAND_M[0] <= tr <= STANDIN_BAND_M[1] and stalled and np.abs(tau - self.base).max() > STANDIN_RISE_NM

    def _obs(self) -> np.ndarray:
        rows = np.asarray(self.rows)
        return np.concatenate([window(rows, len(rows) - 1), [self.tau_th]]).astype(np.float32)

    # ---- truth, for grading and offline labels only ----
    def _log_truth(self):
        b = self.bench; m, d = b.m, b.d
        ok, worst = b.pins_in()
        sock = m.body("socket").id; plug = m.body("plug").id
        n = 0; f = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
            if {b1, b2} == {sock, plug}:
                n += 1
                ff = np.zeros(6); mujoco.mj_contactForce(m, d, i, ff); f += abs(ff[0])
        Ts = np.eye(4); Ts[:3, :3] = d.body("socket").xmat.reshape(3, 3); Ts[:3, 3] = d.body("socket").xpos
        tip = (np.linalg.inv(Ts) @ np.append(d.geom("pin_tip_2").xpos, 1))[:2]
        for k, v in (("depth", b.seat_depth()), ("pins_in", ok), ("worst", worst), ("n_contact_socket", n),
                     ("contact_force", f), ("plug_lat", tip)):
            self.truth_log[k].append(v)

    def truth(self) -> dict:
        b = self.bench
        ok, worst = b.pins_in()
        return dict(depth=b.seat_depth(), pins_in=bool(ok), worst=float(worst), outcome=b.outcome(),
                    header_tilt_deg=float(self.rand.header_tilt_deg), header_slip_mm=float(self.rand.header_slip_mm),
                    female_tilt_deg=float(self.rand.female_tilt_deg))

    def _info(self) -> dict:
        return {"t": self.t, "onset": self.onset_tick >= 0, "detector": self.detector_on, "end_reason": self.end_reason,
                "truth": self.truth()}


class ExpertProxy:
    """The Bench API a controller uses, routed through InsertionEnv ticks.

    Each step_tool call is converted to action units and split into ceil(max |u|) bounded ticks. With
    noise_sigma > 0 every tick executes clean + N(0, sigma) (DART) while the clean action is recorded."""

    def __init__(self, env: InsertionEnv, noise_sigma: float = 0.0, rng: np.random.Generator | None = None):
        self.env = env
        self.noise_sigma = noise_sigma
        self.rng = rng or np.random.default_rng(0)
        self.clean_actions: list[np.ndarray] = []
        self.target_q = env.bench.target_q

    def _tick(self, clean):
        if self.env.done:
            raise EpisodeOver
        a = clean + (self.rng.normal(0, self.noise_sigma, ACTION_DIM) if self.noise_sigma > 0 else 0.0)
        self.clean_actions.append(np.clip(clean, -1, 1))
        _, _, term, trunc, _ = self.env.step(a)
        if term or trunc:
            raise EpisodeOver

    def step_tool(self, dpos_tool, drot_tool, max_joint_step=0.02):
        u = np.array([dpos_tool[0], dpos_tool[1], dpos_tool[2], drot_tool[0], drot_tool[1]]) / ACTION_SCALE
        n = max(1, int(math.ceil(np.abs(u).max() - 1e-9)))
        for _ in range(n):
            self._tick(u / n)

    def hold(self):
        self._tick(np.zeros(ACTION_DIM))

    def torques(self):
        return self.env.tau_last.copy()

    def tool_pose(self):
        return self.env.bench.tool_pose()

    def tool_travel(self):
        return self.env.bench.tool_travel()

    def mark_start(self):
        self.env.bench.mark_start()

    def seated(self):
        return self.env.detector_on      # the env's seat detector (observables), never truth

    def __getattr__(self, name):
        raise AttributeError(f"ExpertProxy does not expose Bench.{name} (sensor-only)")


class SensorView:
    """Read-only bench view for per-tick policies (controllers.Expert.act): sensors and own commands only."""

    def __init__(self, bench):
        self._b = bench

    def torques(self):
        return self._b.torques()

    def tool_pose(self):
        return self._b.tool_pose()

    def tool_travel(self):
        return self._b.tool_travel()


def run_policy(env: InsertionEnv, pol, noise_sigma: float = 0.0, noise_rng: np.random.Generator | None = None):
    """Drive the episode with a per-tick policy object (act(view) -> action, .done). With noise_sigma > 0 each tick
    executes clean + N(0, sigma) (DART). Returns the clean actions."""
    rng = noise_rng or np.random.default_rng(0)
    view = SensorView(env.bench); clean = []
    while not env.done:
        a = np.clip(np.asarray(pol.act(view), dtype=float), -1, 1)
        clean.append(a)
        ex = a + rng.normal(0, noise_sigma, ACTION_DIM) if noise_sigma > 0 else a
        env.step(ex)
        if getattr(pol, "done", False) and not env.done:
            env.done = True; env.end_reason = "controller_returned"
    return clean


def run_controller(env: InsertionEnv, fn, noise_sigma: float = 0.0, noise_rng: np.random.Generator | None = None,
                   **kw) -> ExpertProxy:
    """Drive the current episode with a Bench controller through the proxy until it returns or the env ends it."""
    px = ExpertProxy(env, noise_sigma, noise_rng)
    try:
        fn(px, **kw)
        if not env.done:
            env.done = True; env.end_reason = env.end_reason or "controller_returned"
    except EpisodeOver:
        pass
    return px


assert OBS_DIM == WINDOW * ROW_DIM
