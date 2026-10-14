"""Sensor-only insertion controllers for the bench.

push_only      advance along the tool axis until the torque rise says 'blocked' or the time runs out.
align_by_contact  advance; on contact, read the torque change; back off, rotate the wrist against the
               sensed moment, advance again; retract and re-approach with a random lateral offset
               when the push force bound is exceeded. Uses only joint positions and motor torques.
"""
from __future__ import annotations

import numpy as np

from .bench import Bench

APPROACH_STEP = 0.0004        # m per 30 Hz tick along the tool axis
APPROACH_BUDGET = 0.0075      # the face is ~4 mm ahead at the start; beyond this with no contact the pins missed it
CONTACT_RISE_NM = 0.6         # torque rise (any joint) that counts as first contact, above the 0.05 Nm noise
PUSH_BOUND_NM = 5.0           # a seat costs ~3-4 Nm of drag; jammed = torque above this AND no measured progress
ROT_STEP_RAD = np.radians(0.5)


def _held_base(b, n):
    """Free-motion torque baseline: mean of n readings over n held ticks (one sample per tick since the bench
    samples the sensor once per tick; reading n times in one tick no longer averages the noise)."""
    vals = []
    for _ in range(n):
        b.hold(); vals.append(b.torques())
    return np.mean(vals, axis=0)


def push_only(b: Bench, max_ticks=300):
    b.mark_start()
    base = _held_base(b, 5)
    for t in range(max_ticks):
        b.step_tool([0, 0, APPROACH_STEP], [0, 0, 0])
        tau = b.torques()
        if np.abs(tau - base).max() > PUSH_BOUND_NM or b.tool_travel() > APPROACH_BUDGET + 0.008:
            break
    return b.seated()


def align_by_contact(b: Bench, max_ticks=600, rng=None):
    rng = rng or np.random.default_rng(1)
    b.mark_start()
    base = _held_base(b, 5)
    phase = "approach"; attempts = 0; last_rise = np.zeros(6); spiral_pos = np.zeros(2); approach_from = 0.0; contact_travel = 0.0
    travel_hist = []
    def spiral_step():
        nonlocal attempts, spiral_pos, base
        attempts += 1
        k = attempts
        rad = 0.0003 * np.sqrt(k); ang = k * 2.39996               # golden-angle spiral, 0.3 mm pitch
        step = np.array([rad * np.cos(ang), rad * np.sin(ang)]) - spiral_pos
        spiral_pos = spiral_pos + step
        b.step_tool([step[0], step[1], 0], [0, 0, 0])
        base = np.mean([b.torques() for _ in range(3)], axis=0)
    for t in range(max_ticks):
        tau = b.torques()
        rise = tau - base
        if phase == "approach":
            b.step_tool([0, 0, APPROACH_STEP], [0, 0, 0])
            if np.abs(rise).max() > CONTACT_RISE_NM:
                # first contact: do not rotate yet. Try the push first; rotate only if it stalls
                # (rotating on a clean entry threw away 5 of 6 perfect approaches).
                last_rise = rise.copy(); phase = "push"; n_push = 0; contact_travel = b.tool_travel()
            elif b.tool_travel() - approach_from > APPROACH_BUDGET:
                # went past where the face should be: the pins slid alongside. Back up and shift.
                if attempts > 40:
                    return b.seated()
                while b.tool_travel() - approach_from > 0.0:
                    b.step_tool([0, 0, -APPROACH_STEP * 2], [0, 0, 0])
                spiral_step()
        elif phase == "align":
            # the moment on the wrist joints (4, 5) says which way the pin row is skewed; rotate against it
            b.step_tool([0, 0, -APPROACH_STEP], [0, 0, 0])            # unload
            rx = -np.sign(last_rise[4]) * ROT_STEP_RAD if abs(last_rise[4]) > 0.2 else 0.0
            ry = -np.sign(last_rise[3]) * ROT_STEP_RAD if abs(last_rise[3]) > 0.2 else 0.0
            b.step_tool([0, 0, 0], [rx, ry, 0])
            n_align += 1
            if n_align >= 2:
                phase = "push"; n_push = 0
        elif phase == "push":
            b.step_tool([0, 0, APPROACH_STEP], [0, 0, 0]); n_push += 1
            travel_hist.append(b.tool_travel())
            if b.seated():
                return True
            progress = (travel_hist[-1] - travel_hist[-8]) if len(travel_hist) >= 8 else 1.0
            stalled = progress < 0.0004          # measured tool advanced < 0.4 mm over the last 8 ticks (encoders)
            if np.abs(rise).max() > PUSH_BOUND_NM and stalled:
                # blocked: the pins are not in the cavities. Unload, shift along the spiral, push again.
                if attempts > 40:
                    return b.seated()
                for _ in range(3):
                    b.step_tool([0, 0, -APPROACH_STEP], [0, 0, 0])
                spiral_step()
                phase = "push"; n_push = 0
            elif b.tool_travel() - contact_travel > 0.009:
                # pushed 9 mm past the contact point without seating: the pins are beside the housing. Back up, shift.
                if attempts > 40:
                    return b.seated()
                while b.tool_travel() - approach_from > 0.0:
                    b.step_tool([0, 0, -APPROACH_STEP * 2], [0, 0, 0])
                spiral_step()
                phase = "approach"
            elif n_push > 25 and stalled and np.abs(rise).max() > 1.5:
                # pushing hard but not advancing and not yet at the bound: wedged, try squaring the row
                phase = "align"; last_rise = rise.copy(); n_align = 0
    return b.seated()


# ---------------------------------------------------------------------------------------------------------------
# Sensor-only expert and seat detector (spec A3, A4). Inputs: b.torques(), b.tool_pose(), b.tool_travel() and the
# controller's own commands. Nothing here reads part pose, depth, contacts or the sampled pick error.
# ---------------------------------------------------------------------------------------------------------------
from .training_config import ACTION_DIM, measured_torque_noise  # noqa: E402

FACE_TRAVEL = 0.004           # the approach hands over ~4 mm short of the housing face (bench standoff, the ACT prior)
PUSH_DIR = np.array([0.45, 1.6, 0.33, 0.0, 0.0, 0.0]) / np.linalg.norm([0.45, 1.6, 0.33])   # tau_ext signature of a push, j1-j3
ONSET_Z = 3.0                 # matched-filter score (noise std units), 2 ticks running, that counts as contact onset
SURFACE_MAX = FACE_TRAVEL + 0.0030   # onset before this: pins on the housing face
ENTRY_MAX = FACE_TRAVEL + 0.0056     # onset before this: pins in the cavities (leaf drag); later: header on the face
FAR_TRAVEL = FACE_TRAVEL + 0.0085    # no onset by this travel: pins and header went past the housing
SAFE_TRAVEL = FACE_TRAVEL - 0.0025   # pins clear of the face: lateral moves allowed only above this
SEAT_BAND = (0.0040, 0.0160)         # travel since onset at a seat (onset = leaf engagement; grasp compliance adds ~1.5 mm/Nm)
POP_M = 0.0014                       # leaf pop-through: >= 1.4 mm of travel in 3 ticks (commanded push 1.2 mm) while
POP_FROM_NM = 0.4                    # pushing at >= 0.4 Nm; the pop releases ~0.55 mm per tick
SEAT_LEVEL_NM = 3.0                  # push-axis tau_ext held at seat
# detector v2 (no pop: with the rigid grasp weld the leaf pop-through shows in under 10 percent of seats).
# Fitted on 320 logged stage-2 episodes (weld 0.001): on stalled ticks with push >= 2 Nm, travel since onset was
# 9.75 mm or more (1st percentile) when seated; not-seated stalls were header-on-face blocks at the same travel.
DETECT_BAND = (0.0028, 0.0120)       # travel since onset minus DETECT_K x push (the arm and housing yield ~1 mm/Nm):
DETECT_K = 0.001                     # seated stalls gave >= 3.6 mm (1st percentile), leaf wedges 0.3 to 2.0 mm
DETECT_LEVEL_NM = 2.5                # push-axis tau_ext
DETECT_RETRACT_M = -0.0006           # commanded advance over the stall window must stay above this (no retract)
PUSH_TARGET_NM = 5.0                 # push until this, then hold (above the ~3.3 Nm leaf pop-through)
STALL_M = 0.0001                     # travel change under 0.1 mm over STALL_TICKS ticks = stall
STALL_TICKS = 8
TILT_MAX_RAD = np.radians(8.0)


class ContactMonitor:
    """Contact onset from tau_ext (torque minus a free-motion baseline) and travel; re-arms after a retract."""

    def __init__(self, noise_nm=None):
        sig = np.maximum(np.asarray(noise_nm if noise_nm is not None else measured_torque_noise()) * 1.5, 0.02)
        self.w = PUSH_DIR / sig ** 2
        self.w_norm = np.sqrt(np.sum(PUSH_DIR ** 2 / sig ** 2))
        self.base = None; self.hist = []; self.travel_hist = []
        self.onset_travel = None; self.onset_pose = None; self.tau_ext = np.zeros(6); self.above = 0
        self.soft_travel = None       # travel where the score first rose above 1.5 in the current run-up
        self.popped = False; self.push_hist = []

    def update(self, tau, travel, pose=None):
        self.travel_hist.append(travel)
        if self.base is None:
            self.base = tau.copy()
        ref = self.hist[-3] if len(self.hist) >= 3 else self.base     # baseline lagged 3 ticks, frozen at onset
        self.tau_ext = tau - (self.base if self.onset_travel is not None else ref)
        z = float(self.w @ self.tau_ext) / self.w_norm
        if self.onset_travel is None:
            self.above = self.above + 1 if z > ONSET_Z else 0
            self.soft_travel = (self.soft_travel or travel) if z > 1.5 else None
            h = self.travel_hist
            # slowed-by-contact: a weak rise (score > 1.5) while the travel rate halves (encoders), for contacts whose
            # drag is too light for the torque rule (a clean entry at 0.3 mm per tick peaked at score 2.5)
            slowed = (len(h) > 6 and z > 1.5 and h[-4] - h[-7] > 0.0006 and (h[-1] - h[-4]) < 0.5 * (h[-4] - h[-7]))
            if (self.above >= 2 or slowed) and len(self.travel_hist) > 3:
                self.onset_travel = self.soft_travel or travel; self.onset_pose = pose
                self.popped = False; self.push_hist = []
                self.base = ref.copy()
                self.tau_ext = tau - self.base
            else:
                self.hist.append(self.base * 0.7 + tau * 0.3 if self.hist else tau.copy())
                self.base = self.hist[-1]
        elif travel < self.onset_travel - 0.001 and self.push() < 0.3:
            self.onset_travel = None; self.hist = [tau.copy()]; self.base = tau.copy()    # re-armed after a retract
        if self.onset_travel is not None:
            self.push_hist.append(self.push())
            h = self.travel_hist
            if (len(self.push_hist) > 3 and h[-1] - h[-4] >= POP_M and self.push_hist[-4] >= POP_FROM_NM
                    and h[-1] > self.onset_travel + 0.0005):
                self.popped = True
        return self.onset_travel is not None

    def push(self):
        """tau_ext along the push signature (Nm)."""
        return float(PUSH_DIR @ self.tau_ext)

    def stalled(self):
        h = self.travel_hist
        return len(h) > STALL_TICKS and abs(h[-1] - h[-1 - STALL_TICKS]) < STALL_M


class SeatDetector:
    """Hand-built seat detector v2 from observables: travel since onset in DETECT_BAND, stall (under 0.1 mm over 8
    ticks) with the commanded push held or continuing (no retract in the window), push-axis tau_ext at or above
    DETECT_LEVEL_NM, all for 3 consecutive ticks. Step it once per tick from the start of the episode."""

    def __init__(self, b, noise_nm=None):
        self.b = b; self.mon = ContactMonitor(noise_nm); self.count = 0; self.fired = False

    def step(self, tau=None):
        b = self.b
        tau = b.torques() if tau is None else tau
        on = self.mon.update(tau, b.tool_travel())
        cmd = sum(b.cmd_z_hist[-STALL_TICKS:]) if len(b.cmd_z_hist) >= STALL_TICKS else -1.0
        ok = (on and DETECT_BAND[0] <= b.tool_travel() - self.mon.onset_travel - DETECT_K * self.mon.push() <= DETECT_BAND[1]
              and self.mon.stalled() and cmd >= DETECT_RETRACT_M and self.mon.push() >= DETECT_LEVEL_NM)
        self.count = self.count + 1 if ok else 0
        self.fired = self.count >= 3
        return self.fired


def seat_detector(b, noise_nm=None) -> SeatDetector:
    return SeatDetector(b, noise_nm)


class Expert:
    """Sensor-only scripted expert. act(b) reads the sensors, returns one action in ACTION_DIM units; it does not step.
    Probe: approach until onset or FAR_TRAVEL; classify by onset travel (surface / entry / near / far), each class
    bounding |lateral offset| of the pin row; a feasibility grid over the offset picks the next lateral shift (the pick
    error is a slip across the row plus a tilt about the pinch axis, so the search is along tool y). On entry: a short
    tilt-to-null on the wrist moment, push to PUSH_TARGET_NM, hold; on a wedge back off 1 mm and re-tilt."""

    CLASS_BAND = {"entry": (0.0, 1.0e-3), "surface": (0.9e-3, 1.8e-3), "near": (1.6e-3, 2.9e-3), "far": (2.6e-3, 1.0)}

    def __init__(self, rng=None, noise_nm=None, y_range=0.008):
        self.rng = rng or np.random.default_rng(0)
        self.mon = ContactMonitor(noise_nm)
        self.grid = np.linspace(-y_range, y_range, 641)       # candidate initial offsets of the row, tool y (m)
        # prior: small offsets are likelier; it also orders the miss branch as a widening, alternating search
        self.weight = np.exp(-np.abs(self.grid) / 0.003) * (1.0 + 1e-3 * np.sign(self.grid))
        self.phase = "approach"; self.target_y = 0.0; self.p0 = None; self.R0 = None
        self.tilt = 0.0; self.tilt_dir = 0.0; self.tilt_prev = None; self.wedges = 0; self.hold = 0; self.seat_ok = 0
        self.touched = False; self.waited = 0; self.cmd_z = 0.0
        self.t = 0; self.done = False; self.y_cmd = None; self.x_cmd = None; self.target_x = 0.0; self.x_idx = 0

    def _y(self, b):
        p, _ = b.tool_pose()
        return float((p - self.p0) @ self.R0[:, 1])

    X_DITHER = ()                  # along-row tries after a surface contact; off: the tip moved 2 mm per 0.9 mm tried
    SETTLE_TICKS = 45              # after a contact, wait for the housing to creep back (time constant ~40 ticks)

    def _classify(self, cls, y_now):
        # pins on the face can mean an along-row error (the left arm's soft wrist lets the housing shift ~1 mm along
        # the row under contact and creep back slowly), not an across-row one: try the row axis first
        if cls == "surface" and self.x_idx < len(self.X_DITHER):
            self.target_x = self.X_DITHER[self.x_idx]; self.x_idx += 1
            return
        self.x_idx = 0; self.target_x = 0.0
        lo, hi = self.CLASS_BAND[cls]
        off = np.abs(self.grid + y_now)          # |offset after the shift we are at| for every candidate
        self.weight *= np.where((off >= lo) & (off <= hi), 1.0, 0.02)
        self.weight /= self.weight.sum()
        # next shift: the one that puts the most candidate mass within 0.8 mm of a cavity line, nearest first
        shifts = -self.grid
        mass = np.convolve(self.weight, np.ones(65), mode="same")      # 65 x 0.025 mm = 1.6 mm window
        score = mass
        self.target_y = float(shifts[int(np.argmax(score))])

    def act(self, b) -> np.ndarray:
        if self.p0 is None:
            self.p0, self.R0 = b.tool_pose()
        self.t += 1
        tau = b.torques(); travel = b.tool_travel(); y = self._y(b)
        on = self.mon.update(tau, travel)
        a = np.zeros(ACTION_DIM)
        if self.phase == "approach":
            if on:
                ot = self.mon.onset_travel
                cls = "surface" if ot < SURFACE_MAX else ("entry" if ot < ENTRY_MAX else "near")
                self.touched = True
                if cls == "entry":
                    self.phase = "tilt"; self.tilt_prev = None; self.tilt_dir = 0.0; self.tilt_steps = 0
                else:
                    self._classify(cls, y); self.phase = "retract"
            elif travel > FAR_TRAVEL:
                self._classify("far", y); self.phase = "retract"
            else:
                # slower through the face zone, and hold the measured across-row position at the search target
                # (the soft wrist drifts ~0.5 mm across the row on a fast approach). A hold along the row at 0.5
                # gain oscillated by +-4.7 mm with the servo lag, so the row axis is left to the servo.
                a[2] = 0.9 if travel < FACE_TRAVEL - 0.0015 else 0.6
                a[1] = np.clip((self.target_y - y) / 0.0003 * 0.3, -1.0, 1.0)
        if self.phase == "retract":
            if travel > SAFE_TRAVEL:
                a[2] = -1.0
            elif self.touched and self.waited < self.SETTLE_TICKS:
                self.waited += 1           # a contact pushed the soft-held housing ~1 mm along the row; let it return
            else:
                self.phase = "shift"; self.touched = False; self.waited = 0
        if self.phase == "shift":
            # open loop on the commanded y (measured y at the start of the shift plus our own increments), then settle
            if self.y_cmd is None:
                p, _ = b.tool_pose()
                self.y_cmd = y; self.x_cmd = float((p - self.p0) @ self.R0[:, 0]); self.settle = 0
            dy = self.target_y - self.y_cmd; dx = self.target_x - self.x_cmd
            if abs(dy) > 1e-5 or abs(dx) > 1e-5:
                a[1] = np.clip(dy / 0.0003, -1.0, 1.0); self.y_cmd += a[1] * 0.0003
                a[0] = np.clip(dx / 0.0003, -1.0, 1.0); self.x_cmd += a[0] * 0.0003
            elif self.settle < 4:
                self.settle += 1
            else:
                self.phase = "approach"; self.mon.onset_travel = None; self.y_cmd = None
        if self.phase == "tilt":
            # rotate about the pinch axis to null the wrist moment; stop when it stops falling or at the bound
            m = float(np.linalg.norm(self.mon.tau_ext[3:6]))
            if self.tilt_prev is not None and m >= self.tilt_prev:
                if self.tilt_dir != 0.0 and self.tilt_steps == 1:
                    self.tilt_dir = -self.tilt_dir          # first step made it worse: try the other way once
                else:
                    self.phase = "push"
            if self.phase == "tilt":
                if self.tilt_dir == 0.0:
                    self.tilt_dir = -np.sign(self.mon.tau_ext[4]) or 1.0
                if abs(self.tilt + self.tilt_dir * np.radians(0.5)) > TILT_MAX_RAD or self.tilt_steps >= 6:
                    self.phase = "push"
                else:
                    a[3] = self.tilt_dir; self.tilt += self.tilt_dir * np.radians(0.5); self.tilt_steps += 1
                    self.tilt_prev = m
        if self.phase == "push":
            since = travel - self.mon.onset_travel
            if self.mon.push() >= PUSH_TARGET_NM:
                if since < SEAT_BAND[0]:
                    self.wedges += 1; self.phase = "backoff"; self.back = 0
                else:
                    self.phase = "hold"
            elif since > 0.013:
                self._classify("near", y); self.phase = "retract"      # went deep with no seat force: beside
            else:
                a[2] = 0.8
        if self.phase == "backoff":
            if self.wedges > 3:
                self._classify("surface", y); self.phase = "retract"
            elif self.back < 2:
                a[2] = -1.0; self.back += 1
            else:
                self.phase = "tilt"; self.tilt_prev = None; self.tilt_dir = 0.0; self.tilt_steps = 0
        if self.phase == "hold":
            self.hold += 1
            # regulate the held force (the servo's sag correction keeps creeping the tool in while it is blocked)
            p = self.mon.push()
            # hold: bring the servo setpoint back to the measured pose (the setpoint leads the blocked tool and its
            # error term would keep creeping the joints in); the integrated joint targets keep the force
            lead = self.cmd_z - travel
            if p < SEAT_LEVEL_NM + 0.2:
                a[2] = 0.3
            elif lead > 0.0002:
                a[2] = -min(1.0, lead / 0.0005)
            since = travel - self.mon.onset_travel
            self.seat_ok = self.seat_ok + 1 if (self.mon.stalled() and SEAT_BAND[0] <= since <= SEAT_BAND[1]
                                                and p >= SEAT_LEVEL_NM) else 0
            if self.seat_ok >= 6 or self.hold >= 60:
                self.done = True
        self.cmd_z += a[2] * 0.0005
        return a


class Noisy:
    """DART-style noise: executes the policy's action plus N(0, sigma) per component (clipped), keeps the clean one."""

    def __init__(self, policy, sigma, rng=None):
        self.policy = policy; self.sigma = sigma; self.rng = rng or np.random.default_rng(0); self.clean = None

    @property
    def done(self):
        return self.policy.done

    def act(self, b):
        self.clean = self.policy.act(b)
        return np.clip(self.clean + self.rng.normal(0, self.sigma, ACTION_DIM), -1.0, 1.0)


def noisy(expert_policy, sigma, rng=None) -> Noisy:
    return Noisy(expert_policy, sigma, rng)


def expert(b, rng=None, max_ticks=900, policy=None, detector=None):
    """Run the expert (or a Noisy wrapper of it) on the bench. Returns (executed actions, clean actions, ticks of
    seat-detector firing or None). Stops when the expert is done or at max_ticks."""
    pol = policy or Expert(rng)
    det = detector or SeatDetector(b)
    acts, clean, fired = [], [], None
    det.step()
    for t in range(max_ticks):
        a = pol.act(b)
        acts.append(a); clean.append(pol.clean if isinstance(pol, Noisy) else a)
        b.step_action(a)
        if det.step() and fired is None:
            fired = t
        if pol.done:
            break
    return np.array(acts), np.array(clean), fired
