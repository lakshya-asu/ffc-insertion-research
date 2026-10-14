"""Insertion bench: the twin reduced to what the insertion policy sees.

Per episode:
  * both arms start at a recorded pre-insertion state (default: preinsertion2 from a saved demo),
  * the header's pose in the right fingers and the housing's pose in the left fingers are sampled
    once (rigid grasp, pickup error): a rotation of `skew_deg` about a random axis perpendicular to
    the mating axis plus a lateral offset, drawn from the measured spread,
  * the right arm is driven in its tool frame (small translations / rotations and a push) through a
    damped-least-squares Jacobian onto the position actuators, at 30 Hz,
  * the policy observes only what the cell has: right joint positions, right motor torques
    (sim actuator forces plus sensor noise), and the commanded motion. Ground truth is used only for
    grading (seat depth, pin-in-cavity check).
"""
from __future__ import annotations

from dataclasses import dataclass

import mujoco
import numpy as np
from .layout import j6_offset

from . import scene as S
from .layout import FPS, JOINT_NAMES, grip_to_slide
from .training_config import ACTION_SCALE, Randomisation, measured_torque_noise

TORQUE_NOISE_NM = measured_torque_noise("right")   # per joint, out/torque_noise.json (scripts/torque_noise.py); 0.05 if absent
TORQUE_LAG_FRAMES = 1         # nominal obs latency (ticks) when no Randomisation is applied
TORQUE_LOWPASS = 0.5          # 1-tick low-pass on the torque reading: y = a x_t + (1 - a) y_(t-1)
GRIP_SLIDE = S.HOUSING_L / 2 + 0.00005   # pad-face gap = housing length (+0.1 mm): pads clamp the end faces
SERVO_ROT_GAIN = 0.05         # orientation-error gain of the tool servo per tick
SERVO_GAIN = 0.05             # pose-error gain of the tool servo per tick (feedforward carries the commanded motion)
GAIN_SCALE = 1.0   # real fitted gains: the lateral softness of joints 4-6 is the compliance that lets the chamfers centre the pins


@dataclass
class PickError:
    """Pick error of a housing clamped on its two end faces (pads close along tool x = the row direction).
    The clamp fixes the part; the errors are those it was picked with: a tilt about the pinch axis (the part
    sits rotated in the pad plane, so the pins are no longer square to the approach) and a slip across the row
    (along tool y, the thin direction). No twist in the grip."""
    skew_deg: float = 0.0           # rotation about the pinch axis (tool x)
    skew_dir_deg: float = 0.0       # 0 = about the pinch axis (the physical case)
    lateral_mm: tuple = (0.0, 0.0)  # (along the row = 0 when clamped, across the row)


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4); T[1, 1], T[1, 2], T[2, 1], T[2, 2] = c, -s, s, c; return T


def _rz(a):
    c, s = np.cos(a), np.sin(a)
    T = np.eye(4); T[0, 0], T[0, 1], T[1, 0], T[1, 1] = c, -s, s, c; return T


def pick_transform(err: PickError) -> np.ndarray:
    """Rigid pick error of a part in the pads: tilt `skew_deg` about an axis in the pad plane at `skew_dir_deg`
    from the pinch axis (x), plus a lateral slip (mm). Applied about the part's own origin."""
    d = np.radians(err.skew_dir_deg)
    axis = np.array([np.cos(d), np.sin(d), 0.0])
    a = np.radians(err.skew_deg)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    R = np.eye(3) + np.sin(a) * K + (1 - np.cos(a)) * K @ K
    T = np.eye(4); T[:3, :3] = R
    T[0, 3] = err.lateral_mm[0] * 1e-3; T[1, 3] = err.lateral_mm[1] * 1e-3
    return T


def pick_errors(r: Randomisation) -> tuple[PickError, PickError]:
    """Header and housing pick errors of a Randomisation draw (both a tilt about the pinch axis; slip on the header)."""
    return (PickError(skew_deg=r.header_tilt_deg, lateral_mm=(0.0, r.header_slip_mm)),
            PickError(skew_deg=r.female_tilt_deg))


def perturb(M: np.ndarray, err: PickError) -> np.ndarray:
    """Apply a pick error to a mate-in-tool transform: tilt about an axis perpendicular to z, then shift."""
    tilt = _rz(np.radians(err.skew_dir_deg)) @ _rx(np.radians(err.skew_deg)) @ _rz(-np.radians(err.skew_dir_deg))
    T = M @ tilt
    T[0, 3] += err.lateral_mm[0] * 1e-3
    T[1, 3] += err.lateral_mm[1] * 1e-3
    return T


class Bench:
    def __init__(self, start_state14: np.ndarray, header_err: PickError, housing_err: PickError | None = None,
                 rng: np.random.Generator | None = None, part_offsets: dict | None = None, retract_mm: float = 10.0,
                 rand: Randomisation | None = None):
        # geometric placement: parts pinched square between the pads; the pick error is a small rigid transform
        # of the part relative to the pads (tilt about the pinch axis x, a little about y, lateral slip)
        self.T_plug_nominal = S._tz(S.PAD_Z + S.HOUSING_PROTRUSION - S.HEADER_H / 2 + (S.PLUG_L / 2 - S.HEADER_H / 2))
        pick = {"header": pick_transform(header_err).tolist(),
                "housing": (pick_transform(housing_err) if housing_err else np.eye(4)).tolist()}
        self.spec, self.m = S.compile_model(part_offsets={"pick": pick}, free_plug=True, part_mode="geometric")
        # the fitted gains reproduce free-air sag, but joints 4-6 were never loaded in that data and come out
        # far too soft for contact (kp 8 Nm/rad bends 37 mrad under 0.3 Nm). The bench uses the stiff tracking
        # gains (x10, as replay --mode track) so the arm holds its commanded pose against the insertion load.
        for i in range(self.m.nu):
            if self.m.actuator(i).name.endswith(("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")):
                self.m.actuator_gainprm[i, 0] *= GAIN_SCALE
                self.m.actuator_biasprm[i, 1] *= GAIN_SCALE
                self.m.actuator_biasprm[i, 2] *= 3
        self.d = mujoco.MjData(self.m)
        self.rng = rng or np.random.default_rng(0)
        m = self.m
        self.jid = {(arm, i): m.joint(f"{arm}_{jn}").qposadr[0] for arm in ("left", "right") for i, jn in enumerate(JOINT_NAMES)}
        self.dof = {(arm, i): m.joint(f"{arm}_{jn}").dofadr[0] for arm in ("left", "right") for i, jn in enumerate(JOINT_NAMES)}
        self.aid = {(arm, i): m.actuator(f"{arm}_{jn}").id for arm in ("left", "right") for i, jn in enumerate(JOINT_NAMES)}
        self.gid = {arm: m.actuator(f"{arm}_gripper").id for arm in ("left", "right")}
        self.fid = {(arm, s): m.joint(f"{arm}_{s}_finger").qposadr[0] for arm in ("left", "right") for s in ("left", "right")}
        self.substeps = int(round(1.0 / FPS / m.opt.timestep))
        self.tool = m.body("right_link_6").id
        self.retract_mm = retract_mm
        right = [self.aid[("right", i)] for i in range(6)]
        self._base_gain = (m.actuator_gainprm[right, 0].copy(), m.actuator_biasprm[right, 1].copy(),
                           m.actuator_biasprm[right, 2].copy())
        self._part_geoms = [i for i in range(m.ngeom) if m.geom_contype[i] == S.PART_CONTYPE]
        self._leaf_geoms = [i for i in range(m.ngeom) if m.geom(i).name.startswith("housing_leaf_")]
        self._base_leaf_tc = m.geom_solref[self._leaf_geoms, 0].copy()
        self._latency_on = False
        self.apply(rand or Randomisation())
        self.reset(start_state14)

    def apply(self, r: Randomisation):
        """Set one episode's randomisation (gains, torque sensing, part friction, leaf contact time constant,
        latencies) on the model; call before `reset`. Pick errors are compiled into the model, see `pick_errors`."""
        m = self.m
        right = [self.aid[("right", i)] for i in range(6)]
        kp, bkp, bkd = self._base_gain
        m.actuator_gainprm[right, 0] = kp * r.kp_scale
        m.actuator_biasprm[right, 1] = bkp * r.kp_scale
        m.actuator_biasprm[right, 2] = bkd * r.kd_scale
        m.geom_friction[self._part_geoms, 0] = r.part_friction
        m.geom_solref[self._leaf_geoms, 0] = self._base_leaf_tc * r.contact_stiffness_scale
        self.rand = r

    # ---- state ----
    def reset(self, state14):
        d, m = self.d, self.m
        self._latency_on = False
        self._act_queue = []
        self.cmd_z_hist = []
        mujoco.mj_resetData(m, d)
        for arm, off in (("left", 0), ("right", 7)):
            for i in range(6):
                qi = state14[off + i] + (j6_offset(arm) if i == 5 else 0.0)
                d.qpos[self.jid[(arm, i)]] = qi
                d.ctrl[self.aid[(arm, i)]] = qi
            # pads opened to the housing length: the pad-face gap is 2 x slide (measured on the model), and the
            # recorded normalised value goes through the cell's own calibration, which we do not have.
            s = GRIP_SLIDE
            d.qpos[self.fid[(arm, "left")]] = d.qpos[self.fid[(arm, "right")]] = s
            d.ctrl[self.gid[arm]] = s
        mujoco.mj_forward(m, d)
        # part contacts off from the very start: the seated arm state puts the pins inside the leaves
        part_geoms = [i for i in range(m.ngeom) if m.geom_contype[i] == S.PART_CONTYPE]
        for i in part_geoms:
            m.geom_contype[i] = m.geom_conaffinity[i] = 0
        self._place_plug()
        for _ in range(int(0.3 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        self._torque_hist = []
        self.target_q = np.array([d.ctrl[self.aid[('right', i)]] for i in range(6)])   # commanded, not measured (sag)
        self.mark_start()
        # start aligned but out of contact: back the tool off along its axis with the parts' contacts switched
        # off (the seated start loads the arm against the housing floor, which otherwise fights the retract)
        for _ in range(int(round(self.retract_mm / 0.4))):
            self.step_tool([0, 0, -0.0004], [0, 0, 0])
        for _ in range(int(0.3 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        # the soft wrist droops once the housing no longer supports it; restore an aligned approach
        # (this stands in for the approach policy delivering the header square to the face). Truth is
        # used here once, at reset, never during control.
        self._place_plug()
        self._align_truth(target_depth_mm=-self.retract_mm + 6.0)
        for i in part_geoms:
            m.geom_contype[i] = m.geom_conaffinity[i] = S.PART_CONTYPE
        self._place_plug()
        for _ in range(int(0.1 / m.opt.timestep)):
            mujoco.mj_step(m, d)
        self._torque_hist = []; self._tau_filt = None; self.cmd_z_hist = []
        self._sense()
        self.mark_start()
        self._latency_on = True

    def _pin_row_error(self, nominal=False):
        """Lateral (housing x, y) and axis-angle error of the header relative to the housing, world frame.
        nominal=True evaluates where the header WOULD be without the pick error (what the approach aligns)."""
        Rs = self.d.body("socket").xmat.reshape(3, 3)
        Ts = np.eye(4); Ts[:3, :3] = Rs; Ts[:3, 3] = self.d.body("socket").xpos
        inv = np.linalg.inv(Ts)
        if nominal:
            Rt = self.d.xmat[self.tool].reshape(3, 3); pt = self.d.xpos[self.tool]
            Tn = np.eye(4); Tn[:3, :3] = Rt @ self.T_plug_nominal[:3, :3]; Tn[:3, 3] = pt + Rt @ self.T_plug_nominal[:3, 3]
            tip_w = Tn[:3, 3] + Tn[:3, :3] @ np.array([0, 0, S.PLUG_L / 2])
            Rp = Tn[:3, :3]
            p = (inv @ np.append(tip_w, 1))[:3]
        else:
            p = (inv @ np.append(self.d.geom("pin_tip_2").xpos, 1))[:3]      # centre pin tip in housing frame
            Rp = self.d.body("plug").xmat.reshape(3, 3)
        # full orientation error: the plug frame is the housing frame turned 180 deg about x
        # (measured at the raw seated state: 8 deg off that target, 173 deg off the about-y one).
        # rotvec = log(R_target R_plug^T), world frame.
        # The 5-pin row is symmetric under 180 deg about the plug axis, so the about-y target is as valid as
        # the about-x one; take whichever is nearer so the aligner never spins the wrist by half a turn
        # (it did once the gripper mount roll went into joint 6, which put the wrist camera on the wrong side).
        best = None
        for Rt in (Rs @ np.diag([1.0, -1.0, -1.0]), Rs @ np.diag([-1.0, 1.0, -1.0])):
            Rerr = Rt @ Rp.T
            ang = np.arccos(np.clip((np.trace(Rerr) - 1) / 2, -1, 1))
            if best is None or ang < best[0]:
                best = (ang, Rerr)
        ang, Rerr = best
        axis = np.array([Rerr[2, 1] - Rerr[1, 2], Rerr[0, 2] - Rerr[2, 0], Rerr[1, 0] - Rerr[0, 1]])
        axis = axis / (np.linalg.norm(axis) + 1e-12)
        depth = (S.HOUSING_H / 2) - p[2]
        return np.array([p[0], p[1]]), depth, axis * ang

    def _align_truth(self, target_depth_mm, iters=200):
        prev = None
        for _ in range(iters):
            lat, depth, rotvec = self._pin_row_error(nominal=True)
            err = np.linalg.norm(lat) + abs(depth - target_depth_mm * 1e-3) + 0.05 * np.linalg.norm(rotvec)
            if prev is not None and err > prev * 2.0 and err > 5e-3:
                raise RuntimeError(f"reset alignment diverging: {err:.4f}")
            prev = err
            Rs = self.d.body("socket").xmat.reshape(3, 3)
            dpos_world = Rs @ np.array([-lat[0], -lat[1], 0.0]) * 0.3 + Rs[:, 2] * (-(target_depth_mm * 1e-3 - depth)) * 0.3
            drot_world = rotvec * 0.15         # soft wrist (kp 10): small rotation steps, long settle, or it winds up
            # rotate about the pin row, not the flange: the flange must translate by (rot x (origin - tip))
            r = self.d.xpos[self.tool] - self.d.geom("pin_tip_2").xpos
            dpos_world = dpos_world + np.cross(drot_world, r)
            if np.linalg.norm(lat) < 5e-5 and abs(depth - target_depth_mm * 1e-3) < 2e-4 and np.linalg.norm(rotvec) < 2e-3:
                break
            self.step_world(dpos_world, drot_world)
            for _ in range(10):                # let the soft wrist settle before measuring again
                self.hold()

    def step_world(self, dpos_world, drot_world, max_joint_step=0.03):
        m, d = self.m, self.d
        jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, jacp, jacr, self.tool)
        cols = [self.dof[("right", i)] for i in range(6)]
        J = np.vstack([jacp[:, cols], jacr[:, cols]])
        dx = np.concatenate([np.asarray(dpos_world), np.asarray(drot_world)])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), dx)
        big = np.abs(dq).max()
        if big > max_joint_step:
            dq = dq * (max_joint_step / big)
        self.target_q = self.target_q + dq
        self._command()

    def _command(self):
        """Send the joint targets (through the action latency queue once reset is done), step one tick, sense."""
        m, d = self.m, self.d
        q = self.target_q.copy()
        if self._latency_on:
            self._act_queue.append(q)
            q = self._act_queue[max(0, len(self._act_queue) - 1 - self.rand.act_latency_ticks)]
            del self._act_queue[:-3]
        for i in range(6):
            d.ctrl[self.aid[("right", i)]] = q[i]
        for _ in range(self.substeps):
            mujoco.mj_step(m, d)
        self._sense()

    def _place_plug(self):
        T = S.PLUG_IN_TOOL["right_link_6"]
        Rt = self.d.xmat[self.tool].reshape(3, 3)
        pos = self.d.xpos[self.tool] + Rt @ T[:3, 3]
        q = np.empty(4); mujoco.mju_mat2Quat(q, (Rt @ T[:3, :3]).reshape(-1))
        adr = self.m.joint("plug_free").qposadr[0]
        self.d.qpos[adr:adr + 3] = pos; self.d.qpos[adr + 3:adr + 7] = q
        dadr = self.m.joint("plug_free").dofadr[0]; self.d.qvel[dadr:dadr + 6] = 0
        mujoco.mj_forward(self.m, self.d)

    def q_right(self):
        return np.array([self.d.qpos[self.jid[("right", i)]] for i in range(6)])

    def tool_pose(self):
        return self.d.xpos[self.tool].copy(), self.d.xmat[self.tool].reshape(3, 3).copy()

    # ---- sensors (what the cell has) ----
    def _sense(self):
        """One torque sample per 30 Hz tick: scale, bias and noise from the randomisation, then the 1-tick low-pass."""
        r = self.rand
        raw = np.array([self.d.actuator_force[self.aid[("right", i)]] for i in range(6)])
        x = raw * r.torque_scale + r.torque_bias + self.rng.normal(0, 1, 6) * r.torque_noise_std
        f = getattr(self, "_tau_filt", None)
        self._tau_filt = x if f is None else TORQUE_LOWPASS * x + (1 - TORQUE_LOWPASS) * f
        self._torque_hist.append(self._tau_filt)
        if len(self._torque_hist) > 8:
            del self._torque_hist[0]

    def torques(self):
        """The torque reading the controller has now: the filtered sample `obs_latency_ticks` ticks old.
        Reading twice in the same tick returns the same value (no new sample without time passing)."""
        if not self._torque_hist:
            self._sense()
        k = max(0, len(self._torque_hist) - 1 - self.rand.obs_latency_ticks)
        return self._torque_hist[k].copy()

    # ---- actuation: tool-frame delta -> joint targets (damped least squares) ----
    def step_tool(self, dpos_tool, drot_tool, max_joint_step=0.02):
        """dpos_tool (m) and drot_tool (rad, axis-angle) in the right tool frame; rotations pivot about the pin row.
        Closed loop on the measured tool pose (from the joint encoders): the desired pose is integrated and the
        joint targets are driven to cancel the measured error, so gravity sag does not turn into drift."""
        m, d = self.m, self.d
        R = d.xmat[self.tool].reshape(3, 3)
        drot_w = R @ np.asarray(drot_tool)
        # pivot at the NOMINAL pin-row tip (where the part would be with no pick error): the cell knows only that
        tip_nominal = self.T_plug_nominal[:3, 3] + self.T_plug_nominal[:3, :3] @ np.array([0, 0, S.PLUG_L / 2])
        r = -R @ tip_nominal
        dpos_w = R @ np.asarray(dpos_tool) + np.cross(drot_w, r)
        # integrate the desired pose
        self.cmd_z_hist.append(float(dpos_tool[2]))          # own command record (for the seat detector's stall test)
        self.des_pos = self.des_pos + dpos_w
        dR = np.empty(9); q = np.empty(4); mujoco.mju_axisAngle2Quat(q, drot_w / (np.linalg.norm(drot_w) + 1e-12), np.linalg.norm(drot_w)); mujoco.mju_quat2Mat(dR, q)
        self.des_R = dR.reshape(3, 3) @ self.des_R
        self._servo(max_joint_step, ff=np.concatenate([dpos_w, drot_w]))

    def step_action(self, a):
        """One 30 Hz tick of a policy action: (dx, dy, dz, droll, dpitch) in ACTION_BOUNDS units, clipped to [-1, 1]."""
        a = np.clip(np.asarray(a, dtype=float), -1.0, 1.0) * ACTION_SCALE
        self.step_tool(a[:3], [a[3], a[4], 0.0])

    def _servo(self, max_joint_step=0.02, gain=None, ff=None):
        """Joint-target update: the commanded twist `ff` mapped through the DLS inverse (feedforward) plus `gain` times
        the measured pose error (slow sag correction). Error-only at gain 0.45 overshot a 3 mm lateral move by 1.3 mm
        and a 5 mm push by 1.5 mm on the soft wrist; feedforward plus 0.05 overshoots by 0.4 and 0.5 mm."""
        m, d = self.m, self.d
        jacp = np.zeros((3, m.nv)); jacr = np.zeros((3, m.nv))
        mujoco.mj_jacBody(m, d, jacp, jacr, self.tool)
        cols = [self.dof[("right", i)] for i in range(6)]
        J = np.vstack([jacp[:, cols], jacr[:, cols]])
        R = d.xmat[self.tool].reshape(3, 3)
        e_pos = self.des_pos - d.xpos[self.tool]
        Rerr = self.des_R @ R.T
        ang = np.arccos(np.clip((np.trace(Rerr) - 1) / 2, -1, 1))
        axis = np.array([Rerr[2, 1] - Rerr[1, 2], Rerr[0, 2] - Rerr[2, 0], Rerr[1, 0] - Rerr[0, 1]])
        e_rot = axis / (np.linalg.norm(axis) + 1e-12) * ang
        gp = SERVO_GAIN if gain is None else gain
        gr = SERVO_ROT_GAIN if gain is None else gain
        dx = np.concatenate([e_pos * gp, e_rot * gr]) + (ff if ff is not None else 0.0)
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), dx)
        big = np.abs(dq).max()
        if big > max_joint_step:
            dq = dq * (max_joint_step / big)
        self.target_q = self.target_q + dq
        self._command()

    def tool_travel(self):
        """Measured tool travel since the controller started, along the tool axis at start (encoders only)."""
        p = self.d.xpos[self.tool]
        return float((p - self.start_pos) @ self.start_axis)

    def mark_start(self):
        self.start_pos = self.d.xpos[self.tool].copy()
        self.start_axis = self.d.xmat[self.tool].reshape(3, 3)[:, 2].copy()
        self.des_pos = self.d.xpos[self.tool].copy()
        self.des_R = self.d.xmat[self.tool].reshape(3, 3).copy()

    def hold(self):
        self._command()

    # ---- grading (truth, offline only) ----
    def seat_depth(self):
        mouth = self.d.site("socket_mouth").xpos
        axis = self.d.site("socket_mouth").xmat.reshape(3, 3)[:, 2]
        return float((mouth - self.d.site("plug_tip").xpos) @ axis)

    def pins_in(self, tol=0.0006):
        Ts = np.eye(4); Ts[:3, :3] = self.d.body("socket").xmat.reshape(3, 3); Ts[:3, 3] = self.d.body("socket").xpos
        inv = np.linalg.inv(Ts); x0 = -(S.N_PINS - 1) * S.PITCH / 2
        tips = [(inv @ np.append(self.d.geom(f"pin_tip_{i}").xpos, 1))[:3] for i in range(S.N_PINS)]
        # the row is symmetric under 180 deg about the plug axis: accept either pairing of pins to cavities
        worst = min(max(max(abs(p[0] - (x0 + k * S.PITCH)), abs(p[1])) for k, p in enumerate(order))
                    for order in (tips, tips[::-1]))
        return worst < tol, worst

    def seated(self):
        ok, _ = self.pins_in()
        return self.seat_depth() >= 0.9 * S.INSERT_DEPTH and ok

    def outcome(self):
        """Rubric label from truth: S, F-partial (in but short), F-jam (at the mouth), F-miss (beside the housing)."""
        ok, worst = self.pins_in()
        depth = self.seat_depth()
        if depth >= 0.9 * S.INSERT_DEPTH and ok:
            return "S"
        if ok and depth > 0.001:
            return "F-partial"
        if worst > S.HOUSING_T / 2 - 0.0002 or depth > S.INSERT_DEPTH:
            return "F-miss"
        return "F-jam"

    def plug_contact_force(self):
        f = 0.0
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            if "pin" in self.m.geom(c.geom1).name + self.m.geom(c.geom2).name or "plug" in self.m.geom(c.geom1).name + self.m.geom(c.geom2).name:
                ff = np.zeros(6); mujoco.mj_contactForce(self.m, self.d, i, ff); f += abs(ff[0])
        return f
