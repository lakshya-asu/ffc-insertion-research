"""Gymnasium-style environment for the twin. The controller sees only the observation packet built from the
sensor models in `sensor_models.py`; simulator truth is exposed solely through `truth()` and the episode log's
`truth` field, for offline grading.

Observation packet (dict of named channels, plus `vector()`), channel names after Lakshya's gate:
  tip_estimate        (6) camera estimate of the tip in connector frame C: x,y,z (m), roll,pitch,yaw (rad)
  tip_visible         (1) 1 while the cameras have a view of the mouth; 0 when dead-reckoned from the tool
  tip_sigma_scale     (1) how much the estimator's own uncertainty has grown (1 = a fresh frame)
  fixture_wrench_n_nm (6) board force sensor reading, force N and torque N m
  tactile             (4) jaw load N, pad shear x and y N, slip flag
  robot_q_rad         (6) tool joint positions as the arm reports them (with its repeatability bias and noise)
  last_action         (6)
  t                   (1) ticks since reset / max ticks
Action: 6 values in [-1, 1] -> increments of the tool target, at most 0.2 mm and 0.5 deg per tick. The tool follows
the target through a first-order lag (the impedance controller's settling) after an observation-to-action delay.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass

import mujoco
import numpy as np

from .mjcf import build_mjcf
from .sensor_models import Camera, ForceSensor, Proprio, SensorSuite, Tactile
from .spec import DEFAULT, LEVELS, Level, Spec

ACTION_POS = 0.2e-3
ACTION_ROT = math.radians(0.5)
MAX_TICKS = 150
HARD_ABORT_FORCE = 10.0
OBS_KEYS = ("tip_estimate", "tip_visible", "tip_sigma_scale", "fixture_wrench_n_nm", "tactile", "robot_q_rad", "last_action", "t")


@dataclass
class EstimatorConfig:
    """Camera and tactile quality presets. `good`: lit, focused, in-distribution skin. `poor`: the low-light pixel
    noise figure (4x) and a cross-skin tactile error."""

    camera_quality: float = 1.0
    cross_skin_tactile: bool = False
    bias_axes_m: tuple | None = None

    @classmethod
    def good(cls):
        return cls()

    @classmethod
    def poor(cls):
        return cls(camera_quality=4.0, cross_skin_tactile=True)

    @classmethod
    def lakshya(cls):
        """His estimator as measured in experiment 046: a stereo edge fit 95 um off, 88 um of it in height, with a
        small reprojection residual, i.e. a bias rather than noise. Per-axis bias std (x depth, y lateral, z height)."""
        return cls(camera_quality=1.0, bias_axes_m=(0.03e-3, 0.03e-3, 0.088e-3))

    @classmethod
    def named(cls, name: str):
        return {"good": cls.good(), "poor": cls.poor(), "lakshya": cls.lakshya(), "none": cls.good()}[name]


@dataclass
class Randomization:
    friction: tuple = (0.2, 0.6)
    stiffness_scale: tuple = (0.5, 2.0)
    tool_kp_scale: tuple = (0.7, 1.3)
    enabled: bool = True


def _rpy(R):
    return (math.atan2(R[2, 1], R[2, 2]), -math.asin(max(-1.0, min(1.0, R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))


class TwinEnv:
    def __init__(self, spec: Spec = DEFAULT, estimator: EstimatorConfig | None = None, randomization: Randomization | None = None,
                 sensors: SensorSuite | None = None, log: bool = True, log_geoms: bool = False):
        self.spec = spec
        self.log_geoms = log_geoms
        self.est_cfg = estimator or EstimatorConfig.good()
        self.rand = randomization or Randomization()
        self.sensors = sensors or SensorSuite()
        self.model = mujoco.MjModel.from_xml_string(build_mjcf(spec))
        self.data = mujoco.MjData(self.model)
        self.tip = self.model.site("tip").id
        self.slip_adr = self.model.joint("grip_slip").qposadr[0]
        assert self.model.joint("tx").qposadr[0] == 0 and self.model.joint("rz").qposadr[0] == 5, "tool joints must be the first six dofs"
        self.dt_phys = spec.physics.timestep.value
        self.dt_tick = 1 / spec.physics.control_hz.value
        self.sub = int(round(self.dt_tick / self.dt_phys))
        self.base_friction = self.model.geom_friction.copy()
        self.base_stiffness = self.model.jnt_stiffness.copy()
        self.base_kp = self.model.actuator_gainprm.copy()
        self.base_bias = self.model.actuator_biasprm.copy()
        self.log_enabled = log
        self.rng = np.random.default_rng(0)
        self.level: Level = LEVELS["L0"]

    # ---------------------------------------------------------------- reset / step
    def reset(self, seed: int | None = None, level: str = "L0", start: dict | None = None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        self.level = LEVELS[level]
        mujoco.mj_resetData(self.model, self.data)
        lv, u, s = self.level, self.rng.uniform, (start or {})
        self.start = dict(lateral=s.get("lateral", u(-lv.lateral, lv.lateral)), height=s.get("height", u(-lv.height, lv.height)),
                          yaw=s.get("yaw", u(-lv.yaw, lv.yaw)), pitch=s.get("pitch", u(-lv.pitch, lv.pitch)), roll=s.get("roll", u(-lv.roll, lv.roll)))
        d = self.data
        d.qpos[1], d.qpos[2] = self.start["lateral"], self.start["height"]
        d.qpos[3], d.qpos[4], d.qpos[5] = self.start["roll"], self.start["pitch"], self.start["yaw"]
        d.ctrl[:6] = d.qpos[:6]
        self.desired = d.ctrl[:6].copy()
        self.draws = {}
        if self.rand.enabled:
            f = u(*self.rand.friction); ks = u(*self.rand.stiffness_scale); kp = u(*self.rand.tool_kp_scale)
            self.model.geom_friction[:] = self.base_friction; self.model.geom_friction[:, 0] = f
            self.model.jnt_stiffness[:] = self.base_stiffness * ks
            self.model.actuator_gainprm[:] = self.base_kp; self.model.actuator_biasprm[:] = self.base_bias
            self.model.actuator_gainprm[:3, 0] *= kp; self.model.actuator_biasprm[:3, 1] *= kp
            self.draws = dict(friction=float(f), stiffness_scale=float(ks), tool_kp_scale=float(kp))
        mujoco.mj_forward(self.model, d)
        ss = self.sensors
        self.cam = Camera(ss.camera, self.dt_tick, self.rng, quality=self.est_cfg.camera_quality, bias_axes_m=self.est_cfg.bias_axes_m)
        self.ft = ForceSensor(ss.force, self.dt_phys, self.rng)
        self.prop = Proprio(ss.proprio, self.rng)
        self.tac = Tactile(ss.tactile, self.rng, cross_skin=self.est_cfg.cross_skin_tactile)
        self.lag_alpha = 1 - math.exp(-self.dt_phys / ss.proprio.command_lag_s.value)
        self.delay_steps = int(round(ss.proprio.obs_to_action_s.value / self.dt_phys))
        self.tick = 0
        self.last_action = np.zeros(6)
        self.prev_slip = float(d.qpos[self.slip_adr])
        self.first_contact_tick = None
        self.mouth_cross_x = None
        self.slip_travel = 0.0     # tool travel while the pad flagged slip, from the packet only
        self.peak_force = 0.0
        self.records = []
        self._last_est = None
        self._obs = self._observe(tool_step=0.0)
        self._record()
        return self._obs, dict(start=self.start, draws=self.draws)

    def step(self, action):
        a = np.clip(np.asarray(action, dtype=float), -1, 1)
        if self.spec.tool.dof == 4:
            a[3] = a[4] = 0.0      # SCARA: roll and pitch are not commandable
        d = self.data
        prev_desired = self.desired.copy()
        new_desired = self.desired + np.r_[a[:3] * ACTION_POS, a[3:6] * ACTION_ROT]
        for i in range(self.sub):
            target = prev_desired if i < self.delay_steps else new_desired    # observation-to-action delay
            d.ctrl[:6] += self.lag_alpha * (target - d.ctrl[:6])               # impedance controller settling
            mujoco.mj_step(self.model, d)
            self.ft.push(d.sensordata[:6].copy())
        self.desired = new_desired
        self.tick += 1
        self.last_action = a
        self._obs = self._observe(tool_step=float(new_desired[0] - prev_desired[0]))
        fn = float(np.linalg.norm(self._obs["fixture_wrench_n_nm"][:3]))
        self.peak_force = max(self.peak_force, float(np.linalg.norm(d.sensordata[:3])))
        if self.first_contact_tick is None and fn > 0.3:
            self.first_contact_tick = self.tick
        if self.mouth_cross_x is None and self._obs["tip_estimate"][0] > 0.0:
            self.mouth_cross_x = float(d.ctrl[0])
        if self._obs["tactile"][3] > 0.5 and self.mouth_cross_x is not None:
            self.slip_travel += max(0.0, float(new_desired[0] - prev_desired[0]))
            self._obs["tip_estimate"][0] -= max(0.0, float(new_desired[0] - prev_desired[0]))
        self._record()
        seated = self.detector_seated()
        abort = fn > HARD_ABORT_FORCE
        terminated = seated or abort
        truncated = self.tick >= MAX_TICKS and not terminated
        return self._obs, self._reward(seated, fn), terminated, truncated, dict(seated_by_detector=seated, abort=abort, peak_force=self.peak_force)

    # ---------------------------------------------------------------- sensors
    def _observe(self, tool_step: float):
        d = self.data
        pos = d.site_xpos[self.tip].copy()
        R = d.site_xmat[self.tip].reshape(3, 3)
        true_pose = np.r_[pos, _rpy(R)]
        q = d.qpos[:6].copy()
        est, visible, scale = self.cam.step(true_pose, q, height_above_mouth_m=-pos[0], defocus_m=pos[0], tool_step_m=tool_step)
        if est is None:
            est = self._last_est if self._last_est is not None else true_pose + np.r_[self.cam.bias]
        if not visible:
            est = est.copy(); est[0] -= self.slip_travel   # fused estimator: tool travel during flagged slip did not move the tip
        self._last_est = est
        wrench = self.ft.read(self.dt_tick)
        slip_now = float(d.qpos[self.slip_adr])
        slipping = abs(slip_now - self.prev_slip) > 0.05e-3   # gross slip only: 1 mm per second at the 20 Hz tick
        self.prev_slip = slip_now
        tac = self.tac.read(d.sensordata[:6].copy(), slipping, self.dt_tick)
        return dict(tip_estimate=np.asarray(est, float), tip_visible=np.array([visible]), tip_sigma_scale=np.array([min(scale, 100.0)]),
                    fixture_wrench_n_nm=wrench, tactile=tac, robot_q_rad=self.prop.read(q), last_action=self.last_action.copy(), t=np.array([self.tick / MAX_TICKS]))

    @staticmethod
    def vector(obs) -> np.ndarray:
        return np.concatenate([obs[k] for k in OBS_KEYS])

    # ---------------------------------------------------------------- sensor-only detector and reward
    def detector_seated(self) -> bool:
        from .detector import seated_from_packet

        return seated_from_packet(self._obs, self.mouth_cross_x, float(self.data.ctrl[0]), self.spec, self.slip_travel)

    def _reward(self, seated: bool, fn: float) -> float:
        est = self._obs["tip_estimate"]
        k = self.spec.connector
        dist = abs(k.seat_depth.value - est[0]) + abs(est[1]) + abs(est[2]) + 0.002 * (abs(est[3]) + abs(est[4]) + abs(est[5]))
        r = -dist * 100.0
        if fn > self.spec.success.peak_force_limit.value:
            r -= 0.5 * (fn - self.spec.success.peak_force_limit.value)
        r -= 0.01 * float(np.abs(self.last_action).sum())
        if seated:
            r += 10.0
        return float(r)

    # ---------------------------------------------------------------- truth (offline only)
    def truth(self) -> dict:
        d = self.data
        pos = d.site_xpos[self.tip]
        R = d.site_xmat[self.tip].reshape(3, 3)
        c, k, s = self.spec.cable, self.spec.connector, self.spec.success
        corners = np.array([pos + sy * (c.width.value / 2) * R[:, 1] + sz * (c.header_thickness.value / 2) * R[:, 2] for sy in (-1, 1) for sz in (-1, 1)])
        inside = bool(np.max(np.abs(corners[:, 1])) <= k.opening_width.value / 2 and np.max(np.abs(corners[:, 2])) <= k.slot_height.value / 2)
        depth_ok = bool(corners[:, 0].min() > k.seat_depth.value - s.corner_depth_tolerance.value and corners[:, 0].max() <= k.seat_depth.value + s.over_insertion_tolerance.value)
        slip = float(d.qpos[self.slip_adr])
        dropped = abs(slip) >= 0.9 * self.spec.tool.jaw_length.value     # the tape has slid out of the jaws
        return dict(tip_pose=np.r_[pos, _rpy(R)].tolist(), depth_m=float(pos[0]), seated=inside and depth_ok and not dropped, tip_inside=inside, dropped=dropped,
                    peak_force=self.peak_force, force=float(np.linalg.norm(d.sensordata[:3])), grip_slip_m=slip)

    def _record(self):
        if self.log_enabled:
            rec = dict(tick=self.tick, obs={k: v.tolist() for k, v in self._obs.items()}, truth=self.truth())
            if self.log_geoms:
                # world pose of every geom this tick: what a renderer needs to replay the run without physics
                d = self.data
                rec["geoms"] = np.concatenate([d.geom_xpos, d.geom_xmat.reshape(-1, 9)], axis=1).round(7).tolist()
            self.records.append(rec)

    def geom_catalogue(self) -> list[dict]:
        """Static description of every geom (name, type, size, colour) for the replay exporter."""
        m = self.model
        return [dict(name=m.geom(i).name, type=int(m.geom_type[i]), size=m.geom_size[i].tolist(), rgba=m.geom_rgba[i].tolist()) for i in range(m.ngeom)]

    def episode_record(self, extra: dict | None = None) -> dict:
        return dict(spec_version=self.spec.version, board=self.spec.board, placeholders=self.spec.placeholders(), sensor_register=self.sensors.register(), level=self.level.name,
                    start=self.start, draws=self.draws, estimator=asdict(self.est_cfg), ticks=self.tick, peak_force=self.peak_force,
                    control_hz=self.spec.physics.control_hz.value, geom_catalogue=self.geom_catalogue() if self.log_geoms else None,
                    truth_final=self.truth(), records=self.records, **(extra or {}))

    def write_episode(self, path, extra=None):
        with open(path, "a") as f:
            f.write(json.dumps(self.episode_record(extra)) + "\n")
