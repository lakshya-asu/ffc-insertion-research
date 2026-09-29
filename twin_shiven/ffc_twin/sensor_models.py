"""Sensor models, each grounded in a datasheet or a published measurement (27 Sep 2026 research).

Source classes on every parameter:
  D  datasheet or manufacturer document
  L  published measurement or paper
  I  arithmetic from D or L (stated)
  P  placeholder awaiting the bench

Cameras: two fixed global-shutter industrial cameras 11 deg off the board normal on opposite sides, with a
macro or telecentric lens. Nominal option: Basler a2A2448-75 with an Edmund 0.25x telecentric (91 px/mm,
160 mm working distance, 27 x 22 mm field). Force: ATI Nano17 SI-12-0.12 under the board fixture.
Proprioception: Franka FR3 (Panda figures where FR3 publishes none). Tactile: AnySkin pad on one jaw plus a
strain-gauge jaw load cell.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field, fields

import numpy as np


@dataclass(frozen=True)
class P:
    value: float
    cls: str    # D, L, I, P
    source: str


@dataclass(frozen=True)
class CameraModel:
    px_per_mm: P = P(91.2, "D", "Basler a2A2448-75 (2.74 um pixels) with Edmund 0.25x SilverTL telecentric: 0.25 / 2.74 um")
    frame_hz: P = P(60.0, "D", "a2A2448-75 runs to 75 fps; 60 fps used")
    exposure_s: P = P(0.0025, "I", "keeps motion blur under 1 px at 4 mm/s and 91 px/mm (blur = v * t_exp * k)")
    edge_sigma_px: P = P(0.10, "L", "learned keypoint heads 0.085 to 0.12 px in good light (Deep ChArUco); erf edge fits 0.03 to 0.09 px (Hagara 2011)")
    edge_sigma_px_lowlight: P = P(0.40, "L", "0.4 px at 0.3 lux (Deep ChArUco)")
    depth_axis_factor: P = P(2.6, "I", "the insertion axis is nearly along both optical axes; from two views 11 deg apart, sigma_axis = sigma_lateral / (2 sin 11 deg)")
    sharp_half_width_m: P = P(1.35e-3, "I", "half of the 2.7 mm depth of field at f/8 with the Airy disk as the circle of confusion")
    bias_episode_m: P = P(0.02e-3, "I", "per-episode bias from edge ambiguity (0.3 mm edge seen at 11 deg projects to 0.057 mm) and glare")
    bias_session_rot_rad: P = P(math.radians(0.2), "L", "residual rotational calibration error 0.1 to 0.3 deg")
    transfer_s: P = P(0.013, "I", "5.0 MB frame at the USB3 380 MB/s link limit; less with a region of interest")
    processing_s: P = P(0.010, "I", "5 to 15 ms inference for a small head on a crop")
    late_frame_prob: P = P(0.015, "L", "1 to 2 percent of frames delayed by OS scheduling (Bachhuber 2018)")
    late_frame_s: P = P(0.016, "L", "about 16 ms extra when it happens")
    outlier_prob: P = P(0.026, "L", "2.6 percent of frames fail a 3 px test in good conditions (Deep ChArUco)")
    outlier_prob_blur: P = P(0.22, "L", "22 percent under motion blur")
    outlier_px: P = P(30.0, "I", "gross error uniform 10 to 50 px")
    dead_reckon_q_m_per_mm: P = P(0.03e-3, "I", "covariance growth while occluded: 0.01 to 0.05 mm per mm of tool travel, for cable slip or bending")
    occlusion_depth_m: P = P(0.5e-3, "I", "beyond this depth the mouth is hidden by the cable and jaws (Lakshya's visibility table)")
    visibility_table: tuple = ((15.0e-3, 0.48), (3.0e-3, 0.26), (0.0, 0.08), (-0.5e-3, 0.0))


@dataclass(frozen=True)
class ForceModel:
    physics_hz: P = P(1000.0, "I", "the twin integrates at 1 kHz; the Net F/T samples at 7 kHz and streams down to this")
    resolution_n: P = P(1 / 320, "D", "ATI Nano17 SI-12-0.12 force resolution")
    resolution_nm: P = P(1 / 64000, "D", "ATI Nano17 SI-12-0.12 torque resolution, 1/64 N mm")
    noise_n: P = P(0.003, "I", "sigma set to the quoted resolution; ATI publishes no RMS figure")
    noise_nm: P = P(0.016e-3, "I", "same for torque")
    gain_error: P = P(0.005, "D", "inside the 0.75 to 1.25 percent full-scale uncertainty")
    crosstalk: P = P(0.005, "D", "same")
    bias_episode_n: P = P(0.02, "D", "repeatability, about 20 percent of the uncertainty")
    drift_n_per_s: P = P(0.01 / 60, "I", "about 0.01 N per minute after warm-up; ATI gives no number")
    lowpass_hz: P = P(35.0, "D", "a Net F/T filter setting")
    fixture_f0_hz: P = P(1500.0, "I", "100 g fixture on the 7200 Hz bare-sensor resonance; board bending could lower it to a few hundred Hz")
    fixture_zeta: P = P(0.02, "I", "lightly damped bolted fixture")
    range_n: P = P(12.0, "D", "Fx, Fy range; Fz 17 N")


@dataclass(frozen=True)
class ProprioModel:
    position_noise_m: P = P(0.05e-3, "I", "noise level consistent with the FR3's sub 0.1 mm repeatability")
    position_bias_m: P = P(0.1e-3, "D", "FR3 pose repeatability, ISO 9283")
    rotation_noise_rad: P = P(math.radians(0.01), "I", "same, rotation")
    command_lag_s: P = P(0.05, "I", "Cartesian impedance settling 0.1 to 0.2 s at 3000 N/m with a few kg effective mass; first-order lag of tau 50 ms")
    obs_to_action_s: P = P(0.010, "L", "7 to 13 ms observation-to-action delay randomized in Brahmbhatt 2023; removing it hurt transfer")
    deadzone_n: P = P(0.3, "I", "0.15 N m joint friction (IndustReal) mapped to the tool; FORGE randomizes 0 to 5 N at the wrist")
    impedance_kp: P = P(3000.0, "L", "sub-millimetre commands need 2000 N/m or more (IndustReal); the libfranka default 150 N/m left 4.45 mm of error")


@dataclass(frozen=True)
class TactileModel:
    sample_hz: P = P(100.0, "L", "AnySkin slip experiments at 100 Hz; the ReSkin circuit streams to about 400 Hz")
    shear_sigma_n: P = P(0.07, "L", "ReSkin force error 0.07 N RMS in distribution")
    shear_sigma_cross_skin_n: P = P(0.5, "L", "0.4 to 0.7 N across skins after adaptation")
    drift_n_per_s: P = P(0.01 / 60, "I", "slow baseline drift; re-zeroed before each grasp")
    slip_detect_prob: P = P(0.92, "L", "92 percent slip detection accuracy on unseen objects (AnySkin)")
    load_cell_sigma_n: P = P(0.005, "D", "1 kg strain-gauge cell with HX711: combined error 0.05 percent of full scale")
    load_cell_settle_s: P = P(0.05, "D", "HX711 at 80 samples per second settles in 50 ms")
    clamp_n: P = P(3.0, "I", "required clamp, see spec.tool")
    pad_mu: P = P(0.4, "P", "pad friction until the pull test")


@dataclass(frozen=True)
class SensorSuite:
    camera: CameraModel = field(default_factory=CameraModel)
    force: ForceModel = field(default_factory=ForceModel)
    proprio: ProprioModel = field(default_factory=ProprioModel)
    tactile: TactileModel = field(default_factory=TactileModel)

    def register(self) -> list[dict]:
        rows = []
        for g in ("camera", "force", "proprio", "tactile"):
            grp = getattr(self, g)
            for f in fields(grp):
                v = getattr(grp, f.name)
                if isinstance(v, P):
                    rows.append(dict(group=g, name=f.name, value=v.value, cls=v.cls, source=v.source))
        return rows


def _interp(x, table):
    pts = sorted(table); xs = [p[0] for p in pts]; ys = [p[1] for p in pts]
    return float(np.interp(x, xs, ys))


class Camera:
    """Two-camera tip-pose estimate at the camera's own frame rate, with focus, blur, latency, outliers,
    occlusion and proprioceptive correction. `quality` scales the pixel noise and outlier rate: 1.0 is a
    well-lit, focused setup; 4.0 is the low-light figure."""

    def __init__(self, m: CameraModel, control_dt: float, rng: np.random.Generator, quality: float = 1.0):
        self.m, self.dt, self.rng, self.quality = m, control_dt, rng, quality
        self.reset()

    def reset(self):
        m = self.m
        self.bias = np.r_[self.rng.normal(0, m.bias_episode_m.value, 3), self.rng.normal(0, m.bias_session_rot_rad.value, 3)]
        self.bias[0] *= m.depth_axis_factor.value
        self.period = 1.0 / m.frame_hz.value
        self.next_frame = self.rng.uniform(0, self.period)
        self.t = 0.0
        self.pending: deque = deque()
        self.fix = None            # (estimate6, tool_q6 at exposure, travel_at_fix)
        self.prev_true = None
        self.travel = 0.0
        self.last_visible = True

    def _sigma_px(self, blur_px: float, defocus_px: float) -> float:
        m = self.m
        base = m.edge_sigma_px.value * self.quality
        # Hagara 2011: sigma grows with total blur; combine blur sources in quadrature and scale mildly
        total = math.sqrt(1.0 + blur_px ** 2 / 12 + defocus_px ** 2)
        return base * (1.0 + 0.25 * (total - 1.0))

    def step(self, true_pose6: np.ndarray, tool_q6: np.ndarray, height_above_mouth_m: float, defocus_m: float, tool_step_m: float):
        """One control tick. Returns (estimate6 or None, visible 0/1, sigma_scale)."""
        m = self.m
        self.t += self.dt
        self.travel += abs(tool_step_m)
        speed = 0.0 if self.prev_true is None else float(np.linalg.norm(true_pose6[:3] - self.prev_true[:3]) / self.dt)
        self.prev_true = true_pose6.copy()
        vis = _interp(height_above_mouth_m, m.visibility_table)
        visible = height_above_mouth_m > -m.occlusion_depth_m.value and vis > 0.05
        if self.t >= self.next_frame:
            self.next_frame += self.period
            if visible:
                k = m.px_per_mm.value
                blur_px = speed * 1e3 * k * m.exposure_s.value
                h = m.sharp_half_width_m.value
                defocus_px = 0.0 if abs(defocus_m) <= h else (abs(defocus_m) - h) / h * 4.0
                spx = self._sigma_px(blur_px, defocus_px) / math.sqrt(max(vis, 0.05) / 0.48)
                s_lat = spx / k * 1e-3
                sig = np.array([s_lat * m.depth_axis_factor.value, s_lat, s_lat])
                s_rot = s_lat * 2.0 / 0.016  # edge sigma over half the cable width
                est = true_pose6 + self.bias + np.r_[self.rng.normal(0, 1, 3) * sig, self.rng.normal(0, s_rot, 3)]
                p_out = m.outlier_prob_blur.value if blur_px > 3 else m.outlier_prob.value * self.quality
                if self.rng.random() < p_out:
                    est[:3] += self.rng.uniform(-1, 1, 3) * m.outlier_px.value / k * 1e-3
                delay = m.exposure_s.value / 2 + m.transfer_s.value + m.processing_s.value + self.rng.uniform(0, self.period)
                if self.rng.random() < m.late_frame_prob.value:
                    delay += m.late_frame_s.value
                self.pending.append((self.t + delay, est, tool_q6.copy(), self.travel))
        while self.pending and self.pending[0][0] <= self.t:
            _, est, q_exp, trav = self.pending.popleft()
            self.fix = (est, q_exp, trav)
        if self.fix is None:
            return None, 0.0, float("inf")
        est, q_exp, trav = self.fix
        out = est + (tool_q6 - q_exp)               # add the tool motion since exposure (proprioception is not delayed)
        travelled = self.travel - trav
        grow = m.dead_reckon_q_m_per_mm.value * travelled * 1e3
        if not visible:
            out = out + np.r_[self.rng.normal(0, grow, 3), np.zeros(3)]
        scale = 1.0 + grow / max(m.edge_sigma_px.value / m.px_per_mm.value * 1e-3, 1e-9)
        return out, (1.0 if visible else 0.0), scale


class ForceSensor:
    """Wrench at the board fixture: fixture resonance, 1 kHz sampling, gain and cross-talk error, bias, drift,
    noise, low-pass, per-tick averaging, quantization, range clipping."""

    def __init__(self, m: ForceModel, physics_dt: float, rng: np.random.Generator):
        self.m, self.dt, self.rng = m, physics_dt, rng
        self.reset()

    def reset(self):
        m = self.m
        self.x = np.zeros(6); self.v = np.zeros(6); self.lp = np.zeros(6); self.acc = []
        # the fixture resonance is only representable below the sampling rate: clamp to 0.2 / dt and integrate
        # semi-implicitly (stable for w0 dt < 2). Above the twin's 1 kHz nothing of it survives the 35 Hz low-pass.
        f0 = min(m.fixture_f0_hz.value, 0.2 / self.dt)
        self.w0 = 2 * math.pi * f0; self.zeta = m.fixture_zeta.value
        g = m.gain_error.value; c = m.crosstalk.value
        self.G = np.diag(1 + self.rng.uniform(-g, g, 6)) + c * self.rng.uniform(-1, 1, (6, 6)) * (1 - np.eye(6))
        self.bias = self.rng.uniform(-m.bias_episode_n.value, m.bias_episode_n.value, 6) * np.r_[1, 1, 1, 1e-3, 1e-3, 1e-3]
        self.alpha = 1 - math.exp(-2 * math.pi * m.lowpass_hz.value * self.dt)

    def push(self, wrench6: np.ndarray):
        a = self.w0 ** 2 * (wrench6 - self.x) - 2 * self.zeta * self.w0 * self.v
        self.v += a * self.dt; self.x += self.v * self.dt          # semi-implicit Euler
        self.lp += self.alpha * (self.x - self.lp)
        self.acc.append(self.lp.copy())

    def read(self, tick_s: float) -> np.ndarray:
        m = self.m
        raw = np.mean(self.acc, axis=0) if self.acc else self.lp.copy(); n = max(len(self.acc), 1); self.acc = []
        self.bias += self.rng.normal(0, m.drift_n_per_s.value * math.sqrt(tick_s), 6) * np.r_[1, 1, 1, 1e-3, 1e-3, 1e-3]
        noise = np.r_[self.rng.normal(0, m.noise_n.value / math.sqrt(n), 3), self.rng.normal(0, m.noise_nm.value / math.sqrt(n), 3)]
        y = self.G @ raw + self.bias + noise
        y[:3] = np.clip(np.round(y[:3] / m.resolution_n.value) * m.resolution_n.value, -m.range_n.value, m.range_n.value)
        y[3:] = np.round(y[3:] / m.resolution_nm.value) * m.resolution_nm.value
        return y


class Proprio:
    def __init__(self, m: ProprioModel, rng: np.random.Generator):
        self.m, self.rng = m, rng
        self.reset()

    def reset(self):
        self.bias = self.rng.uniform(-self.m.position_bias_m.value, self.m.position_bias_m.value, 3)

    def read(self, q6: np.ndarray) -> np.ndarray:
        m = self.m
        return q6 + np.r_[self.bias + self.rng.normal(0, m.position_noise_m.value, 3), self.rng.normal(0, m.rotation_noise_rad.value, 3)]


class Tactile:
    """Jaw load cell (normal) and pad shear from the tangential load carried through the grip, with a slip flag
    that fires with the published detection probability when the grip is actually sliding."""

    def __init__(self, m: TactileModel, rng: np.random.Generator, cross_skin: bool = False):
        self.m, self.rng = m, rng
        self.sigma = m.shear_sigma_cross_skin_n.value if cross_skin else m.shear_sigma_n.value
        self.reset()

    def reset(self):
        self.drift = np.zeros(2); self.load_lp = self.m.clamp_n.value

    def read(self, tip_wrench6: np.ndarray, slipping: bool, tick_s: float) -> np.ndarray:
        m = self.m
        self.drift += self.rng.normal(0, m.drift_n_per_s.value * math.sqrt(tick_s), 2)
        shear = tip_wrench6[:2] + self.drift + self.rng.normal(0, self.sigma, 2)
        a = 1 - math.exp(-tick_s / m.load_cell_settle_s.value)
        self.load_lp += a * (m.clamp_n.value - self.load_lp)
        load = self.load_lp + self.rng.normal(0, m.load_cell_sigma_n.value)
        flag = 1.0 if (slipping and self.rng.random() < m.slip_detect_prob.value) else 0.0
        return np.r_[load, shear, flag]
