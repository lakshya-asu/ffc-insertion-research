"""Dry-run visual servo proposals. No actuator, scene-state or ROS dependency.

A measured local image Jacobian and independently qualified joint corridor are
prerequisites. This module's output is deliberately never execution permission.
"""

from dataclasses import dataclass

import numpy as np

from ffc.macro_contract import require


@dataclass(frozen=True)
class Limits:
    frame_age_s: float = 0.5
    feedback_age_s: float = 0.1
    step_duration_s: float = 0.2
    max_step_rad: float = 0.001
    max_velocity_rad_s: float = 0.01
    max_acceleration_rad_s2: float = 0.05
    settled_velocity_rad_s: float = 0.001
    tracking_tolerance_rad: float = 0.0001
    max_condition: float = 100.0


class MotionProposal:
    """One proposal in flight; stop, settle, reacquire before planning another."""

    def __init__(self, lower, upper, identity, limits=None):
        limits = Limits() if limits is None else limits
        self.lower, self.upper = (
            np.array(lower, dtype=float, copy=True),
            np.array(upper, dtype=float, copy=True),
        )
        require(self.lower.shape == self.upper.shape == (7,), "Seven joint corridor bounds required")
        require(
            np.isfinite([self.lower, self.upper]).all() and np.all(self.lower < self.upper),
            "Invalid corridor",
        )
        require(
            isinstance(identity, str) and bool(identity),
            "Reviewed calibration/model/Jacobian identity required",
        )
        require(all(np.isfinite(v) and v > 0 for v in vars(limits).values()), "Invalid motion limits")
        self.lower.flags.writeable = self.upper.flags.writeable = False
        self.identity, self.limits = identity, limits
        self.pending = None
        self.last_now = -np.inf
        self.after = -np.inf
        self.stopped = False
        self.sequence = 0

    def _clock(self, now):
        require(np.isfinite(now), "Nonfinite clock")
        if now < self.last_now:
            self.stopped = True
        self.last_now = now
        require(not self.stopped, "Stop latched; reviewed reset required")

    def stop(self):
        self.stopped = True
        self.pending = None
        return {"state": "stopped", "executable": False}

    def propose(
        self, *, now, frame_time, feedback_time, q, velocity, error, jacobian, identity, measurement_valid
    ):
        self._clock(now)
        require(self.pending is None, "Await measured completion; no queued motion")
        require(identity == self.identity, "Calibration/model/Jacobian identity changed")
        require(measurement_valid is True, "Visual estimator abstained")
        require(np.isfinite([frame_time, feedback_time]).all(), "Invalid acquisition times")
        require(0 <= now - frame_time <= self.limits.frame_age_s, "Stale or future camera frame")
        require(0 <= now - feedback_time <= self.limits.feedback_age_s, "Stale or future joint feedback")
        require(frame_time > self.after, "Require a camera frame acquired after settling")
        q, velocity, error, jacobian = (np.asarray(x, float) for x in [q, velocity, error, jacobian])
        require(
            q.shape == velocity.shape == (7,) and error.shape == (2,) and jacobian.shape == (2, 7),
            "Expected seven joints and two image features",
        )
        require(all(np.isfinite(x).all() for x in [q, velocity, error, jacobian]), "Nonfinite measurements")
        require(np.all((q >= self.lower) & (q <= self.upper)), "Measured joints outside reviewed corridor")
        require(np.max(np.abs(velocity)) <= self.limits.settled_velocity_rad_s, "Robot not settled")
        singular = np.linalg.svd(jacobian, compute_uv=False)
        require(
            singular[-1] > 1e-8 and singular[0] / singular[-1] <= self.limits.max_condition,
            "Image response is rank deficient or ill-conditioned",
        )
        # Unit-weighted features must be supplied using the same normalization as the measured Jacobian.
        delta = -0.5 * np.linalg.pinv(jacobian) @ error
        duration = self.limits.step_duration_s
        # Peak derivative factors for quintic smoothstep: 1.875 and 10/sqrt(3).
        bound = min(
            self.limits.max_step_rad,
            self.limits.max_velocity_rad_s * duration / 1.875,
            self.limits.max_acceleration_rad_s2 * duration**2 / (10 / np.sqrt(3)),
        )
        peak = float(np.max(np.abs(delta)))
        if peak < 1e-12:
            return dict(state="hold", executable=False, reason="No image correction requested")
        delta *= min(1.0, bound / peak)
        target = q + delta
        require(
            np.all((target >= self.lower) & (target <= self.upper)), "Proposed path exits reviewed corridor"
        )
        self.sequence += 1
        self.pending = dict(sequence=self.sequence, target=target.copy(), issued_at=now)
        self.after = frame_time
        return dict(
            state="dry_run_proposal",
            executable=False,
            sequence=self.sequence,
            source_frame_time=frame_time,
            valid_until=min(frame_time + self.limits.frame_age_s, feedback_time + self.limits.feedback_age_s),
            duration_s=duration,
            start_q_rad=q.tolist(),
            target_q_rad=target.tolist(),
            interpolation="quintic_stop_to_stop",
            identity=identity,
        )

    def completed(self, *, sequence, now, feedback_time, q, velocity):
        self._clock(now)
        require(
            self.pending is not None and sequence == self.pending["sequence"], "Wrong completion identity"
        )
        require(
            np.isfinite(feedback_time) and self.pending["issued_at"] < feedback_time <= now,
            "Completion needs new joint feedback",
        )
        require(now - feedback_time <= self.limits.feedback_age_s, "Stale completion feedback")
        require(
            feedback_time >= self.pending["issued_at"] + self.limits.step_duration_s,
            "Trajectory duration has not elapsed",
        )
        q, velocity = np.asarray(q, float), np.asarray(velocity, float)
        require(
            q.shape == velocity.shape == (7,) and np.isfinite([q, velocity]).all(), "Invalid completion state"
        )
        require(
            np.max(np.abs(q - self.pending["target"])) <= self.limits.tracking_tolerance_rad,
            "Measured joints have not reached target",
        )
        require(np.max(np.abs(velocity)) <= self.limits.settled_velocity_rad_s, "Robot not settled")
        self.after = feedback_time
        self.pending = None
        return dict(state="await_new_camera_frame", executable=False)
