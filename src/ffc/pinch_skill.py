"""Deterministic presented-sample pinch commissioning, using tool sensors only.

This is not the desk-pickup or insertion skill. Force thresholds are experiment
settings, never a cable damage limit. Output targets require a simulation adapter.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PinchObservation:
    stamp_s: float
    lower_travel_m: float
    upper_travel_m: float
    lift_m: float
    lower_force_n: float
    upper_force_n: float


@dataclass(frozen=True)
class PinchCommand:
    state: str
    closing_travel_m: float
    lift_m: float
    reason: str


class PinchSkill:
    """Close, qualify bilateral contact, lift 10 mm, hold; latch any fault.

    No object position, attachment state or contact-counterpart identity is accepted.
    Force evidence alone is deliberately called contact, not verified cable retention.
    """

    def __init__(self):
        self.state = "close"
        self.closing = 0.0
        self.lift = 0.0
        self.started = None
        self.last = None
        self.contact_since = None
        self.phase_start = None
        self.reason = ""

    def update(self, now_s: float, observation: PinchObservation) -> PinchCommand:
        values = list(vars(observation).values()) + [now_s]
        if not all(math.isfinite(v) for v in values):
            return self._fault("nonfinite_feedback")
        if self.state == "fault":
            return self.command()
        if not 0 <= now_s - observation.stamp_s <= 0.01:
            return self._fault("stale_feedback")
        if self.last is not None and (now_s <= self.last or observation.stamp_s <= self.last):
            return self._fault("nonmonotonic_feedback")
        dt = 0.0 if self.last is None else now_s - self.last
        if dt > 0.01:
            return self._fault("control_gap")
        self.last = now_s
        if self.started is None:
            self.started = now_s
        if min(observation.lower_force_n, observation.upper_force_n) < 0:
            return self._fault("negative_normal_load")
        if max(observation.lower_force_n, observation.upper_force_n) > 1.2:
            return self._fault("bench_force_cap")
        if not (
            -0.0001 <= observation.lower_travel_m <= 0.0051
            and -0.0001 <= observation.upper_travel_m <= 0.0051
            and -0.0002 <= observation.lift_m <= 0.0105
        ):
            return self._fault("encoder_range")
        contact = min(observation.lower_force_n, observation.upper_force_n) >= 0.08
        if self.state == "close":
            measured_gap = 0.010 - observation.lower_travel_m - observation.upper_travel_m
            if measured_gap < 0.00008:
                return self._fault("empty_or_too_thin")
            if contact:
                self.contact_since = now_s if self.contact_since is None else self.contact_since
                if now_s - self.contact_since >= 0.05:
                    self.state, self.phase_start = "lift", now_s
            else:
                self.contact_since = None
                self.closing = min(0.005, self.closing + dt * 0.001)
            if now_s - self.started > 7:
                return self._fault("no_bilateral_contact")
        elif self.state in {"lift", "hold"}:
            if not contact:
                return self._fault("contact_lost")
            if self.state == "lift":
                self.lift = min(0.01, self.lift + dt * 0.005)
                if self.lift >= 0.01 and abs(observation.lift_m - 0.01) < 0.0001:
                    self.state, self.phase_start = "hold", now_s
                elif now_s - self.phase_start > 4:
                    return self._fault("lift_tracking_timeout")
            elif now_s - self.phase_start >= 0.5:
                self.state = "contact_hold_complete"
                self.reason = "Bilateral load held; independent camera retention evidence still required"
        return self.command()

    def _fault(self, reason):
        self.state, self.reason = "fault", reason
        # Freeze drive references; do not blindly open a potentially held part.
        return self.command()

    def command(self):
        return PinchCommand(self.state, self.closing, self.lift, self.reason)
