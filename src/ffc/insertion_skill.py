"""Bounded insertion-feed commissioning; prealignment is an external prerequisite.

Fixture load is an ideal instrumented-fixture channel, not cable pose or seating.
Thresholds are exploratory bench settings, not identified damage limits.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FeedObservation:
    stamp_s: float
    travel_m: float
    lower_force_n: float
    upper_force_n: float
    fixture_force_n: float


@dataclass(frozen=True)
class FeedCommand:
    state: str
    target_m: float
    reason: str


class InsertionFeed:
    def __init__(self, distance_m=0.004, speed_m_s=0.001, force_cap_n=0.25):
        if not (0 < distance_m <= 0.005 and 0 < speed_m_s <= 0.001 and 0 < force_cap_n <= 0.5):
            raise ValueError("Invalid bench limits")
        self.distance, self.speed, self.cap = distance_m, speed_m_s, force_cap_n
        self.state, self.reason, self.target = "advance", "", 0.0
        self.last = None
        self.started = None
        self.retract_to = None

    def update(self, now_s, obs: FeedObservation):
        if self.state in {"stopped", "retracted", "travel_complete_unverified"}:
            return self.command()
        finite = all(math.isfinite(v) for v in [now_s, *vars(obs).values()])
        if not finite:
            return self.stop("nonfinite_feedback")
        valid_travel = -0.0005 <= obs.travel_m <= self.distance + 0.0005
        if not valid_travel:
            return self.stop("encoder_range")
        if not 0 <= now_s - obs.stamp_s <= 0.01:
            return self.stop("stale_feedback", obs.travel_m)
        if self.last is not None and (obs.stamp_s <= self.last or not 0 < now_s - self.last <= 0.01):
            return self.stop("feedback_order_or_gap", obs.travel_m)
        dt = 0 if self.last is None else now_s - self.last
        self.last = now_s
        self.started = now_s if self.started is None else self.started
        loads = [obs.lower_force_n, obs.upper_force_n, obs.fixture_force_n]
        if min(loads) < 0:
            return self.stop("invalid_load", obs.travel_m)
        if min(loads[:2]) < 0.08 or max(loads[:2]) > 1.2:
            return self.stop("grasp_feedback_lost_or_overload", obs.travel_m)
        if now_s - self.started > 7:
            return self.stop("feed_timeout", obs.travel_m)
        if self.state == "advance" and (
            obs.fixture_force_n > self.cap or self.target - obs.travel_m > 0.0002
        ):
            self.state = "retract"
            self.reason = "fixture_load_cap" if obs.fixture_force_n > self.cap else "feed_tracking_error"
            self.target = obs.travel_m
            self.retract_to = max(0, obs.travel_m - 0.0005)
        if self.state == "retract":
            if obs.fixture_force_n > 2 * self.cap:
                return self.stop("persistent_overload", obs.travel_m)
            self.target = max(self.retract_to, self.target - self.speed * dt)
            if self.target <= self.retract_to and abs(obs.travel_m - self.target) < 0.00005:
                self.state = "retracted"
        else:
            self.target = min(self.distance, self.target + self.speed * dt)
            if self.target >= self.distance and abs(obs.travel_m - self.distance) < 0.00005:
                self.state, self.reason = (
                    "travel_complete_unverified",
                    "No seating evidence from travel alone",
                )
        return self.command()

    def stop(self, reason, measured=None):
        self.state, self.reason = "stopped", reason
        if measured is not None:
            self.target = measured
        return self.command()

    def command(self):
        return FeedCommand(self.state, self.target, self.reason)
