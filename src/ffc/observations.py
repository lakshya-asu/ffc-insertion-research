"""Deployable observation ingress prototype, independent of Isaac/USD.

This validates transport, not sensor authenticity or task success. Producers must
run separately from policy code and be audited for privileged-state leakage.
All timestamps are seconds in one episode-local, monotonic clock domain.
"""

import math
from collections.abc import Mapping
from dataclasses import dataclass


@dataclass(frozen=True)
class SensorSpec:
    width: int
    max_age_s: float

    def __post_init__(self):
        if type(self.width) is not int or self.width < 1:
            raise ValueError("Sensor width must be a positive integer")
        if not math.isfinite(self.max_age_s) or self.max_age_s <= 0:
            raise ValueError("Maximum age must be finite and positive")


# Semantic names identify measured quantities, never estimated simulator poses.
# Image buffers are transported separately; this first gate covers scalar/vector I/O.
CHANNELS = frozenset(
    {
        "robot_q_rad",
        "robot_dq_rad_s",
        "jaw_position_m",
        "jaw_load_n",
        "vacuum_pressure_pa",
        "tactile_raw",
        "fixture_wrench_n_nm",
    }
)


@dataclass(frozen=True)
class Sample:
    channel: str
    timestamp_s: float
    sequence: int
    values: tuple[float, ...]


@dataclass(frozen=True)
class Observation:
    timestamp_s: float
    samples: tuple[Sample, ...]


class ObservationGate:
    """Fail-closed vector ingress; commit sequence state only after whole-frame validation."""

    def __init__(self, specs: Mapping[str, SensorSpec], max_skew_s: float):
        if not specs or set(specs) - CHANNELS:
            raise ValueError("Required channels must be a nonempty subset of measured channels")
        if any(not isinstance(spec, SensorSpec) for spec in specs.values()):
            raise ValueError("Every channel needs a SensorSpec")
        if not math.isfinite(max_skew_s) or max_skew_s < 0:
            raise ValueError("Skew limit must be finite and nonnegative")
        self.specs = dict(specs)
        self.max_skew_s = max_skew_s
        self._last: dict[str, Sample] = {}
        self._last_now = -math.inf

    def accept(self, payload: dict, now_s: float) -> Observation:
        if type(now_s) not in (int, float) or not math.isfinite(now_s) or now_s < 0:
            raise ValueError("Invalid observation clock")
        if now_s < self._last_now:
            raise ValueError("Clock moved backwards; create a gate for the new episode")
        if type(payload) is not dict or set(payload) != {"samples"}:
            raise ValueError("Only the samples envelope is permitted")
        if type(payload["samples"]) is not dict or set(payload["samples"]) != set(self.specs):
            raise ValueError("Missing or unconfigured sensor channels")
        accepted = []
        for name, spec in self.specs.items():
            raw = payload["samples"][name]
            if type(raw) is not dict or set(raw) != {"timestamp_s", "sequence", "values"}:
                raise ValueError(f"{name}: unexpected sample fields")
            timestamp, sequence, values = raw["timestamp_s"], raw["sequence"], raw["values"]
            if type(timestamp) not in (int, float) or not math.isfinite(timestamp) or timestamp < 0:
                raise ValueError(f"{name}: invalid timestamp")
            if not 0 <= now_s - timestamp <= spec.max_age_s:
                raise ValueError(f"{name}: stale or future sample")
            if type(sequence) is not int or sequence < 0:
                raise ValueError(f"{name}: invalid sequence")
            if type(values) is not list or len(values) != spec.width:
                raise ValueError(f"{name}: invalid sensor width")
            if any(type(v) not in (int, float) or not math.isfinite(v) for v in values):
                raise ValueError(f"{name}: nonnumeric or nonfinite reading")
            sample = Sample(name, float(timestamp), sequence, tuple(float(v) for v in values))
            previous = self._last.get(name)
            if previous is not None:
                if sequence < previous.sequence or timestamp < previous.timestamp_s:
                    raise ValueError(f"{name}: out-of-order sample")
                if sequence == previous.sequence and sample != previous:
                    raise ValueError(f"{name}: modified replay")
                if sequence > previous.sequence and timestamp <= previous.timestamp_s:
                    raise ValueError(f"{name}: new sequence without a later timestamp")
            accepted.append(sample)
        timestamps = [sample.timestamp_s for sample in accepted]
        if max(timestamps) - min(timestamps) > self.max_skew_s:
            raise ValueError("Sensor synchronization exceeds skew limit")
        self._last = {sample.channel: sample for sample in accepted}
        self._last_now = float(now_s)
        return Observation(float(now_s), tuple(accepted))
