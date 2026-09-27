"""Image ingress with explicit calibration and acquisition-clock provenance."""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraSpec:
    width: int
    height: int
    calibration_id: str
    max_age_s: float = 0.25

    def __post_init__(self):
        if any(type(v) is not int or v <= 0 for v in (self.width, self.height)):
            raise ValueError("Invalid calibrated dimensions")
        if not self.calibration_id or not math.isfinite(self.max_age_s) or self.max_age_s <= 0:
            raise ValueError("Invalid calibration or age budget")


@dataclass(frozen=True)
class CameraFrame:
    camera_id: str
    sequence: int
    timestamp_s: float
    width: int
    height: int
    calibration_id: str
    rgb: bytes


class CameraGate:
    """Fresh RGB frames only. Identical pixels cannot prove a frozen camera.

    Detect frozen delivery from repeated sequence/timestamp, not image equality:
    a real static scene may legitimately produce identical images.
    """

    def __init__(self, specs: dict[str, CameraSpec]):
        if not specs or any(not isinstance(v, CameraSpec) for v in specs.values()):
            raise ValueError("Camera specifications required")
        self._specs = dict(specs)
        self._last = {}
        self._clock = -math.inf

    def accept(self, packet: dict, now_s: float) -> CameraFrame:
        fields = {"camera_id", "sequence", "timestamp_s", "width", "height", "calibration_id", "rgb"}
        if type(packet) is not dict or set(packet) != fields:
            raise ValueError("Unexpected image packet fields")
        name = packet["camera_id"]
        if type(name) is not str or name not in self._specs:
            raise ValueError("Unknown camera")
        spec = self._specs[name]
        if any(type(packet[k]) is not int for k in ("width", "height", "sequence")):
            raise ValueError("Invalid dimensions or sequence")
        if (packet["width"], packet["height"], packet["calibration_id"]) != (
            spec.width,
            spec.height,
            spec.calibration_id,
        ):
            raise ValueError("Calibration mismatch")
        timestamp = packet["timestamp_s"]
        if any(type(v) not in (int, float) or not math.isfinite(v) or v < 0 for v in (timestamp, now_s)):
            raise ValueError("Invalid acquisition clock")
        if now_s < self._clock or not 0 <= now_s - timestamp <= spec.max_age_s:
            raise ValueError("Stale, future or regressing clock")
        if type(packet["rgb"]) is not bytes or len(packet["rgb"]) != spec.width * spec.height * 3:
            raise ValueError("Expected immutable packed RGB8 buffer")
        sequence = packet["sequence"]
        if sequence < 0:
            raise ValueError("Invalid sequence")
        previous = self._last.get(name)
        if previous is not None and (sequence <= previous[0] or timestamp <= previous[1]):
            raise ValueError("Repeated or reordered camera delivery")
        self._last[name] = (sequence, timestamp)
        self._clock = now_s
        return CameraFrame(**packet)
