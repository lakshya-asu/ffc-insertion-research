"""Sensor timing and calibration guards shared by ROS nodes and fault tests."""


class FrameGuard:
    def __init__(self, max_age_ns=500_000_000):
        self.max_age_ns = max_age_ns
        self.last_ns = -1
        self.calibration = None

    def check(self, stamp_ns, now_ns, calibration):
        if stamp_ns <= 0:
            raise ValueError("missing acquisition timestamp")
        if stamp_ns > now_ns + 5_000_000:
            raise ValueError("future acquisition timestamp")
        if now_ns - stamp_ns > self.max_age_ns:
            raise ValueError("stale acquisition timestamp")
        if stamp_ns <= self.last_ns:
            raise ValueError("duplicate or out-of-order acquisition timestamp")
        if self.calibration is not None and calibration != self.calibration:
            raise ValueError("calibration changed; restart with an explicitly reviewed profile")

    def accept(self, stamp_ns, calibration):
        self.last_ns = stamp_ns
        self.calibration = calibration
