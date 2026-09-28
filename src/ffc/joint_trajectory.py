"""Bounded stop-to-stop joint references for the Isaac commissioning runner."""

import numpy as np

from ffc.macro_contract import require


def validate_move(start, target, lower, upper, duration, max_step, max_velocity, max_acceleration):
    arrays = [np.asarray(x, dtype=float) for x in [start, target, lower, upper]]
    require(all(a.shape == (7,) and np.isfinite(a).all() for a in arrays), "Invalid seven-joint move")
    start, target, lower, upper = arrays
    require(np.all(lower < upper), "Invalid joint bounds")
    require(
        all(np.isfinite(x) and x > 0 for x in [duration, max_step, max_velocity, max_acceleration]),
        "Invalid trajectory limits",
    )
    require(
        np.all((start >= lower) & (start <= upper) & (target >= lower) & (target <= upper)),
        "Joint bounds exceeded",
    )
    peak = float(np.max(np.abs(target - start)))
    require(peak <= max_step, "Joint step exceeded")
    require(1.875 * peak / duration <= max_velocity, "Velocity bound exceeded")
    require((10 / np.sqrt(3)) * peak / duration**2 <= max_acceleration, "Acceleration bound exceeded")


def reference(start, target, elapsed, duration):
    require(np.isfinite(elapsed) and np.isfinite(duration) and duration > 0, "Invalid trajectory time")
    t = np.clip(elapsed / duration, 0.0, 1.0)
    s = 10 * t**3 - 15 * t**4 + 6 * t**5
    ds = (30 * t**2 - 60 * t**3 + 30 * t**4) / duration
    delta = np.asarray(target) - np.asarray(start)
    return np.asarray(start) + s * delta, ds * delta


def bounded_integral(desired, measured, integral, lower, upper, limit, gain_dt):
    """Conditional integration at position-target limits (not effort anti-windup).

    Do not build correction in a direction the clipped target cannot command.
    Correction can unwind when the error reverses. All arrays use the same per-axis
    units; callers are responsible for finite feedback and valid travel limits.
    """
    error = np.asarray(desired) - np.asarray(measured)
    candidate = np.clip(integral + gain_dt * error, -np.asarray(limit), limit)
    target = np.asarray(desired) + candidate
    blocked = ((target > upper) & (error > 0)) | ((target < lower) & (error < 0))
    return np.where(blocked, integral, candidate)
