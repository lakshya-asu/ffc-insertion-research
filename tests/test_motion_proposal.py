import numpy as np
import pytest

from ffc.motion_proposal import MotionProposal


def planner():
    # Synthetic corridor/Jacobian fixture. Not measured FR3 calibration.
    return MotionProposal([-1] * 7, [1] * 7, "synthetic-test-only")


def inputs():
    return dict(
        now=10.0,
        frame_time=9.9,
        feedback_time=9.99,
        q=np.zeros(7),
        velocity=np.zeros(7),
        error=np.array([10.0, -5.0]),
        jacobian=np.eye(2, 7) * 100.0,
        identity="synthetic-test-only",
        measurement_valid=True,
    )


def test_bounded_stop_to_stop_proposal_and_reacquisition():
    p = planner()
    a = inputs()
    result = p.propose(**a)
    delta = np.array(result["target_q_rad"])
    assert not result["executable"]
    assert np.max(abs(delta)) <= p.limits.max_step_rad
    assert 1.875 * np.max(abs(delta)) / result["duration_s"] <= p.limits.max_velocity_rad_s
    assert (
        10 / np.sqrt(3) * np.max(abs(delta)) / result["duration_s"] ** 2
        <= p.limits.max_acceleration_rad_s2 + 1e-12
    )
    assert np.linalg.norm(a["error"] + a["jacobian"] @ delta) < np.linalg.norm(a["error"])
    with pytest.raises(ValueError, match="no queued"):
        p.propose(**a)
    p.completed(sequence=result["sequence"], now=10.3, feedback_time=10.29, q=delta, velocity=np.zeros(7))
    a.update(now=10.31, feedback_time=10.30, q=delta)
    with pytest.raises(ValueError, match="after settling"):
        p.propose(**a)
    a["frame_time"] = 10.30
    assert p.propose(**a)["sequence"] == 2


@pytest.mark.parametrize(
    "fault",
    [
        "stale_camera",
        "future_camera",
        "stale_joints",
        "nan",
        "abstain",
        "identity",
        "rank",
        "moving",
        "corridor",
    ],
)
def test_invalid_input_never_creates_proposal(fault):
    p = planner()
    a = inputs()
    if fault == "stale_camera":
        a["frame_time"] = 9.0
    elif fault == "future_camera":
        a["frame_time"] = 11.0
    elif fault == "stale_joints":
        a["feedback_time"] = 9.0
    elif fault == "nan":
        a["error"][0] = np.nan
    elif fault == "abstain":
        a["measurement_valid"] = False
    elif fault == "identity":
        a["identity"] = "changed"
    elif fault == "rank":
        a["jacobian"][:] = 0
    elif fault == "moving":
        a["velocity"][0] = 0.1
    elif fault == "corridor":
        a["q"][0] = -1
    with pytest.raises(ValueError):
        p.propose(**a)
    assert p.pending is None


def test_completion_cannot_be_commanded_state_or_old_feedback():
    p = planner()
    r = p.propose(**inputs())
    with pytest.raises(ValueError, match="new joint feedback"):
        p.completed(
            sequence=r["sequence"], now=10.2, feedback_time=9.99, q=r["target_q_rad"], velocity=np.zeros(7)
        )
    with pytest.raises(ValueError, match="not reached"):
        p.completed(sequence=r["sequence"], now=10.3, feedback_time=10.29, q=np.ones(7), velocity=np.zeros(7))
    assert p.pending is not None


def test_stop_and_clock_reset_latch():
    p = planner()
    p.stop()
    with pytest.raises(ValueError, match="latched"):
        p.propose(**inputs())
    p = planner()
    a = inputs()
    a["error"][:] = 0
    assert p.propose(**a)["state"] == "hold"
    a["now"] = 9.0
    with pytest.raises(ValueError, match="latched"):
        p.propose(**a)


def test_completion_waits_for_trajectory_duration():
    p = planner()
    r = p.propose(**inputs())
    with pytest.raises(ValueError, match="duration"):
        p.completed(
            sequence=r["sequence"], now=10.1, feedback_time=10.09, q=r["target_q_rad"], velocity=np.zeros(7)
        )
