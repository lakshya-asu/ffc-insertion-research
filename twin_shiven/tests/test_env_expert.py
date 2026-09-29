import math

import numpy as np

from ffc_twin.detector import seated_from_packet
from ffc_twin.env import MAX_TICKS, TwinEnv, EstimatorConfig
from ffc_twin.expert import Expert


def test_packet_has_only_sensor_channels_and_no_truth():
    env = TwinEnv(log=False)
    obs, info = env.reset(seed=1)
    assert set(obs) == {"tip_estimate", "tip_visible", "tip_sigma_scale", "fixture_wrench_n_nm", "tactile", "robot_q_rad", "last_action", "t"}
    assert TwinEnv.vector(obs).shape == (31,)


def test_changing_truth_without_changing_sensors_changes_nothing_downstream():
    """The detector reads only the packet: feed it a packet and a truth that disagree; the answer follows the packet."""
    env = TwinEnv(log=False)
    obs, _ = env.reset(seed=2)
    fake = {k: v.copy() for k, v in obs.items()}
    fake["tip_estimate"][:] = [env.spec.connector.seat_depth.value, 0, 0, 0, 0, 0]
    fake["fixture_wrench_n_nm"][:] = [0.5, 0, 0, 0, 0, 0]
    assert seated_from_packet(fake, mouth_cross_x=0.0, ctrl_x=env.spec.connector.seat_depth.value) is True
    assert env.truth()["seated"] is False


def test_estimator_freezes_when_occluded():
    env = TwinEnv(log=False)
    obs, _ = env.reset(seed=3, level="L0", start=dict(lateral=0, height=0, yaw=0, pitch=0, roll=0))
    seen = []
    for _ in range(40):
        obs, *_ = env.step(np.array([1, 0, 0, 0, 0, 0.0]))
        seen.append(float(obs["tip_visible"][0]))
    assert seen[0] == 1.0 and seen[-1] == 0.0


def test_expert_seats_from_a_hard_start_with_a_good_estimator():
    env = TwinEnv(estimator=EstimatorConfig.good(), log=True)
    obs, _ = env.reset(seed=4, level="L1", start=dict(lateral=1.0e-3, height=0.5e-3, yaw=math.radians(4), pitch=0.0, roll=0.0))
    ex = Expert(env.spec)
    for _ in range(MAX_TICKS):
        obs, r, term, trunc, info = env.step(ex.act(obs))
        if term or trunc:
            break
    t = env.truth()
    assert t["seated"], t
    assert env.peak_force < env.spec.success.peak_force_limit.value
    rec = env.episode_record()
    assert rec["records"][0]["truth"]["seated"] is False and rec["truth_final"]["seated"] is True


def test_straight_push_from_large_lateral_error_does_not_seat():
    env = TwinEnv(log=False)
    obs, _ = env.reset(seed=5, level="L1", start=dict(lateral=1.0e-3, height=0.0, yaw=0.0, pitch=0.0, roll=0.0))
    for _ in range(80):
        obs, r, term, trunc, info = env.step(np.array([1, 0, 0, 0, 0, 0.0]))
        if term or trunc:
            break
    assert env.truth()["seated"] is False
