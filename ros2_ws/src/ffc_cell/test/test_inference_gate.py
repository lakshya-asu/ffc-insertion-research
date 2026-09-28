import copy

import numpy as np
import pytest
from ffc_cell.inference_gate import InferenceGate
from ffc_cell.macro_contract import CameraCalibration
from test_observation_contract import observation


def gate_and_message():
    gate = InferenceGate(
        CameraCalibration(
            (17600.0, 0.0, 1224.0, 0.0, 17600.0, 1024.0, 0.0, 0.0, 1.0),
            (0.0,) * 5,
            tuple(tuple(row) for row in np.eye(4)),
        )
    )
    message = observation()
    message.calibration_id = gate.identity
    return gate, message


def rig(gate, x=0):
    gate.observe_transform("world", "macro_optical_frame", [x, 0, 0], [0, 0, 0, 1])


def test_requires_rig_and_rejects_changed_rig_until_restart():
    gate, message = gate_and_message()
    with pytest.raises(ValueError, match="TF unavailable"):
        gate.check(message, 100_100_000_000)
    rig(gate)
    gate.check(message, 100_100_000_000)
    with pytest.raises(ValueError, match="TF changed"):
        rig(gate, 0.001)
    with pytest.raises(ValueError, match="latched"):
        rig(gate)
    with pytest.raises(ValueError, match="TF unavailable"):
        gate.check(message, 100_100_000_000)


def test_checks_frozen_intrinsics_even_with_self_consistent_payload():
    gate, message = gate_and_message()
    rig(gate)
    message.camera_info.k[0] += 10
    message.camera_info.p[0] += 10
    with pytest.raises(ValueError, match="Processed intrinsics"):
        gate.check(message, 100_100_000_000)


def test_post_inference_deadline_and_recovery():
    gate, message = gate_and_message()
    rig(gate)
    gate.check(message, 100_100_000_000)
    with pytest.raises(ValueError, match="stale"):
        gate.complete(message, 100_600_000_000)
    assert gate.guard.last_ns == -1
    fresh = copy.deepcopy(message)
    for item in [fresh, fresh.image, fresh.edges, fresh.camera_info]:
        item.header.stamp.sec = 101
    gate.complete(fresh, 101_100_000_000)
    with pytest.raises(ValueError, match="duplicate"):
        gate.complete(fresh, 101_200_000_000)


def test_quaternion_sign_equivalence_and_wrong_parent():
    gate, _ = gate_and_message()
    gate.observe_transform("world", "macro_optical_frame", [0, 0, 0], [0, 0, 0, -1])
    assert gate.rig_seen
    with pytest.raises(ValueError, match="directly expressed"):
        gate.observe_transform("board", "macro_optical_frame", [0, 0, 0], [0, 0, 0, 1])
