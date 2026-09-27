"""Ingress fault injection. These tests do not qualify physical perception."""

from copy import deepcopy

import pytest

from ffc.observations import ObservationGate, SensorSpec


def gate():
    return ObservationGate({"robot_q_rad": SensorSpec(7, 0.1), "jaw_load_n": SensorSpec(1, 0.1)}, 0.025)


def packet():
    return {
        "samples": {
            "robot_q_rad": {"timestamp_s": 1.0, "sequence": 1, "values": [0.0] * 7},
            "jaw_load_n": {"timestamp_s": 1.0, "sequence": 1, "values": [0.2]},
        }
    }


def test_immutable_copy_and_fresh_replay():
    ingress, payload = gate(), packet()
    observation = ingress.accept(payload, 1.01)
    payload["samples"]["jaw_load_n"]["values"][0] = 999
    assert observation.samples[1].values == (0.2,)
    assert ingress.accept(packet(), 1.02).samples == observation.samples
    with pytest.raises(ValueError, match="stale"):
        ingress.accept(packet(), 1.2)


@pytest.mark.parametrize("field", ["cable_tip_xyz_m", "true_pose", "contact_ids", "success", "usd_stage"])
def test_privileged_envelope_rejected(field):
    payload = packet()
    payload[field] = [0, 0, 0]
    with pytest.raises(ValueError):
        gate().accept(payload, 1.01)


def test_unconfigured_and_missing_channels():
    for replacement in ({}, {"true_tip": {}}, {**packet()["samples"], "true_tip": {}}):
        with pytest.raises(ValueError):
            gate().accept({"samples": replacement}, 1.01)
    with pytest.raises(ValueError):
        ObservationGate({"true_tip": SensorSpec(3, 0.1)}, 0.025)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("timestamp_s", float("nan")),
        ("timestamp_s", 1.02),
        ("timestamp_s", 0.5),
        ("timestamp_s", True),
        ("sequence", -1),
        ("sequence", True),
        ("values", [float("inf")]),
        ("values", [float("nan")]),
        ("values", [True]),
        ("values", []),
        ("values", ["0.2"]),
        ("true_depth", 0.004),
    ],
)
def test_bad_sensor_samples(key, value):
    payload = packet()
    payload["samples"]["jaw_load_n"][key] = value
    with pytest.raises(ValueError):
        gate().accept(payload, 1.01)


def test_skew_and_atomic_failure():
    ingress, payload = gate(), packet()
    bad = deepcopy(payload)
    bad["samples"]["jaw_load_n"]["timestamp_s"] = 0.95
    with pytest.raises(ValueError, match="synchronization"):
        ingress.accept(bad, 1.01)
    # Rejected input must not poison replay state or clock.
    assert ingress.accept(payload, 1.0).timestamp_s == 1.0


@pytest.mark.parametrize(("key", "value"), [("sequence", 0), ("values", [0.4]), ("timestamp_s", 0.99)])
def test_reordered_or_mutated_samples(key, value):
    ingress, payload = gate(), packet()
    ingress.accept(payload, 1.0)
    payload["samples"]["jaw_load_n"][key] = value
    with pytest.raises(ValueError):
        ingress.accept(payload, 1.01)


def test_clock_reset_requires_new_episode():
    ingress = gate()
    ingress.accept(packet(), 1.01)
    with pytest.raises(ValueError, match="backwards"):
        ingress.accept(packet(), 1.0)


def test_new_frame():
    ingress, payload = gate(), packet()
    ingress.accept(payload, 1.0)
    for sample in payload["samples"].values():
        sample.update(timestamp_s=1.02, sequence=2)
    assert ingress.accept(payload, 1.03).samples[0].sequence == 2
