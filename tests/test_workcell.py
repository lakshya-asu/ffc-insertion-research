"""Checks for frame conventions, IK classification, and collision rejection."""

import mujoco
import numpy as np
import pytest

from ffc.workcell import (
    HORIZONTAL_ROTATION,
    Target,
    build_model,
    contact_diagnostics,
    load_config,
    rotation_error,
    solve_target,
    targets,
)


@pytest.fixture(scope="module")
def cell():
    config = load_config()
    return config, build_model(config)[0]


def test_home_respects_all_joint_limits(cell):
    _, model = cell
    assert model.nq == 8
    assert np.all(model.key_qpos[0] >= model.jnt_range[:, 0])
    assert np.all(model.key_qpos[0] <= model.jnt_range[:, 1])


def test_rotation_error_handles_half_turn():
    turn = np.diag([1.0, -1.0, -1.0])
    error = rotation_error(turn, np.eye(3))
    assert np.linalg.norm(error) == pytest.approx(np.pi)
    assert abs(error[0]) == pytest.approx(np.pi)
    np.testing.assert_allclose(rotation_error(HORIZONTAL_ROTATION, HORIZONTAL_ROTATION), 0, atol=1e-12)


def test_world_frame_rotation_error_matches_site_jacobian(cell):
    _, model = cell
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    site_id = model.site("tip_reference").id
    initial = data.site_xmat[site_id].reshape(3, 3).copy()
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    delta = np.array([0.2, -0.3, 0.1, 0.4, -0.1, 0.15, 0.1, 0.0]) * 1e-6
    data.qpos[:] += delta
    mujoco.mj_forward(model, data)
    actual = rotation_error(data.site_xmat[site_id].reshape(3, 3), initial)
    np.testing.assert_allclose(actual, jacr @ delta, atol=1e-11)


def test_approach_solution_replays_with_clearance(cell):
    config, model = cell
    target = targets(config)[-1]
    result = solve_target(model, target, config, config["seed"] + 12)
    assert result["feasible_endpoint"]
    data = mujoco.MjData(model)
    data.qpos[:] = result["qpos"]
    mujoco.mj_forward(model, data)
    np.testing.assert_allclose(
        data.site_xpos[model.site(target.site).id], target.position_m, atol=config["position_tolerance_m"]
    )
    assert not any(item["reject"] for item in contact_diagnostics(model, data, 0.0001))


def test_unreachable_target_is_not_a_success(cell):
    config, model = cell
    target = Target("unreachable", "tip_reference", np.array([3.0, 0.0, 0.5]), 0)
    result = solve_target(model, target, {**config, "ik_restarts": 2, "ik_iterations": 80}, 19)
    assert not result["ik_converged"]
    assert not result["feasible_endpoint"]


def test_low_fixture_collision_rejects_converged_ik():
    config = {**load_config(), "fixture_height_m": 0.005}
    model, _ = build_model(config)
    result = solve_target(model, targets(config)[-1], config, config["seed"] + 12)
    assert result["ik_converged"]
    assert not result["endpoint_clear"]
    assert any("desk" in contact["geoms"] and contact["reject"] for contact in result["contacts"])
    assert not result["feasible_endpoint"]


def test_results_repeat_for_fixed_seed(cell):
    config, model = cell
    target = targets(config)[0]
    a = solve_target(model, target, config, 11)
    b = solve_target(model, target, config, 11)
    assert a == b
