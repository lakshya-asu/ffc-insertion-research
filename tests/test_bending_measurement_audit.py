"""The offline rescore must catch missing settling data and altered reports."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

spec = importlib.util.spec_from_file_location(
    "bending_measurement_audit", Path(__file__).resolve().parents[1] / "scripts/audit_bending_measurements.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def fixture_run(folder, missing_sample=False, altered_report=False):
    times = np.arange(51) * 0.001 + 0.95
    q = np.array([1, 0, 0.00022779005, 0], dtype=np.float32)
    expected_sag = -Rotation.from_quat(q, scalar_first=True).apply([0.001, 0, 0])[2]
    poses = np.tile(np.r_[0, 0, 0, q], (51, 1, 1))
    if missing_sample:
        times, poses = np.delete(times, 20), np.delete(poses, 20, axis=0)
    np.savez_compressed(
        folder / "measured-poses.npz",
        time_s=times,
        position_xyz_quaternion_wxyz=poses,
        tip_paths=np.array(["/tip"]),
    )
    settings = {"dt_s": 0.001, "duration_s": 1}
    case = {
        "tip_path": "/tip",
        "origin_m": [0, 0, 0],
        "reference": {"nonlinear_hinge_chain_prediction_m": expected_sag},
        "observed_sag_m": expected_sag + (1e-7 if altered_report else 0),
        "tail_excursion_m": 0,
        "passed": True,
    }
    (folder / "report.json").write_text(json.dumps({"settings": settings, "cases": [case]}))
    (folder / "settings.json").write_text(json.dumps(settings))


def test_audit_reproduces_small_angle_and_complete_window(tmp_path):
    fixture_run(tmp_path)
    result = module.audit(tmp_path)
    assert result["all_passed"]
    assert result["tail_samples"] == 51


@pytest.mark.parametrize("defect", ["missing_sample", "altered_report"])
def test_audit_rejects_incomplete_or_inconsistent_evidence(tmp_path, defect):
    fixture_run(tmp_path, **{defect: True})
    with pytest.raises(ValueError):
        module.audit(tmp_path)
