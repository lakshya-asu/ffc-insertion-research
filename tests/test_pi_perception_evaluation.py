"""Guard against misleading visible-object scores on empty or fragmented masks."""

import importlib.util
from pathlib import Path

import numpy as np

spec = importlib.util.spec_from_file_location(
    "pi_evaluator", Path(__file__).resolve().parents[1] / "scripts/evaluate_pi_perception.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_object_centroid_uses_connected_region_not_all_foreground_pixels():
    mask = np.zeros((40, 60), dtype=bool)
    mask[2:6, 3:7] = True
    mask[20:23, 45:48] = True
    mask[35, 59] = True
    np.testing.assert_allclose(module.largest(mask), [4.5, 3.5])
    assert module.largest(np.eye(6, dtype=bool)) is None
    assert module.largest(np.zeros((20, 20), dtype=bool)) is None


def test_empty_truth_with_predicted_socket_is_false_positive_not_localization_success():
    objects = {
        name: {"target_present": False, "prediction_present": name == "csi", "centroid_error_px": None}
        for name in module.CLASSES[1:]
    }
    confusion = np.zeros((6, 6), dtype=int)
    confusion[0, 0] = 990
    confusion[0, 4] = 10
    summary = module.summarize([{"confusion": confusion, "objects": objects}])
    assert summary["iou"]["csi"] == 0
    assert summary["iou"]["cable"] is None
    assert summary["objects"]["csi"]["false_positive_frames"] == 1
    assert summary["objects"]["csi"]["visible_frames"] == 0
    assert summary["objects"]["csi"]["localized_within_12px"] == 0
