import copy
import json
from pathlib import Path

import numpy as np
import pytest

from ffc.macro_contract import (
    CAMERA_PROFILE,
    CLASSES,
    OUTPUT_SIZE,
    PREPROCESSING_REVISION,
    CameraCalibration,
    ContractError,
    labels_to_model,
    validate_checkpoint_metadata,
    verify_digest,
)


def calibration():
    profile = json.loads(Path("config/macro-mounted-camera.json").read_text())
    return dict(
        width=2448,
        height=2048,
        k=[17627.0, 0, 1224, 0, 17627.0, 1024, 0, 0, 1],
        d=[0.0] * 5,
        distortion_model="plumb_bob",
        frame_id="macro_optical_frame",
        world_optical=np.eye(4).tolist(),
        profile=profile["revision"],
        preprocessing_revision=PREPROCESSING_REVISION,
    )


def test_fixed_extrinsic_is_part_of_identity_and_value_is_immutable():
    raw = calibration()
    value = CameraCalibration.from_mapping(raw)
    raw["world_optical"][0][3] = 0.01
    moved = CameraCalibration.from_mapping(raw)
    assert moved.fingerprint != value.fingerprint
    assert value.world_optical[0][3] == 0
    assert CameraCalibration.from_mapping(value.as_mapping()) == value


@pytest.mark.parametrize(
    "fault", ["extra_truth", "reflection", "bad_bottom_row", "nan_k", "wrong_profile", "wrong_size"]
)
def test_bad_camera_artifact_rejected(fault):
    raw = calibration()
    if fault == "extra_truth":
        raw["cable_pose"] = [0, 0, 0]
    elif fault == "reflection":
        raw["world_optical"][0][0] = -1
    elif fault == "bad_bottom_row":
        raw["world_optical"][3][0] = 1
    elif fault == "nan_k":
        raw["k"][0] = float("nan")
    elif fault == "wrong_profile":
        raw["profile"] = "other-camera"
    elif fault == "wrong_size":
        raw["width"] = 3840
    with pytest.raises(ContractError):
        CameraCalibration.from_mapping(raw)


def test_label_geometry_and_distortion_guard():
    raw = calibration()
    cal = CameraCalibration.from_mapping(raw)
    labels = np.zeros((2048, 2448), np.uint8)
    labels[10:14, 20:24] = 3
    result = labels_to_model(labels, cal)
    assert result.shape == (1024, 1232)
    assert np.argwhere(result == 3).tolist() == [[5, 14], [5, 15], [6, 14], [6, 15]]
    raw["d"][0] = 0.01
    with pytest.raises(ContractError, match="rectification"):
        labels_to_model(labels, CameraCalibration.from_mapping(raw))
    labels[0, 0] = 6
    with pytest.raises(ContractError, match="class ID"):
        labels_to_model(labels, cal)


def test_checkpoint_semantics_and_weight_identity():
    metadata = dict(
        classes=CLASSES,
        preprocessing_revision=PREPROCESSING_REVISION,
        input_size=OUTPUT_SIZE,
        camera_profile=CAMERA_PROFILE,
        use_dino=True,
    )
    validate_checkpoint_metadata(metadata)
    for key, value in [("classes", CLASSES[::-1]), ("input_size", [896, 448]), ("use_dino", False)]:
        wrong = copy.deepcopy(metadata)
        wrong[key] = value
        with pytest.raises(ContractError):
            validate_checkpoint_metadata(wrong)
    verify_digest("a" * 64, "a" * 64, "model")
    with pytest.raises(ContractError):
        verify_digest("a" * 64, "b" * 64, "backbone")
