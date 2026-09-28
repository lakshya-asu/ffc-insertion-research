"""No ROS runtime required; model malformed atomic messages using attribute records."""

import copy
from types import SimpleNamespace as NS

import numpy as np
import pytest
from ffc_cell.macro_contract import CAMERA_PROFILE, PREPROCESSING_REVISION, ContractError
from ffc_cell.observation_contract import validate_observation


def observation():
    header = NS(stamp=NS(sec=100, nanosec=1), frame_id="macro_optical_frame")
    k = [8800.0, 0, 615.75, 0, 8800.0, 511.75, 0, 0, 1]
    image = NS(
        header=header,
        width=1232,
        height=1024,
        encoding="rgb8",
        is_bigendian=0,
        step=1232 * 3,
        data=bytes(1232 * 1024 * 3),
    )
    edges = NS(
        header=header,
        width=1232,
        height=1024,
        encoding="mono8",
        is_bigendian=0,
        step=1232,
        data=bytes(1232 * 1024),
    )
    info = NS(
        header=header,
        width=1232,
        height=1024,
        k=k,
        d=[0.0] * 5,
        distortion_model="plumb_bob",
        r=np.eye(3).ravel().tolist(),
        p=np.column_stack([np.array(k).reshape(3, 3), np.zeros(3)]).ravel().tolist(),
        binning_x=0,
        binning_y=0,
        roi=NS(width=0, height=0, x_offset=0, y_offset=0),
    )
    return NS(
        header=header,
        image=image,
        edges=edges,
        camera_info=info,
        calibration_id="a" * 64,
        preprocessing_revision=PREPROCESSING_REVISION,
        source_profile=CAMERA_PROFILE,
        quality_names=[
            "mean_luma",
            "std_luma",
            "low_clipped_fraction",
            "high_clipped_fraction",
            "laplacian_variance",
            "edge_fraction",
        ],
        quality_values=[100.0, 30.0, 0.0, 0.0, 12.0, 0.1],
    )


def test_valid_observation_preserves_acquisition_stamp():
    assert validate_observation(observation()) == (100_000_000_001, "a" * 64)


@pytest.mark.parametrize(
    "fault",
    [
        "edge_stamp",
        "calibration_stamp",
        "truncated_rgb",
        "wrong_dimensions",
        "wrong_projection",
        "unknown_quality",
        "nan_quality",
        "profile",
        "bad_id",
    ],
)
def test_atomic_envelope_rejects_mixed_and_malformed_inputs(fault):
    msg = observation()
    if fault == "edge_stamp":
        msg.edges.header = copy.deepcopy(msg.header)
        msg.edges.header.stamp.sec += 1
    elif fault == "calibration_stamp":
        msg.camera_info.header = copy.deepcopy(msg.header)
        msg.camera_info.header.stamp.nanosec += 1
    elif fault == "truncated_rgb":
        msg.image.data = b""
    elif fault == "wrong_dimensions":
        msg.edges.width = 800
    elif fault == "wrong_projection":
        msg.camera_info.p[0] += 1
    elif fault == "unknown_quality":
        msg.quality_names[-1] = "true_tip_depth"
    elif fault == "nan_quality":
        msg.quality_values[0] = float("nan")
    elif fault == "profile":
        msg.source_profile = "old-camera"
    elif fault == "bad_id":
        msg.calibration_id = "missing"
    with pytest.raises(ContractError):
        validate_observation(msg)


def test_mask_transform_matches_opencv_tie_rule():
    import cv2
    from ffc_cell.macro_contract import CameraCalibration, labels_to_model

    raw = dict(
        width=2448,
        height=2048,
        k=[8800.0, 0, 1224, 0, 8800.0, 1024, 0, 0, 1],
        d=[0.0] * 5,
        distortion_model="plumb_bob",
        frame_id="macro_optical_frame",
        profile=CAMERA_PROFILE,
        preprocessing_revision=PREPROCESSING_REVISION,
        world_optical=np.eye(4).tolist(),
    )
    native = np.random.default_rng(1).integers(0, 6, (2048, 2448), dtype=np.uint8)
    expected = cv2.copyMakeBorder(
        cv2.resize(native, (1224, 1024), interpolation=cv2.INTER_NEAREST_EXACT),
        0,
        0,
        4,
        4,
        cv2.BORDER_CONSTANT,
        value=0,
    )
    assert np.array_equal(labels_to_model(native, CameraCalibration.from_mapping(raw)), expected)
