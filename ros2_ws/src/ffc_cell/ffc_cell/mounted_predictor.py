"""Frozen GPU predictor. No ROS, USD, annotation or object-pose dependency."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch
from dinov3_features import encode_dinov3, load_dinov3
from entrance_feature_model import FeatureHead

from ffc_cell.macro_contract import (
    CLASSES,
    CameraCalibration,
    require,
    validate_checkpoint_metadata,
    verify_digest,
)


class MountedPredictor:
    def __init__(self, model_dir, backbone_dir, rig_file):
        model, backbone = Path(model_dir), Path(backbone_dir)
        self.camera = CameraCalibration.from_mapping(json.loads(Path(rig_file).read_text()))
        frozen_camera = CameraCalibration.from_mapping(
            json.loads((model / "sensor-contract.json").read_text())
        )
        require(self.camera == frozen_camera, "Reviewed rig differs from frozen model camera")
        freeze = json.loads((model / "frozen.json").read_text())
        weights = model / "member0.pt"
        self.model_sha256 = hashlib.sha256(weights.read_bytes()).hexdigest()
        verify_digest(self.model_sha256, freeze["model_sha256"], "model")
        with (backbone / "model.safetensors").open("rb") as stream:
            self.backbone_sha256 = hashlib.file_digest(stream, "sha256").hexdigest()
        verify_digest(self.backbone_sha256, freeze["training_report"]["backbone_sha256"], "backbone")
        checkpoint = torch.load(weights, map_location="cpu", weights_only=True)
        validate_checkpoint_metadata(checkpoint)
        torch.set_num_threads(4)
        self.head = FeatureHead().cuda().eval()
        self.head.load_state_dict(checkpoint["state_dict"])
        self.backbone = load_dinov3(backbone, "cuda")
        # Compile/allocate/warm CUDA before subscribing to time-sensitive observations.
        self.predict(np.zeros((1024, 1232, 3), dtype=np.uint8))

    @torch.inference_mode()
    def predict(self, rgb):
        require(rgb.dtype == np.uint8 and rgb.shape == (1024, 1232, 3), "Wrong model RGB layout")
        start = time.perf_counter()
        x = torch.from_numpy(np.ascontiguousarray(rgb)).permute(2, 0, 1)[None].cuda().float() / 255
        with torch.autocast("cuda", dtype=torch.bfloat16):
            probabilities = self.head(x, encode_dinov3(self.backbone, x)).softmax(1)
        mask = probabilities.argmax(1)[0].cpu().numpy().astype(np.uint8)
        probabilities = probabilities[0].float().cpu().numpy()
        require(np.isfinite(probabilities).all(), "Nonfinite model output")
        scores = [
            float(probabilities[c][mask == c].mean()) if (mask == c).any() else 0.0
            for c in range(len(CLASSES))
        ]
        return mask, scores, time.perf_counter() - start
