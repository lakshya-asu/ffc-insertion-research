"""Architecture-only probe. Random weights are never reported as model performance."""

import json

import torch
from dinov3_features import encode_dinov3
from transformers import DINOv3ViTConfig, DINOv3ViTModel
from zero_region_model import RegionHead

torch.set_num_threads(4)
config = DINOv3ViTConfig(
    hidden_size=768,
    num_hidden_layers=12,
    num_attention_heads=12,
    intermediate_size=3072,
    patch_size=16,
    num_register_tokens=4,
)
model = DINOv3ViTModel(config).eval()
results = []
with torch.inference_mode():
    for height, width in [(64, 128), (448, 896)]:
        rgb = torch.zeros((1, 3, height, width))
        features = encode_dinov3(model, rgb)
        logits = RegionHead(True).eval()(rgb, features)
        assert tuple(features.shape) == (1, 1536, height // 16, width // 16)
        assert tuple(logits.shape) == (1, 5, height, width)
        results.append(
            {
                "input_shape": list(rgb.shape),
                "feature_shape": list(features.shape),
                "logit_shape": list(logits.shape),
            }
        )
print(
    json.dumps(
        {
            "cases": results,
            "pretrained_weights_loaded": False,
            "scope": "random-weight architecture compatibility only; includes exact native task window",
        }
    )
)
