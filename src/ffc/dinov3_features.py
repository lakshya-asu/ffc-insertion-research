"""DINOv3 ViT-B feature adapter for the fixed native RGB window.

Requires an approved, locally downloaded official Hugging Face checkpoint.
No implicit download or random-weight fallback is permitted in load_dinov3.
"""

from pathlib import Path

import torch
from transformers import DINOv3ViTModel


def load_dinov3(directory, device="cuda"):
    path = Path(directory)
    if not (path / "model.safetensors").is_file() or not (path / "config.json").is_file():
        raise FileNotFoundError("Approved DINOv3 config and checkpoint must be downloaded first")
    model = DINOv3ViTModel.from_pretrained(path, local_files_only=True)
    if (
        model.config.hidden_size != 768
        or model.config.patch_size != 16
        or model.config.num_hidden_layers != 12
    ):
        raise ValueError("Expected official ViT-B/16 architecture")
    return model.to(device).eval().requires_grad_(False)


def encode_dinov3(model, rgb):
    if rgb.ndim != 4 or rgb.shape[1] != 3 or rgb.shape[-2] % 16 or rgb.shape[-1] % 16:
        raise ValueError(
            "RGB BCHW dimensions must be divisible by 16; do not silently resize the native window"
        )
    mean = rgb.new_tensor([0.485, 0.456, 0.406])[None, :, None, None]
    std = rgb.new_tensor([0.229, 0.224, 0.225])[None, :, None, None]
    result = model(pixel_values=(rgb - mean) / std, output_hidden_states=True)
    height, width = rgb.shape[-2] // 16, rgb.shape[-1] // 16
    prefix = 1 + model.config.num_register_tokens
    maps = []
    # HF hidden_states[0] is the embedding; entries 6 and 12 are blocks 6/12.
    for i in [6, 12]:
        patch = result.hidden_states[i][:, prefix:]
        if patch.shape[1] != height * width:
            raise ValueError("Unexpected patch/register token contract")
        maps.append(patch.transpose(1, 2).reshape(rgb.shape[0], 768, height, width))
    return torch.cat(maps, dim=1)
