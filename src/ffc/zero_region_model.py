"""Image-only visible-region model. No simulator, geometry or task-state inputs."""

import sys
from pathlib import Path

import torch
from torch import nn
from torch.nn import functional as F

CLASSES = ("background", "connector", "cable", "mini_end", "gripper")
CROP = (1472, 856, 2368, 1304)


def load_backbone(source, weights, device):
    sys.path.insert(0, str(Path(source)))
    from dinov2.hub.backbones import dinov2_vitb14

    model = dinov2_vitb14(pretrained=False)
    model.load_state_dict(torch.load(weights, map_location="cpu", weights_only=True))
    return model.to(device).eval().requires_grad_(False)


def encode(backbone, rgb):
    mean = rgb.new_tensor([0.485, 0.456, 0.406])[None, :, None, None]
    std = rgb.new_tensor([0.229, 0.224, 0.225])[None, :, None, None]
    return torch.cat(backbone.get_intermediate_layers((rgb - mean) / std, n=[5, 11], reshape=True), dim=1)


class RegionHead(nn.Module):
    def __init__(self, use_dino=True):
        super().__init__()
        self.use_dino = use_dino
        self.project = nn.Sequential(nn.Conv2d(1536, 64, 1), nn.GELU()) if use_dino else None
        self.rgb = nn.Sequential(
            nn.Conv2d(3, 24, 5, stride=2, padding=2),
            nn.GELU(),
            nn.Conv2d(24, 32, 3, stride=2, padding=1),
            nn.GELU(),
        )
        self.fuse = nn.Sequential(
            nn.Conv2d(96 if use_dino else 32, 48, 3, padding=1),
            nn.GELU(),
            nn.Conv2d(48, 24, 3, padding=1),
            nn.GELU(),
        )
        self.detail = nn.Sequential(
            nn.Conv2d(27, 24, 3, padding=1), nn.GELU(), nn.Conv2d(24, len(CLASSES), 1)
        )

    def forward(self, rgb, features=None):
        detail = self.rgb(rgb)
        if self.use_dino:
            if features is None:
                raise ValueError("DINO features required")
            deep = F.interpolate(
                self.project(features), size=detail.shape[-2:], mode="bilinear", align_corners=False
            )
            detail = torch.cat([detail, deep], dim=1)
        x = self.fuse(detail)
        x = F.interpolate(x, size=rgb.shape[-2:], mode="bilinear", align_corners=False)
        return self.detail(torch.cat([x, rgb], dim=1))
