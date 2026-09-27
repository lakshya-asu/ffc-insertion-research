"""Small RGB-only U-Net baseline; no simulator or camera-pose inputs."""

import torch
from torch import nn
from torch.nn import functional as F

CLASSES = ("background", "cable", "contacts", "stiffener", "csi", "dsi")


def block(a, b):
    return nn.Sequential(
        nn.Conv2d(a, b, 3, padding=1),
        nn.GroupNorm(4, b),
        nn.SiLU(),
        nn.Conv2d(b, b, 3, padding=1),
        nn.GroupNorm(4, b),
        nn.SiLU(),
    )


class PiPerception(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc = nn.ModuleList([block(3, 12), block(12, 24), block(24, 48), block(48, 96)])
        self.dec = nn.ModuleList([block(144, 48), block(72, 24), block(36, 12)])
        self.out = nn.Conv2d(12, len(CLASSES), 1)

    def forward(self, rgb):
        if rgb.ndim != 4 or rgb.shape[1] != 3:
            raise ValueError("Expected BCHW RGB only")
        x = rgb
        features = []
        for i, layer in enumerate(self.enc):
            if i:
                x = F.max_pool2d(x, 2)
            x = layer(x)
            features.append(x)
        for layer, skip in zip(self.dec, reversed(features[:-1]), strict=True):
            x = F.interpolate(x, size=skip.shape[-2:], mode="bilinear", align_corners=False)
            x = layer(torch.cat([x, skip], dim=1))
        return self.out(x)
