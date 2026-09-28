"""RGB-only feature segmentation. Offline geometry is not a runtime dependency."""

from torch import nn
from zero_region_model import RegionHead

CLASSES = (
    "background",
    "upper_entrance_rim",
    "lower_entrance_rim",
    "cable_leading_band",
    "slider_open",
    "slider_closed",
)
CROP = (1472, 856, 2368, 1304)


class FeatureHead(RegionHead):
    def __init__(self):
        super().__init__(True)
        self.detail[-1] = nn.Conv2d(24, len(CLASSES), 1)
