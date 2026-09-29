import numpy as np
import pytest
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation

from ffc.stereo_terminal import estimate, estimate_top_surface


def projection(eye, target):
    z = target - eye
    z /= np.linalg.norm(z)
    x = np.cross(z, [0, 0, 1])
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    r = np.stack([x, y, z])
    k = np.array([[2500, 0, 400], [0, 2500, 400], [0, 0, 1]])
    return k @ np.c_[r, -r @ eye]


@pytest.mark.parametrize("yaw", [-5, 0, 5])
@pytest.mark.parametrize("estimator", [estimate, estimate_top_surface])
def test_stereo_measures_a_projected_terminal(yaw, estimator):
    center = np.array([0.491, 0.0105, 0.29])
    quad = np.array([[-0.003, -0.00575, 0], [0.003, -0.00575, 0], [0.003, 0.00575, 0], [-0.003, 0.00575, 0]])
    quad = Rotation.from_euler("z", yaw, degrees=True).apply(quad) + center
    cameras = [
        projection(center + offset, center)
        for offset in np.array([[-0.035, 0.07, 0.10], [0.035, 0.07, 0.10]])
    ]
    images = []
    for p in cameras:
        xy = np.c_[quad, np.ones(4)] @ p.T
        xy = xy[:, :2] / xy[:, 2:]
        image = Image.new("RGB", (800, 800), (100, 100, 100))
        ImageDraw.Draw(image).polygon([tuple(v) for v in xy], fill=(15, 65, 190))
        images.append(np.array(image))
    result = estimator(*images, *cameras)
    expected = (quad[0] + quad[3]) / 2
    assert np.linalg.norm(np.array(result["tip_center_m"]) - expected) < 0.00015
    assert abs(result["yaw_deg"] - yaw) < 1
    assert result["motion_permitted"] is False


def test_no_blue_feature_abstains():
    rgb = np.zeros((800, 800, 3), np.uint8)
    with pytest.raises(ValueError, match="not visible"):
        estimate(rgb, rgb, np.eye(3, 4), np.eye(3, 4))
