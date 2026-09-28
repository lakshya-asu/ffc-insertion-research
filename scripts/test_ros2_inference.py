"""DDS integration probe for mounted inference; receives no offline annotations.

Run inside the built ROS image with /input/rgb.png, /rig/camera.json,
/model, /backbone and fresh /output. Slow mode injects delay in the test harness,
not through a production parameter. All image stamps describe file replay.
"""

import argparse
import copy
import json
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from ffc_cell.cv_frontend import Frontend
from ffc_cell.inference_node import Inference
from ffc_cell.mounted_predictor import MountedPredictor
from ffc_cell.ros_common import BASE, SENSOR_QOS
from ffc_interfaces.msg import FeatureFrame, Observation
from geometry_msgs.msg import TransformStamped
from PIL import Image
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation
from tf2_msgs.msg import TFMessage

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--slow", action="store_true")
a = p.parse_args()
rclpy.init()


class DelayedPredictor(MountedPredictor):
    def predict(self, rgb):
        result = super().predict(rgb)
        time.sleep(0.6)
        return result


worker = Inference(engine_factory=DelayedPredictor if a.slow else MountedPredictor)
probe = rclpy.create_node("inference_integration_probe")
executor = SingleThreadedExecutor()
executor.add_node(worker)
executor.add_node(probe)
features, diagnostics = [], []
probe.create_subscription(FeatureFrame, BASE + "/features", features.append, SENSOR_QOS)
probe.create_subscription(DiagnosticArray, "/cell/diagnostics", diagnostics.append, 10)
pub = probe.create_publisher(Observation, BASE + "/observation", SENSOR_QOS)
tfpub = probe.create_publisher(
    TFMessage,
    "/tf_static",
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
)
bridge = CvBridge()
cal = worker.engine.camera
rgb = np.array(Image.open("/input/rgb.png").convert("RGB"))
window, k, edges, quality = Frontend().process(rgb, cal.k, cal.d)
base = Observation()
base.header.frame_id = "macro_optical_frame"
base.image = bridge.cv2_to_imgmsg(window, encoding="rgb8")
base.edges = bridge.cv2_to_imgmsg(edges, encoding="mono8")
base.camera_info.width, base.camera_info.height = 1232, 1024
base.camera_info.k = k.ravel().tolist()
base.camera_info.r = np.eye(3).ravel().tolist()
base.camera_info.p = np.column_stack([k, np.zeros(3)]).ravel().tolist()
base.camera_info.d = [0.0] * 5
base.camera_info.distortion_model = "plumb_bob"
base.calibration_id = worker.gate.identity
base.source_profile = "macro-mount-elevation45-v1"
base.preprocessing_revision = "macro-rectify-area-half-pad4-v1"
base.quality_names, base.quality_values = list(quality), list(quality.values())
results = []


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=0.02)


def send(mutator=None):
    message = copy.deepcopy(base)
    message.header.stamp = probe.get_clock().now().to_msg()
    for item in [message.image, message.edges, message.camera_info]:
        item.header = copy.deepcopy(message.header)
    if mutator:
        mutator(message)
    pub.publish(message)
    return message


def transform(offset=0):
    t = TransformStamped()
    t.header.frame_id = "world"
    t.child_frame_id = "macro_optical_frame"
    pose = np.asarray(cal.world_optical)
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = pose[:3, 3].tolist()
    t.transform.translation.x += offset
    q = Rotation.from_matrix(pose[:3, :3]).as_quat().tolist()
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
    tfpub.publish(TFMessage(transforms=[t]))
    spin(0.15)


def rejected(name, mutator=None):
    before = len(features)
    send(mutator)
    spin(0.9 if a.slow else 0.2)
    assert len(features) == before, name + " incorrectly published features"
    results.append({"case": name, "result": "no feature emitted"})


spin(0.8)
rejected("missing reviewed TF")
transform()
if a.slow:
    rejected("inference exceeded acquisition deadline")
    assert worker.rejected >= 2
    assert any("stale acquisition" in s.message for d in diagnostics for s in d.status)
else:
    message = send()
    spin(0.4)
    assert features, "No feature frame received"
    result = features[-1]
    assert result.header == message.header == result.mask.header == result.camera_info.header
    assert result.model_sha256 == worker.engine.model_sha256
    assert result.rig_calibration_sha256 == cal.fingerprint
    assert not result.motion_permitted
    mask = bridge.imgmsg_to_cv2(result.mask, desired_encoding="mono8")
    Image.fromarray(mask).save("/output/prediction.png")
    results.append({"case": "valid", "result": "acquisition stamp and model/rig identities preserved"})
    rejected(
        "stale input",
        lambda m: [
            setattr(x.header.stamp, "sec", x.header.stamp.sec - 2)
            for x in [m, m.image, m.edges, m.camera_info]
        ],
    )
    rejected("wrong profile", lambda m: setattr(m, "source_profile", "wrong"))
    rejected("wrong native calibration", lambda m: setattr(m, "calibration_id", "0" * 64))
    rejected("truncated edge image", lambda m: setattr(m.edges, "data", b""))

    def wrong_k(m):
        m.camera_info.k[0] += 1
        m.camera_info.p[0] += 1

    rejected("self-consistent but wrong processed K", wrong_k)
    count = len(features)
    send()
    spin(0.4)
    assert len(features) == count + 1
    results.append({"case": "fresh recovery", "result": "new feature emitted; motion false"})
    spin(0.6)
    assert any("expired" in s.message for d in diagnostics for s in d.status)
    results.append({"case": "dropout", "result": "expired-feature diagnostic"})
    transform(0.001)
    rejected("shifted camera TF")
    transform()
    rejected("rig fault remains latched after TF restore")
assert all(not f.motion_permitted for f in features)
report = dict(
    results=results,
    features=len(features),
    accepted=worker.accepted,
    rejected=worker.rejected,
    reasons=sorted({s.message for d in diagnostics for s in d.status}),
    motion_permitted=False,
    scope="DDS sensor replay; frozen GPU model; no offline labels available",
)
Path("/output/report.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2), flush=True)
executor.shutdown()
worker.destroy_node()
probe.destroy_node()
rclpy.shutdown()
