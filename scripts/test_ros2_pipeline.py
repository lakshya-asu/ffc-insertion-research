"""Real DDS transport/fault test; input is RGB only, not simulator labels."""

import copy
import json
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from ffc_cell.ros_common import BASE, SENSOR_QOS
from ffc_interfaces.msg import Observation, TaskStatus
from sensor_msgs.msg import CameraInfo, Image

rclpy.init()
node = rclpy.create_node("pipeline_fault_test")
observations = []
statuses = []
diagnostics = []
node.create_subscription(Observation, BASE + "/observation", observations.append, SENSOR_QOS)
node.create_subscription(TaskStatus, "/cell/task/status", statuses.append, 10)
node.create_subscription(DiagnosticArray, "/cell/diagnostics", diagnostics.append, 10)
images = node.create_publisher(Image, BASE + "/image_raw", SENSOR_QOS)
infos = node.create_publisher(CameraInfo, BASE + "/camera_info", SENSOR_QOS)
raw = cv2.cvtColor(cv2.imread("/input/rgb.png"), cv2.COLOR_BGR2RGB)
image = CvBridge().cv2_to_imgmsg(raw, encoding="rgb8")
image.header.frame_id = "macro_optical_frame"
info = CameraInfo()
info.header.frame_id = "macro_optical_frame"
info.width = 2448
info.height = 2048
info.k = [17627.737226277368, 0.0, 1224.0, 0.0, 17627.737226277368, 1024.0, 0.0, 0.0, 1.0]
info.r = np.eye(3).ravel().tolist()
info.d = [0.0] * 5
info.distortion_model = "plumb_bob"
info.p = np.column_stack([np.array(info.k).reshape(3, 3), np.zeros(3)]).ravel().tolist()


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def publish(mutator=None):
    im, ci = copy.deepcopy(image), copy.deepcopy(info)
    stamp = node.get_clock().now().to_msg()
    im.header.stamp = copy.deepcopy(stamp)
    ci.header.stamp = stamp
    if mutator:
        mutator(im, ci)
    infos.publish(ci)
    images.publish(im)
    return im.header


spin(2)
print("MATCHES", images.get_subscription_count(), infos.get_subscription_count(), flush=True)
header = publish()
spin(0.45)
if not observations:
    print("DIAGNOSTICS", [[s.message for s in d.status] for d in diagnostics], flush=True)
    print("STATUSES", [s.state for s in statuses], flush=True)
assert observations, "No observation received over DDS"
observed = observations[-1]
assert observed.header == header == observed.image.header == observed.camera_info.header
assert (observed.image.width, observed.image.height) == (1232, 1024)
assert observed.camera_info.k[2] == 615.75
results = [{"case": "valid", "result": "accepted", "stamp_preserved": True}]


def fault(name, mutator):
    count = len(observations)
    publish(mutator)
    spin(0.3)
    assert len(observations) == count, name + " was incorrectly accepted"
    results.append({"case": name, "result": "no observation emitted"})


fault(
    "stale",
    lambda im, ci: (
        setattr(im.header.stamp, "sec", im.header.stamp.sec - 2),
        setattr(ci.header.stamp, "sec", ci.header.stamp.sec - 2),
    ),
)
fault("wrong optical frame", lambda im, ci: setattr(im.header, "frame_id", "wrong_frame"))
fault(
    "calibration changed",
    lambda im, ci: setattr(ci, "k", [9000.0, 0.0, 1224.0, 0.0, 9000.0, 1024.0, 0.0, 0.0, 1.0]),
)
fault(
    "timestamp mismatch",
    lambda im, ci: setattr(ci.header.stamp, "nanosec", (ci.header.stamp.nanosec + 1) % 1_000_000_000),
)
fault("blank image", lambda im, ci: setattr(im, "data", bytes(len(im.data))))
spin(0.6)
assert statuses and statuses[-1].state == "stale"
assert all(not status.motion_permitted for status in statuses)
results.append({"case": "camera dropout", "result": "stale status, motion false"})
# A fresh valid frame recovers observation health after faults without enabling actions.
header = publish()
spin(0.35)
assert observations[-1].header == header
assert statuses[-1].state == "observation_only"
results.append({"case": "fresh recovery", "result": "observation_only, motion false"})
report = dict(
    results=results,
    observations=len(observations),
    status_messages=len(statuses),
    diagnostic_reasons=sorted({s.message for d in diagnostics for s in d.status}),
    motion_permitted=False,
)
Path("/output/report.json").write_text(json.dumps(report, indent=2) + "\n")
print(json.dumps(report, indent=2), flush=True)
node.destroy_node()
rclpy.shutdown()
