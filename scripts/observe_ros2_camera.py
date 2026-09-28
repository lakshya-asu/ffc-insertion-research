"""Collect camera pipeline timing using only ROS observations and static camera TF."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from ffc_cell.ros_common import BASE, SENSOR_QOS, stamp_ns
from ffc_interfaces.msg import Observation, TaskStatus
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from tf2_msgs.msg import TFMessage

p = argparse.ArgumentParser()
p.add_argument("--output", type=Path, required=True)
p.add_argument("--seconds", type=float, default=30)
a = p.parse_args()
rclpy.init()
node = rclpy.create_node("camera_pipeline_observer")
rows = []
raw_stamps = []
info_stamps = []
statuses = []
frames = []


def observation(msg):
    rows.append(
        dict(
            stamp_ns=stamp_ns(msg.header),
            age_ms=(node.get_clock().now().nanoseconds - stamp_ns(msg.header)) / 1e6,
            width=msg.image.width,
            height=msg.image.height,
            frame_id=msg.header.frame_id,
            calibration_id=msg.calibration_id,
            preprocessing_revision=msg.preprocessing_revision,
        )
    )


node.create_subscription(Observation, BASE + "/observation", observation, SENSOR_QOS)
node.create_subscription(
    Image, BASE + "/image_raw", lambda m: raw_stamps.append(stamp_ns(m.header)), SENSOR_QOS
)
node.create_subscription(
    CameraInfo, BASE + "/camera_info", lambda m: info_stamps.append(stamp_ns(m.header)), SENSOR_QOS
)
node.create_subscription(TaskStatus, "/cell/task/status", lambda m: statuses.append(m.state), 10)
node.create_subscription(
    TFMessage,
    "/tf_static",
    lambda m: frames.extend([dict(parent=t.header.frame_id, child=t.child_frame_id) for t in m.transforms]),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE),
)
start = time.monotonic()
while time.monotonic() - start < a.seconds:
    rclpy.spin_once(node, timeout_sec=0.1)
result = dict(
    frames=rows,
    raw_stamps=raw_stamps,
    info_stamps=info_stamps,
    statuses=sorted(set(statuses)),
    static_transforms=frames,
    scope=("Native rendered RGB through ROS 2 transport; "
           "static simulation, wall-clock acquisition stamps"),
    motion_permitted=False,
)
a.output.write_text(json.dumps(result, indent=2) + "\n")
print("OBSERVATIONS", len(rows), "TF", frames, flush=True)
node.destroy_node()
rclpy.shutdown()
if len(rows) < 3:
    raise SystemExit("Fewer than three fresh observations")
