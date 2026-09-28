"""Read-only ROS feature viewer/recorder. No robot controls or simulator data."""

import argparse
import base64
import io
import json
import os
import time
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from ffc_cell.ros_common import BASE, SENSOR_QOS, stamp_ns
from ffc_interfaces.msg import FeatureFrame, Observation
from PIL import Image, ImageDraw, ImageFont

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--output", type=Path, required=True)
p.add_argument("--record-dir", type=Path)
p.add_argument("--seconds", type=float, default=43200)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
if a.record_dir:
    a.record_dir.mkdir(parents=True, exist_ok=False)
rclpy.init()
node = rclpy.create_node("feature_review_observer")
bridge = CvBridge()
observations = {}
rows = []
colors = np.array(
    [[0, 0, 0], [230, 141, 55], [49, 180, 162], [226, 97, 144], [87, 150, 236], [157, 112, 222]], np.uint8
)
font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
font = ImageFont.truetype(font_path, 20)
small = ImageFont.truetype(font_path, 15)


def observe(message):
    observations[stamp_ns(message.header)] = message
    while len(observations) > 4:
        del observations[next(iter(observations))]


def feature(message):
    stamp = stamp_ns(message.header)
    source = observations.get(stamp)
    if source is None:
        return
    rgb = bridge.imgmsg_to_cv2(source.image, desired_encoding="rgb8")
    mask = bridge.imgmsg_to_cv2(message.mask, desired_encoding="mono8")
    if mask.shape != rgb.shape[:2] or mask.max() > 5 or message.motion_permitted:
        raise ValueError("Invalid feature review contract")
    overlay = rgb.copy()
    fg = mask > 0
    overlay[fg] = (rgb[fg] * 0.4 + colors[mask[fg]] * 0.6).astype(np.uint8)
    canvas = Image.new("RGB", (1280, 660), "#f4f3ed")
    draw = ImageDraw.Draw(canvas)
    draw.text((24, 20), "CAMERA RGB", font=font, fill="#203d36")
    draw.text((652, 20), "LIVE DINOv3 PREDICTION", font=font, fill="#203d36")
    for x, arr in [(24, rgb), (652, overlay)]:
        panel = Image.fromarray(arr).resize((604, 502), Image.Resampling.LANCZOS)
        canvas.paste(panel, (x, 60))
    now = time.time()
    age_ms = (now - stamp / 1e9) * 1000
    processing_ms = message.processing_time.sec * 1000 + message.processing_time.nanosec / 1e6
    draw.text((24, 580), f"SIMULATION / static cell   |   acquisition {stamp} ns", font=small, fill="#203d36")
    draw.text(
        (24, 607),
        f"Inference {processing_ms:.1f} ms   |   age at receipt {age_ms:.1f} ms   |   MOTION DISABLED",
        font=small,
        fill="#203d36",
    )
    draw.text(
        (24, 634),
        "Orange: upper rim   Teal: lower rim   Pink: cable edge   Blue / purple: open / closed slider",
        font=small,
        fill="#203d36",
    )
    buffer = io.BytesIO()
    canvas.save(buffer, format="JPEG", quality=90)
    row = dict(
        acquired_at=stamp / 1e9,
        received_at=now,
        stamp_ns=stamp,
        processing_ms=processing_ms,
        age_at_receipt_ms=age_ms,
        model_sha256=message.model_sha256,
        rig_calibration_sha256=message.rig_calibration_sha256,
        motion_permitted=False,
        scope="Live simulated camera through ROS preprocessing and frozen DINOv3; static cell",
    )
    payload = dict(row, image="data:image/jpeg;base64," + base64.b64encode(buffer.getvalue()).decode())
    temporary = a.output / "perception.json.tmp"
    temporary.write_text(json.dumps(payload))
    os.replace(temporary, a.output / "perception.json")
    if a.record_dir:
        name = f"{len(rows):04d}.jpg"
        (a.record_dir / name).write_bytes(buffer.getvalue())
        rows.append(dict(row, file=name))
        (a.record_dir / "frames.json").write_text(json.dumps(rows, indent=2) + "\n")


node.create_subscription(Observation, BASE + "/observation", observe, SENSOR_QOS)
node.create_subscription(FeatureFrame, BASE + "/features", feature, SENSOR_QOS)
start = time.monotonic()
try:
    while time.monotonic() - start < a.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    print(f"Recorded {len(rows)} matched feature frames", flush=True)
    node.destroy_node()
    rclpy.shutdown()
