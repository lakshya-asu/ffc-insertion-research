"""Replay RGB files as explicitly labelled new observations, never physical camera evidence."""

import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

from ffc_cell.ros_common import BASE, SENSOR_QOS


class Replay(Node):
    def __init__(self):
        super().__init__("macro_rgb_replay")
        self.declare_parameter("image", "")
        self.declare_parameter("calibration", "")
        self.declare_parameter("fps", 2.0)
        filename = self.get_parameter("image").value
        config = json.loads(Path(self.get_parameter("calibration").value).read_text())
        rgb = cv2.cvtColor(cv2.imread(filename), cv2.COLOR_BGR2RGB)
        if rgb.shape != (2048, 2448, 3):
            raise ValueError("Replay requires native macro RGB")
        self.image = CvBridge().cv2_to_imgmsg(rgb, encoding="rgb8")
        self.info = CameraInfo()
        self.info.width, self.info.height = 2448, 2048
        self.info.distortion_model = "plumb_bob"
        self.info.d = [0.0] * 5
        self.info.k = np.array(config["K"]).ravel().tolist()
        self.info.r = np.eye(3).ravel().tolist()
        self.info.p = np.column_stack([np.array(config["K"]), np.zeros(3)]).ravel().tolist()
        self.image.header.frame_id = self.info.header.frame_id = "macro_optical_frame"
        self.images = self.create_publisher(Image, BASE + "/image_raw", SENSOR_QOS)
        self.infos = self.create_publisher(CameraInfo, BASE + "/camera_info", SENSOR_QOS)
        fps = self.get_parameter("fps").value
        if not 0 < fps <= 30:
            raise ValueError("Replay FPS must be in (0, 30]")
        self.create_timer(1 / fps, self.publish)
        self.get_logger().warning("Static RGB file replay with new wall-clock stamps; not a live camera.")

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        self.image.header.stamp = self.info.header.stamp = stamp
        self.infos.publish(self.info)
        self.images.publish(self.image)


def main():
    rclpy.init()
    node = Replay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
