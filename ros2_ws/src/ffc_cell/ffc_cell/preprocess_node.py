import copy

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ffc_interfaces.msg import Observation
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from ffc_cell.cv_frontend import REVISION, Frontend, calibration_id
from ffc_cell.guard import FrameGuard
from ffc_cell.ros_common import BASE, SENSOR_QOS, diagnostic, stamp_ns


class Preprocess(Node):
    def __init__(self):
        super().__init__("macro_preprocess")
        self.declare_parameter("max_age_ms", 500)
        self.declare_parameter("source_profile", "macro-mount-elevation45-v1")
        self.bridge = CvBridge()
        self.frontend = Frontend()
        self.guard = FrameGuard(self.get_parameter("max_age_ms").value * 1_000_000)
        self.output = self.create_publisher(Observation, BASE + "/observation", SENSOR_QOS)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/cell/diagnostics", 10)
        sync_qos = QoSProfile(depth=4, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.images = message_filters.Subscriber(self, Image, BASE + "/image_raw", qos_profile=sync_qos)
        self.infos = message_filters.Subscriber(self, CameraInfo, BASE + "/camera_info", qos_profile=sync_qos)
        self.sync = message_filters.TimeSynchronizer([self.images, self.infos], 4)
        self.sync.registerCallback(self.process)
        self.accepted = 0
        self.rejected = 0
        self.create_timer(0.2, self.watchdog)

    def watchdog(self):
        age = self.get_clock().now().nanoseconds - self.guard.last_ns
        if self.guard.last_ns < 0 or age > self.guard.max_age_ns:
            diagnostic(
                self,
                self.diagnostics,
                DiagnosticStatus.ERROR,
                "No fresh synchronized camera frame",
                {"accepted": self.accepted, "rejected": self.rejected},
            )

    def process(self, image, info):
        try:
            if image.header != info.header or image.header.frame_id != "macro_optical_frame":
                raise ValueError("image/calibration optical frame or stamp mismatch")
            if image.encoding != "rgb8" or image.is_bigendian:
                raise ValueError("expected little-endian RGB8")
            if (image.width, image.height) != (2448, 2048) or (info.width, info.height) != (2448, 2048):
                raise ValueError("wrong native image or calibration dimensions")
            if image.step < image.width * 3 or len(image.data) != image.step * image.height:
                raise ValueError("invalid image payload")
            if (
                info.binning_x not in [0, 1]
                or info.binning_y not in [0, 1]
                or info.roi.width
                or info.roi.height
            ):
                raise ValueError("unexpected hardware ROI or binning")
            if not np.allclose(info.r, np.eye(3).ravel()):
                raise ValueError("non-identity rectification rotation requires another profile")
            identity = calibration_id(info.width, info.height, info.k, info.d, info.header.frame_id)
            stamp = stamp_ns(image.header)
            self.guard.check(stamp, self.get_clock().now().nanoseconds, identity)
            rgb = self.bridge.imgmsg_to_cv2(image, desired_encoding="rgb8")
            window, k, edges, quality = self.frontend.process(rgb, info.k, info.d, info.distortion_model)
            self.guard.check(stamp, self.get_clock().now().nanoseconds, identity)
            result = Observation()
            result.header = image.header
            result.image = self.bridge.cv2_to_imgmsg(window, encoding="rgb8")
            result.image.header = image.header
            result.edges = self.bridge.cv2_to_imgmsg(edges, encoding="mono8")
            result.edges.header = image.header
            result.camera_info = copy.deepcopy(info)
            result.camera_info.width, result.camera_info.height = 1232, 1024
            result.camera_info.k = k.ravel().tolist()
            result.camera_info.d = [0.0] * 5
            result.camera_info.p = np.column_stack([k, np.zeros(3)]).ravel().tolist()
            result.calibration_id = identity
            result.preprocessing_revision = REVISION
            result.source_profile = self.get_parameter("source_profile").value
            result.quality_names = list(quality)
            result.quality_values = list(quality.values())
            self.output.publish(result)
            self.guard.accept(stamp, identity)
            self.accepted += 1
            diagnostic(
                self,
                self.diagnostics,
                DiagnosticStatus.OK,
                "Fresh rectified RGB observation",
                dict(quality, accepted=self.accepted, rejected=self.rejected, motion_permitted=False),
            )
        except (ValueError, TypeError, RuntimeError) as exc:
            self.rejected += 1
            diagnostic(self, self.diagnostics, DiagnosticStatus.ERROR, str(exc), {"rejected": self.rejected})


def main():
    rclpy.init()
    node = Preprocess()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
