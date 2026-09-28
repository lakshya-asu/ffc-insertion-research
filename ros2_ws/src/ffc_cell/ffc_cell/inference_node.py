"""Acquisition-stamped, rig-bound segmentation; never motion authorization."""

import copy

import rclpy
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from ffc_interfaces.msg import FeatureFrame, Observation
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from ffc_cell.inference_gate import InferenceGate
from ffc_cell.macro_contract import CLASSES, PREPROCESSING_REVISION
from ffc_cell.mounted_predictor import MountedPredictor
from ffc_cell.ros_common import BASE, SENSOR_QOS, diagnostic


class Inference(Node):
    def __init__(self, engine_factory=MountedPredictor):
        super().__init__("mounted_macro_inference")
        for name, default in [
            ("model_dir", "/model"),
            ("backbone_dir", "/backbone"),
            ("rig_file", "/rig/camera.json"),
        ]:
            self.declare_parameter(name, default)
        self.engine = engine_factory(
            *(self.get_parameter(n).value for n in ["model_dir", "backbone_dir", "rig_file"])
        )
        self.gate = InferenceGate(self.engine.camera)
        self.bridge = CvBridge()
        self.output = self.create_publisher(FeatureFrame, BASE + "/features", SENSOR_QOS)
        self.diagnostics = self.create_publisher(DiagnosticArray, "/cell/diagnostics", 10)
        self.accepted = self.rejected = 0
        self.reason = "Waiting for reviewed camera TF and a fresh observation"
        self.create_subscription(
            TFMessage,
            "/tf_static",
            self.transforms,
            QoSProfile(
                depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE
            ),
        )
        self.create_subscription(Observation, BASE + "/observation", self.observe, SENSOR_QOS)
        self.create_timer(0.2, self.watchdog)
        self.get_logger().info("Frozen mounted model loaded; segmentation only, motion disabled")

    def transforms(self, message):
        for transform in message.transforms:
            t, q = transform.transform.translation, transform.transform.rotation
            try:
                self.gate.observe_transform(
                    transform.header.frame_id, transform.child_frame_id, [t.x, t.y, t.z], [q.x, q.y, q.z, q.w]
                )
            except ValueError as exc:
                self.reason = str(exc)
                diagnostic(self, self.diagnostics, DiagnosticStatus.ERROR, self.reason)

    def observe(self, message):
        try:
            self.gate.check(message, self.get_clock().now().nanoseconds)
            rgb = self.bridge.imgmsg_to_cv2(message.image, desired_encoding="rgb8")
            mask, scores, seconds = self.engine.predict(rgb)
            result = FeatureFrame()
            result.header = copy.deepcopy(message.header)
            result.mask = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            result.mask.header = copy.deepcopy(message.header)
            result.camera_info = copy.deepcopy(message.camera_info)
            result.calibration_id = message.calibration_id
            result.rig_calibration_sha256 = self.engine.camera.fingerprint
            result.model_sha256 = self.engine.model_sha256
            result.preprocessing_revision = PREPROCESSING_REVISION
            result.class_names = list(CLASSES)
            result.mean_scores = scores
            duration = int(seconds * 1e9)
            result.processing_time.sec, result.processing_time.nanosec = divmod(duration, 1_000_000_000)
            result.motion_permitted = False
            self.gate.complete(message, self.get_clock().now().nanoseconds)
            self.output.publish(result)
            self.accepted += 1
            self.reason = ""
            diagnostic(
                self,
                self.diagnostics,
                DiagnosticStatus.OK,
                "Fresh segmentation; pose unverified",
                {"accepted": self.accepted, "rejected": self.rejected, "motion_permitted": False},
            )
        except (ValueError, TypeError, RuntimeError) as exc:
            self.rejected += 1
            self.reason = str(exc)
            diagnostic(
                self,
                self.diagnostics,
                DiagnosticStatus.ERROR,
                self.reason,
                {"accepted": self.accepted, "rejected": self.rejected},
            )

    def watchdog(self):
        age = self.get_clock().now().nanoseconds - self.gate.guard.last_ns
        if self.reason or not 0 <= age <= self.gate.guard.max_age_ns:
            diagnostic(
                self,
                self.diagnostics,
                DiagnosticStatus.ERROR,
                self.reason or "No fresh feature frame; previous masks are expired",
            )


def main():
    rclpy.init()
    node = None
    try:
        node = Inference()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
