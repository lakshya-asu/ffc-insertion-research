"""Standard ROS camera messages from a rendered sensor and fixed calibration only."""

import numpy as np


class RosCamera:
    def __init__(self, k, world_optical, size=(2448, 2048)):
        import rclpy
        from geometry_msgs.msg import TransformStamped
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
        from scipy.spatial.transform import Rotation
        from sensor_msgs.msg import CameraInfo, Image
        from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

        rclpy.init()
        self.rclpy = rclpy
        self.node = rclpy.create_node("isaac_macro_camera")
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.image_type = Image
        self.info = CameraInfo()
        self.info.header.frame_id = "macro_optical_frame"
        self.info.width, self.info.height = size
        self.info.k = np.asarray(k).ravel().tolist()
        self.info.r = np.eye(3).ravel().tolist()
        self.info.p = np.column_stack([k, np.zeros(3)]).ravel().tolist()
        self.info.d = [0.0] * 5
        self.info.distortion_model = "plumb_bob"
        self.images = self.node.create_publisher(Image, "/cell/cameras/macro/image_raw", qos)
        self.infos = self.node.create_publisher(CameraInfo, "/cell/cameras/macro/camera_info", qos)
        self.tf = StaticTransformBroadcaster(self.node)
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = "macro_optical_frame"
        xyz = np.asarray(world_optical)[:3, 3]
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        q = Rotation.from_matrix(np.asarray(world_optical)[:3, :3]).as_quat()
        transform.transform.rotation.x = float(q[0])
        transform.transform.rotation.y = float(q[1])
        transform.transform.rotation.z = float(q[2])
        transform.transform.rotation.w = float(q[3])
        self.tf.sendTransform(transform)

    def publish(self, rgb):
        if rgb.shape != (self.info.height, self.info.width, 3) or rgb.dtype != np.uint8:
            raise ValueError("Native RGB shape/type mismatch")
        image = self.image_type()
        image.header.frame_id = "macro_optical_frame"
        image.header.stamp = self.node.get_clock().now().to_msg()
        image.height, image.width = rgb.shape[:2]
        image.encoding = "rgb8"
        image.is_bigendian = 0
        image.step = image.width * 3
        image.data = np.ascontiguousarray(rgb).tobytes()
        self.info.header.stamp = image.header.stamp
        self.infos.publish(self.info)
        self.images.publish(image)
        self.rclpy.spin_once(self.node, timeout_sec=0)

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()
