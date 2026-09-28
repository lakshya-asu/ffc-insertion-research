import rclpy
from ffc_interfaces.msg import Observation, TaskStatus
from rclpy.node import Node

from ffc_cell.cv_frontend import REVISION
from ffc_cell.ros_common import BASE, SENSOR_QOS, stamp_ns


class Supervisor(Node):
    def __init__(self):
        super().__init__("observation_supervisor")
        self.output = self.create_publisher(TaskStatus, "/cell/task/status", 10)
        self.subscription = self.create_subscription(
            Observation, BASE + "/observation", self.observe, SENSOR_QOS
        )
        self.last = None
        self.create_timer(0.1, self.tick)

    def observe(self, message):
        if message.preprocessing_revision != REVISION or message.image.header != message.header:
            self.last = None
            return
        self.last = message.header

    def tick(self):
        now = self.get_clock().now()
        result = TaskStatus()
        result.header.stamp = now.to_msg()
        result.skill = "observe_entrance"
        result.motion_permitted = False
        if self.last is None or not 0 <= now.nanoseconds - stamp_ns(self.last) <= 500_000_000:
            result.state = "stale"
            result.reason = "No fresh observation. No actuator endpoint is connected."
        else:
            result.state = "observation_only"
            result.reason = (
                "RGB preprocessing healthy; verified pose, contact feedback and control gates absent."
            )
        self.output.publish(result)


def main():
    rclpy.init()
    node = Supervisor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
