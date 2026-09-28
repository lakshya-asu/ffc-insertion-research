import rclpy
from ffc_interfaces.msg import Observation, TaskStatus
from rclpy.node import Node

from ffc_cell.guard import FrameGuard
from ffc_cell.observation_contract import validate_observation
from ffc_cell.ros_common import BASE, SENSOR_QOS, stamp_ns


class Supervisor(Node):
    def __init__(self):
        super().__init__("observation_supervisor")
        self.output = self.create_publisher(TaskStatus, "/cell/task/status", 10)
        self.subscription = self.create_subscription(
            Observation, BASE + "/observation", self.observe, SENSOR_QOS
        )
        self.last = None
        self.guard = FrameGuard()
        self.rejection = "No observation received"
        self.create_timer(0.1, self.tick)

    def observe(self, message):
        try:
            stamp, identity = validate_observation(message)
            self.guard.check(stamp, self.get_clock().now().nanoseconds, identity)
            self.guard.accept(stamp, identity)
            self.last = message.header
            self.rejection = ""
        except (ValueError, TypeError, RuntimeError) as exc:
            self.last = None
            self.rejection = str(exc)

    def tick(self):
        now = self.get_clock().now()
        result = TaskStatus()
        result.header.stamp = now.to_msg()
        result.skill = "observe_entrance"
        result.motion_permitted = False
        if self.last is None or not 0 <= now.nanoseconds - stamp_ns(self.last) <= 500_000_000:
            result.state = "stale"
            result.reason = self.rejection or "No fresh observation. No actuator endpoint is connected."
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
