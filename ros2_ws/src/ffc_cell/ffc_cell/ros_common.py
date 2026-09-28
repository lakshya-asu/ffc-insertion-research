from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
BASE = "/cell/cameras/macro"


def stamp_ns(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


def diagnostic(node, publisher, level, reason, values=None):
    result = DiagnosticArray()
    result.header.stamp = node.get_clock().now().to_msg()
    status = DiagnosticStatus()
    status.name = node.get_name()
    status.hardware_id = "sim_macro_observation"
    status.level = level
    status.message = reason
    status.values = [KeyValue(key=str(k), value=str(v)) for k, v in (values or {}).items()]
    result.status = [status]
    publisher.publish(result)
