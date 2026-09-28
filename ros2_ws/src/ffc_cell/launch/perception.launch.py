from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # No actuator node, controller, or simulator truth topic is included.
    return LaunchDescription(
        [
            Node(package="ffc_cell", executable="preprocess", output="screen"),
            Node(package="ffc_cell", executable="supervise", output="screen"),
        ]
    )
