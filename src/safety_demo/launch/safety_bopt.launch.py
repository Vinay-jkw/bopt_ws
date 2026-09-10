from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="safety_demo",
            executable="safety_node_viz",
            name="safety_node",
            output="screen",
        )
    ])
