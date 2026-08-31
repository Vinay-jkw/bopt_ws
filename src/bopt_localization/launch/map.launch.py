import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    map_path = PathJoinSubstitution([
        get_package_share_directory("workflow_node"),
        "map_details",
        "demo_warehouse.yaml"
    ])

    lifecycle_nodes = ["map_server"]

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        nav2_map_server,
        nav2_lifecycle_manager
    ])