import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

import math

def generate_launch_description():
    robot_name = LaunchConfiguration("robot_name")
    amcl_config = LaunchConfiguration("amcl_config")
    lifecycle_nodes = ["amcl"]
    robot_name_arg = DeclareLaunchArgument("robot_name")

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bopt_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to amcl yaml file to load"
    )

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {"use_sim_time": True,}
        ],
        remappings=[
        ("map", "/map")   
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
        robot_name_arg,
        amcl_config_arg,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])