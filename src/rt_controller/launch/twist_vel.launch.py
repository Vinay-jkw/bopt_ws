import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    rt_controller_pkg = get_package_share_directory('rt_controller')
    
    twist_mux_launch = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "rt_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(rt_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(rt_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(rt_controller_pkg, "config", "twist_mux_joy.yaml"),
        }.items(),
    )
    rt_controller = Node(
        package="rt_controller",
        executable="rt_key",
    )
    twist_relay_node = Node(
        package="rt_controller",
        executable="twist_relay",
        name="twist_relay",    
        output="screen",
    )

    return LaunchDescription(
        [
            # twist_mux_launch,
            twist_relay_node,
            rt_controller,
        ]
    )