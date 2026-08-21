import os

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("bopt_mapping"),
            "config",
            "slam_toolbox.yaml"
        ),
        description="Full path to slam yaml file to load"
    )

    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox,
    ])