# pyrefly: ignore [missing-import]
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    map_name = LaunchConfiguration("map_name")
    amcl_config = LaunchConfiguration("amcl_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    lifecycle_nodes = [
        "map_server",
        "amcl"
    ]

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="bopt_map",
        description="Map yaml filename without extension"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("bopt_localization"),
            "config",
            "amcl.yaml"
        ),
        description="Full path to AMCL config file"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock"
    )

    # --------------------------------------------------
    # Map path
    # --------------------------------------------------

    map_path = os.path.join(
        get_package_share_directory("bopt_localization"),
        "maps"
    )

    map_file = os.path.join(
        get_package_share_directory("bopt_localization"),
        "maps",
        "bopt_map.yaml"
    )

    # --------------------------------------------------
    # Map Server
    # --------------------------------------------------

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
            "yaml_filename": map_file,
            "use_sim_time": use_sim_time,
            },
        ],
    )

    # --------------------------------------------------
    # AMCL
    # --------------------------------------------------

    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        emulate_tty=True,
        parameters=[
            amcl_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # --------------------------------------------------
    # Lifecycle Manager
    # --------------------------------------------------

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "node_names": lifecycle_nodes,
                "autostart": True,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    return LaunchDescription([
        map_name_arg,
        amcl_config_arg,
        use_sim_time_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager,
    ])