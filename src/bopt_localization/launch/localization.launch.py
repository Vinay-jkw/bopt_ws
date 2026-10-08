# pyrefly: ignore [missing-import]

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ============================================================
    # Launch configuration
    # ============================================================

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ============================================================
    # Package paths
    # ============================================================

    bopt_localization_pkg = get_package_share_directory(
        "bopt_localization"
    )

    costmap_pkg = get_package_share_directory(
        "costmap_pkg"
    )

    costmap_to_laserscan_pkg = get_package_share_directory(
        "costmap_to_laserscan"
    )

    # ============================================================
    # Configuration files
    # ============================================================

    amcl_config = os.path.join(
        bopt_localization_pkg,
        "config",
        "amcl.yaml",
    )

    ekf_config = os.path.join(
        bopt_localization_pkg,
        "config",
        "ekf.yaml",
    )

    stvl_config = os.path.join(
        costmap_pkg,
        "config",
        "stvl_costmap.yaml",
    )

    # ============================================================
    # Map
    # ============================================================

    map_file = os.path.join(
        bopt_localization_pkg,
        "maps",
        "RB_amcl.yaml",
    )

    # ============================================================
    # Launch arguments
    # ============================================================

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock simulation time",
    )

    # ============================================================
    # Map Server
    # ============================================================

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {
                "yaml_filename": map_file,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ============================================================
    # AMCL
    # ============================================================

    amcl = Node(
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

    # ============================================================
    # IMU covariance relay
    # ============================================================

    imu_relay = Node(
        package="bopt_localization",
        executable="imu_covariance_relay",
        name="imu_covariance_relay",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # ============================================================
    # EKF
    #
    # Publishes:
    #
    #     odom -> base_footprint
    #
    # Your current EKF configuration already has:
    #     world_frame: odom
    #     odom_frame: odom
    #     base_link_frame: base_footprint
    #     publish_tf: true
    # ============================================================

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            ekf_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # ============================================================
    # Standalone Costmap + STVL
    #
    # This executable creates /costmap/costmap.
    # ============================================================

    costmap = Node(
        package="nav2_costmap_2d",
        executable="nav2_costmap_2d",
        name="costmap",
        output="screen",
        parameters=[
            stvl_config,
            {
                "use_sim_time": use_sim_time,
            },
        ],
    )

    # ============================================================
    # Lifecycle Manager - Localization
    #
    # These nodes support the standard Nav2 lifecycle bond.
    # ============================================================

    localization_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "node_names": [
                    "map_server",
                    "amcl",
                ],
                "autostart": True,
                "use_sim_time": use_sim_time,

                # Normal Nav2 bond monitoring.
                "bond_timeout": 4.0,

                # Keep normal respawn/bond behavior.
                "attempt_respawn_reconnection": True,
                "bond_respawn_max_duration": 10.0,
            }
        ],
    )

    # ============================================================
    # Lifecycle Manager - Standalone Costmap
    #
    # IMPORTANT:
    #
    # Humble's standalone Costmap2DROS does not create the
    # lifecycle bond in its on_activate().
    #
    # Therefore this manager must NOT wait for a bond.
    #
    # bond_timeout = 0 disables bond creation/waiting in
    # LifecycleManager.
    # ============================================================

    costmap_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_costmap",
        output="screen",
        parameters=[
            {
                "node_names": [
                    "costmap/costmap",
                ],
                "autostart": True,
                "use_sim_time": use_sim_time,

                # Standalone Costmap2DROS in Humble does not
                # create the lifecycle bond.
                "bond_timeout": 0.0,

                # These are irrelevant when bond_timeout == 0,
                # but keeping them explicit makes the configuration clear.
                "attempt_respawn_reconnection": False,
            }
        ],
    )

    # ============================================================
    # Costmap -> LaserScan
    # ============================================================

    costmap_to_laserscan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                costmap_to_laserscan_pkg,
                "launch",
                "costmap_to_laserscan.launch.py",
            )
        )
    )

    # ============================================================
    # Current pose publisher
    # ============================================================

    current_pose_publisher = Node(
        package="bopt_localization",
        executable="current_pose_publisher",
        name="current_pose_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_frame": "map",
                "base_frame": "base_footprint",
                "pose_topic": "current_pose",
                "publish_rate": 20.0,
            }
        ],
    )

    # ============================================================
    # Launch
    # ============================================================

    return LaunchDescription(
        [
            # Parameters
            use_sim_time_arg,

            # Localization / state estimation
            imu_relay,
            ekf_node,

            # Nav2 lifecycle nodes
            map_server,
            amcl,


            # Lifecycle managers
            localization_lifecycle_manager,
            # costmap_lifecycle_manager,

            # Standalone costmap

            # costmap,

            # Auxiliary nodes
            current_pose_publisher,
            costmap_to_laserscan_launch,
        ]
    )