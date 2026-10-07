#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():

    # ==========================================
    # STVL Config File
    # ==========================================

    stvl_config = LaunchConfiguration(
        'stvl_config'
    )

    declare_stvl_config = DeclareLaunchArgument(
        'stvl_config',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'multi_lidar_ws',
            'src',
            'costmap_pkg',
            'config',
            'stvl_costmap.yaml'
        ),
        description='STVL Costmap Config'
    )

    # ==========================================
    # Costmap Node
    # ==========================================

    costmap_node = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        # nav2_costmap_2d creates a Costmap2DROS node named "costmap".
        # Keep this name aligned with the YAML root key and lifecycle manager.
        name='costmap',
        output='screen',
        parameters=[stvl_config]
    )

    # ==========================================
    # Lifecycle Manager
    # ==========================================

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'autostart': True,
                'bond_timeout': 0.0,
                'node_names': ['costmap/costmap']
            }
        ]
    )

    return LaunchDescription([

        declare_stvl_config,

        costmap_node,

        lifecycle_manager

    ])
