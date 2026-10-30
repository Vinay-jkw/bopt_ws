from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('bopt_fleet_manager'),
        'config', 'fleet_monitor.yaml'
    )
    return LaunchDescription([
        Node(
            package='bopt_fleet_manager',
            executable='fleet_monitor',
            name='bopt_fleet_monitor',
            output='screen',
            parameters=[config],
        )
    ])
