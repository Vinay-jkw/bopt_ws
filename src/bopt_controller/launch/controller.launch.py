"""
controller.launch.py
--------------------
Launches the robot_controller_node alongside the full simulation stack.
Usage:
    ros2 launch bopt_controller controller.launch.py [gui:=false]
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless'
    )

    # ── Bring up the full simulation (Gazebo + ros2_control) ────────────
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('bopt_description'),
                'launch',
                'simulation.launch.py'
            )
        ),
        launch_arguments={'gui': LaunchConfiguration('gui')}.items()
    )

    # ── High-level robot controller ─────────────────────────────────────
    robot_controller = Node(
        package='bopt_controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'use_sim_time':    True,
            'max_linear_vel':  1.0,
            'max_angular_vel': 3.0,
            'lift_min':        0.0,
            'lift_max':        0.095,
            'cmd_vel_timeout': 0.5,
        }]
    )

    return LaunchDescription([
        gui_arg,
        simulation,
        robot_controller,
    ])
