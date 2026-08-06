"""
teleop.launch.py
----------------
Standalone launch for keyboard teleoperation.
Assumes the simulation (or real robot) is already running.

Usage:
    ros2 launch bopt_controller teleop.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    robot_controller = Node(
        package='bopt_controller',
        executable='robot_controller',
        name='robot_controller',
        output='screen',
        parameters=[{
            'max_linear_vel':  1.0,
            'max_angular_vel': 1.5,
            'lift_min':        0.0,
            'lift_max':        0.095,
            'cmd_vel_timeout': 0.5,
        }]
    )

    teleop = Node(
        package='bopt_controller',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',   # open in its own terminal window
    )

    return LaunchDescription([
        robot_controller,
        teleop,
    ])
