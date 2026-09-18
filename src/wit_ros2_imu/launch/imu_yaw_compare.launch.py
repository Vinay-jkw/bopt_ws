"""Bring up the IMU driver, the frame transformer and the yaw CSV recorder.

    ros2 launch wit_ros2_imu imu_yaw_compare.launch.py

Assumes robot_state_publisher is already running (byd_bopt_description
display.launch.py), since the transformer needs imu_link -> base_link on
/tf_static. Set start_driver:=false if the WIT driver is already up.
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_csv = os.path.join(
        os.path.expanduser('~'),
        'imu_odom_yaw_{}.csv'.format(datetime.now().strftime('%Y%m%d_%H%M%S')))

    args = [
        DeclareLaunchArgument('start_driver', default_value='true',
                              description='Also start the WIT IMU serial driver'),
        DeclareLaunchArgument('target_frame', default_value='base_link',
                              description='Body frame to rotate the IMU data into'),
        DeclareLaunchArgument('odom_topic', default_value='/byd/byd_cpp_odometry',
                              description='Odometry topic to compare against'),
        DeclareLaunchArgument('amcl_topic', default_value='/current_pose',
                              description='AMCL-corrected pose (map -> base) to compare against'),
        DeclareLaunchArgument('output_path', default_value=default_csv,
                              description='Where to write the comparison CSV'),
        DeclareLaunchArgument('sample_rate_hz', default_value='20.0',
                              description='CSV sampling rate'),
    ]

    driver = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu_driver_node',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )

    transformer = Node(
        package='wit_ros2_imu',
        executable='imu_transformer',
        name='imu_transformer',
        output='screen',
        parameters=[{
            'input_topic': 'imu/data',
            'output_topic': 'imu/transformed',
            'target_frame': LaunchConfiguration('target_frame'),
        }],
    )

    comparator = Node(
        package='wit_ros2_imu',
        executable='yaw_comparator',
        name='yaw_comparator',
        output='screen',
        parameters=[{
            'imu_topic': 'imu/transformed',
            'odom_topic': LaunchConfiguration('odom_topic'),
            'amcl_topic': LaunchConfiguration('amcl_topic'),
            'output_path': LaunchConfiguration('output_path'),
            'sample_rate_hz': LaunchConfiguration('sample_rate_hz'),
        }],
    )

    return LaunchDescription(args + [driver, transformer, comparator])
