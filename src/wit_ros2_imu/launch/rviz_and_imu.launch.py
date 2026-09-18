import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    package_name = 'wit_ros2_imu'  

    pkg_share = FindPackageShare(package=package_name).find(package_name)

    # ------------------------
    # IMU node
    # ------------------------
    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='imu',
        parameters=[
            {'port': '/dev/ttyUSB0'},
            {'baud': 9600}
        ],
        output='screen'
    )

    # ------------------------
    # EKF config
    # ------------------------
    ekf_yaml = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # ekf_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[ekf_yaml]
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'imu_link']
    )

    return LaunchDescription([
        imu_node,
        rviz_node,
        tf_node,
        # ekf_node
    ])
