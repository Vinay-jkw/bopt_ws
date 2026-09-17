import os

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction
)

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration


def generate_launch_description():



    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless'
    )



    package_path = get_package_share_directory(
        'bopt_description'
    )

    localization_path = get_package_share_directory(
        'bopt_localization'
    )
    controller_path = get_package_share_directory(
        'bopt_controller'
    )



    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                package_path,
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'gui': LaunchConfiguration('gui')
        }.items()
    )


    odometry_node = Node(
        package='bopt_controller',
        executable='odometry_node',
        name='bopt_odometry',
        output='screen',
        parameters=[
        {'use_sim_time': True}
    ]
    )
    sensor_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/lidar/top3dl/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/back/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/back/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/back/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/lidar/left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/Lidar_LFT@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/Lidar_RFT@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/left/range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/ultrasonic/right/range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )



    rviz_config_file = os.path.join(
        package_path,
        'rviz',
        'display.rviz'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            rviz_config_file
        ],
        parameters=[
            {
                'use_sim_time': True
            }
        ],
        output='screen'
    )


    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                localization_path,
                'launch',
                'localization.launch.py'
            )
        )
    )
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                controller_path,
                'launch',
                'controller.launch.py'
            )
        ),
        launch_arguments={
            'robot_name': ''
        }.items()
    )
    controller_delayed = TimerAction(
        period=3.0,
        actions=[
            controller_launch
        ]
    )
    localization_delayed = TimerAction(
        period=2.0,
        actions=[
            localization_launch
        ]
    )

    return LaunchDescription([
        gui_arg,

        gazebo,

        # BOPT controller stack
        controller_delayed,

        # Sensors
        sensor_bridge,

        # Localization
        localization_delayed,

        # Visualization
        rviz,
    ])