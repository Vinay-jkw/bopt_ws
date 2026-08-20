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


    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )


    traction_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'traction_joint_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )



    steering_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'steering_joint_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )



    lift_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'lift_joint_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    bopt_controller = Node(
        package='bopt_controller',
        executable='bopt_controller',
        output='screen',
        parameters=[
            {
                'use_sim_time': True
            }
        ]
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

            '/lidar/top3dl/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/lidar/right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
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

    bopt_controller_delayed = TimerAction(
        period=3.0,
        actions=[
            bopt_controller
        ]
    )

    return LaunchDescription([
        gui_arg,
        gazebo,
        # Controllers
        joint_state_broadcaster,
        traction_controller,
        steering_controller,
        lift_controller,
        odometry_node,
        sensor_bridge,

        bopt_controller_delayed,
        rviz
    ])