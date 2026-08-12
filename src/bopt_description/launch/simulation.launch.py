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


    # mimic_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     arguments=[
    #         'mimic_joint_controller',
    #         '--controller-manager',
    #         '/controller_manager'
    #     ],
    #     output='screen'
    # )


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



    # mimic_controller_delayed = TimerAction(
    #     period=5.0,
    #     actions=[
    #         mimic_controller
    #     ]
    # )



    return LaunchDescription([

        # GUI
        gui_arg,

        # Gazebo
        gazebo,

        # Controllers
        joint_state_broadcaster,
        traction_controller,
        steering_controller,
        lift_controller,

        # High-level controller FIRST (must be running before mimic)
        bopt_controller_delayed,
# 
        # Mimic joint controller AFTER bopt is running
        # mimic_controller_delayed,

        # RViz
        rviz
    ])