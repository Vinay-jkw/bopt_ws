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

    # ============================================================
    # GUI ARGUMENT
    # ============================================================

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Set to "false" to run Gazebo headless'
    )

    # ============================================================
    # PACKAGE PATH
    # ============================================================

    package_path = get_package_share_directory(
        'bopt_description'
    )

    # ============================================================
    # GAZEBO
    # ============================================================

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

    # ============================================================
    # JOINT STATE BROADCASTER
    # ============================================================

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

    # ============================================================
    # TRACTION CONTROLLER
    # ============================================================

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

    # ============================================================
    # STEERING CONTROLLER
    # ============================================================

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

    # ============================================================
    # LIFT CONTROLLER
    # ============================================================

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

    # ============================================================
    # MIMIC JOINT CONTROLLER
    # ============================================================

    mimic_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'mimic_joint_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    # ============================================================
    # BOPT HIGH-LEVEL CONTROLLER
    # ============================================================

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

    # ============================================================
    # RVIZ
    # ============================================================

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

    # ============================================================
    # DELAY MIMIC CONTROLLER
    # ============================================================

    mimic_controller_delayed = TimerAction(
        period=2.0,
        actions=[
            mimic_controller
        ]
    )

    # ============================================================
    # DELAY BOPT CONTROLLER
    # ============================================================

    bopt_controller_delayed = TimerAction(
        period=3.0,
        actions=[
            bopt_controller
        ]
    )

    # ============================================================
    # LAUNCH EVERYTHING
    # ============================================================

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

        # Mimic joint controller
        mimic_controller_delayed,

        # High-level controller
        bopt_controller_delayed,

        # RViz
        rviz
    ])