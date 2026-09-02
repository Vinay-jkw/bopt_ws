from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo


def generate_launch_description():

    # 🟢 Joint State Broadcaster (FIRST)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # 🔵 Steering Controller
    steering_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "steering_joint_controller",
        ],
    )

    # 🔵 Traction Controller
    traction_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "traction_joint_controller",

        ],
    )

    reach_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "reach_joint_controller",

        ],
    )

    mast_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mast_joint_controller",
        
        ],
    )
    mast_02_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mast_02_joint_controller",
        
        ],
    )

    carriage_rotation_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "carriage_rotation_joint_controller",
        ],
    )

    carriage_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "carriage_joint_controller",
        ],
    )

    # 🟡 Your custom nodes (namespaced automatically via topics)
    rt_controller = Node(
        package="rt_controller",
        executable="rt_simple",
    )

    rt_nmpc_controller = Node(
        package="rt_controller",
        executable="rt_nmpc_controller",
    )

    rt_current_pose = Node(
        package="rt_controller",
        executable="rt_current_pose",
        parameters=[{"use_sim_time": True}]
    )

    rt_odom = Node(
        package="rt_controller",
        executable="rt_odom",
        parameters=[{"use_sim_time": True}]
    )

    manipulator_service_node = Node(
        package='rt_controller',
        executable='manipulator_service_node',
        output='screen'
    )
    
    twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rt_controller"),
                "launch",
                "twist_vel.launch.py"
            )
        ),
    )


    # 🔥 ORDER MATTERS (VERY IMPORTANT)
    return LaunchDescription([

        # Step 1
        TimerAction(
            period=2.0,
            actions=[joint_state_broadcaster]
        ),

        # Step 2
        TimerAction(
            period=4.0,
            actions=[steering_controller]
        ),

        # Step 3
        TimerAction(
            period=6.0,
            actions=[traction_controller]
        ),
        TimerAction(
            period=8.0,
            actions=[reach_controller, mast_controller, mast_02_controller, carriage_rotation_controller, carriage_controller]
        ),
        # Step 4 - Custom nodes
        TimerAction(
            period=10.0,
            actions=[
                rt_controller,
                rt_odom,
                rt_nmpc_controller,
                twist,
                manipulator_service_node,
                rt_current_pose
            ]
        ),
    ])