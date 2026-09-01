import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    bopt_description_dir = get_package_share_directory(
        "bopt_description"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=os.path.join(
            bopt_description_dir,
            "urdf",
            "robot.urdf.xacro"
        )
    )

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot001",
        description="Unique name / namespace for this robot"
    )

    robot_param_node_arg = DeclareLaunchArgument(
        "robot_param_node",
        default_value="robot_state_publisher"
    )

    robot_namespace_arg = DeclareLaunchArgument(
        "robot_namespace",
        default_value=""
    )

    x_arg = DeclareLaunchArgument(
        "x",
        default_value="0.0"
    )

    y_arg = DeclareLaunchArgument(
        "y",
        default_value="0.0"
    )

    yaw_arg = DeclareLaunchArgument(
        "yaw",
        default_value="0.0"
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            LaunchConfiguration("model"),
            " robot_name:=",
            LaunchConfiguration("robot_name"),
            " robot_namespace:=",
            LaunchConfiguration("robot_namespace"),
            " robot_param_node:=",
            LaunchConfiguration("robot_param_node"),
        ]),
        value_type=str
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }]
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            [
                "/",
                LaunchConfiguration("robot_name"),
                "/robot_description"
            ],

            "-name",
            LaunchConfiguration("robot_name"),

            "-x",
            LaunchConfiguration("x"),

            "-y",
            LaunchConfiguration("y"),

            "-Y",
            LaunchConfiguration("yaw"),
        ],
    )

    return LaunchDescription([
        robot_name_arg,
        model_arg,
        robot_param_node_arg,
        robot_namespace_arg,
        x_arg,
        y_arg,
        yaw_arg,

        # Start robot_state_publisher immediately
        robot_state_publisher,

        # Delay entity spawn by 2 s so RSP has time to set its
        # robot_description parameter before ign_ros2_control calls it
        TimerAction(
            period=2.0,
            actions=[spawn_entity]
        ),
    ])