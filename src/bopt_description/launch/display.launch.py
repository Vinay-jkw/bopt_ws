from launch import LaunchDescription
from launch_ros.actions import Node

from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    robot_description = ParameterValue(
        Command([
            "xacro ",
            PathJoinSubstitution([
                FindPackageShare("bopt_description"),
                "urdf",
                "robot.urdf.xacro"
            ])
        ]),
        value_type=str
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("bopt_description"),
        "rviz",
        "display.rviz"
    ])

    return LaunchDescription([

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}]
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui"
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config]
        )
    ])
