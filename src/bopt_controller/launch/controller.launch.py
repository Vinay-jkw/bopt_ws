from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="robot001",
    )

    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([
        robot_name_arg,

        Node(
            package="bopt_controller",
            executable="bopt_controller",
            name="bopt_controller",
            output="screen",
            parameters=[
                {"use_sim_time": True}
            ],
        ),

        Node(
            package="bopt_controller",
            executable="odometry_node",
            name="bopt_odometry",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_name": robot_name,
                }
            ],
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager",
                "controller_manager",
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "traction_joint_controller",
                "--controller-manager",
                "controller_manager",
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "steering_joint_controller",
                "--controller-manager",
                "controller_manager",
            ],
            output="screen",
        ),

        Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "lift_joint_controller",
                "--controller-manager",
                "controller_manager",
            ],
            output="screen",
        ),
    ])