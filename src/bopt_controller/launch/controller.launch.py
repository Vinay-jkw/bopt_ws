from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        "robot_name",
        default_value="",
        description="Robot name"
    )

    robot_name = LaunchConfiguration("robot_name")

    return LaunchDescription([

        robot_name_arg,

        # =====================================================
        # ROS 2 CONTROL
        # =====================================================

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

        # =====================================================
        # BOPT KEY
        # /cmd_vel -> /bopt/key_cmd
        # =====================================================

        Node(
            package="bopt_controller",
            executable="bopt_key",
            name="bopt_key_node",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "wheelbase": 1.542,
                    "max_steering_angle": 1.5708,
                }
            ],
        ),

        # =====================================================
        # BOPT TWIST RELAY
        # Manual / NMPC / Hydraulic aggregation
        # =====================================================

        Node(
            package="bopt_controller",
            executable="bopt_twist_relay",
            name="bopt_twist_relay",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "control_mode": "manual",
                }
            ],
        ),

        # =====================================================
        # BOPT MAIN CONTROLLER
        # Safety + command conditioning
        # =====================================================

        Node(
            package="bopt_controller",
            executable="bopt_main_controller",
            name="bopt_main_controller",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "wheel_radius": 0.115,
                    "max_wheel_velocity": 3.0,
                    "max_steering_angle": 1.5708,
                    "control_dt": 0.05,
                    "lift_min": 0.0,
                    "lift_max": 0.095,
                    "command_timeout": 0.5,
                    "steering_tolerance": 0.03,
                    "steering_delay": 0.15,
                }
            ],
        ),

        # =====================================================
        # HYDRAULIC CONTROLLER
        # =====================================================

        Node(
            package="bopt_controller",
            executable="bopt_hydraulic_controller",
            name="bopt_hydraulic_controller",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "lift_min": 0.0,
                    "lift_max": 0.095,
                }
            ],
        ),

        # =====================================================
        # ODOMETRY
        # =====================================================

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
    ])