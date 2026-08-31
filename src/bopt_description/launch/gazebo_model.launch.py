from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os
from os import pathsep
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # model = LaunchConfiguration("model")

    bopt_description_dir = get_package_share_directory("bopt_description")
    # model_arg =DeclareLaunchArgument(
    #         name="model",
    #         default_value=os.path.join(
    #             get_package_share_directory("bopt_description"),
    #             "src", "description", "bopt_plugin.urdf"
    #         )
    #     )
    # 🔴 Use full filename directly
    world_name_arg = DeclareLaunchArgument(
        name="world_name",
        default_value="test.world"
    )

    # 🟢 Clean path join (NO PythonExpression)
    world_path = PathJoinSubstitution([
        bopt_description_dir,
        "worlds",
        LaunchConfiguration("world_name")
    ])

    # 🟡 Resource path
    model_path = str(Path(bopt_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(bopt_description_dir, "models")

    gazebo_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        model_path
    )

    # 🔵 Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            )
        ),
        launch_arguments={
            "gz_args": [world_path, " -r -v 4"]
        }.items()
    )
    # gazebo_plugin = Node(
    #             package="robot_state_publisher",
    #             executable="robot_state_publisher",
    #             parameters=[{
    #                 "robot_description": ParameterValue(
    #                     Command([
    #                         "xacro ",
    #                         LaunchConfiguration("model"),
    #                     ]),
    #                     value_type=str
    #                 ),
    #                 "use_sim_time": True,
    #             }]
    #         )
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
    )

    return LaunchDescription([
        world_name_arg,
        # model_arg,
        gazebo_resource_path,
        gazebo,
        # gazebo_plugin,
        gz_ros2_bridge
    ])