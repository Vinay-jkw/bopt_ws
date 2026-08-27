from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def create_robot(robot_name, x, y, yaw):

    return GroupAction([

        # 🔴 Namespace isolation
        PushRosNamespace(robot_name),

        # 🟢 Spawn robot in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("rt_description"),
                    "launch",
                    "gazebo.launch.py"
                )
            ),
            launch_arguments={
                "robot_name": robot_name,
                "x": str(x),
                "y": str(y),
                "yaw": str(yaw),
            }.items()
        ),
        # LogInfo(msg=["Robot Name: ", robot_name]),
        # # 🔵 Controllers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("rt_controller"),
                    "launch",
                    "controller.launch.py"
                )
            ),
            launch_arguments={
                "robot_name": robot_name
            }.items()
        ),

        # # 🟣 Localization (AMCL)
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(
        #             get_package_share_directory("rt_localization"),
        #             "launch",
        #             "localization.launch.py"
        #         )
        #     )
        # ),
    ])


def generate_launch_description():

    num_robots = 2   # 🔥 CHANGE THIS ONLY

    actions = []

    for i in range(num_robots):
        robot_name = f"robot{i+1:03d}"

        # 🔹 Grid positioning (BEST for FMS / warehouse)
        cols = 3
        spacing = 5.0

        x = (i % cols) * spacing
        y = (i // cols) * spacing
        z = 0.0

        # 🔥 Sequential delay (VERY IMPORTANT)
        delay = 10.0

        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    create_robot(robot_name, x, y, z)
                ]
            )
        )

    return LaunchDescription(actions)