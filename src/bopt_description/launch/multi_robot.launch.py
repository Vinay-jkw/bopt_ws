from launch import LaunchDescription
from launch.actions import GroupAction, TimerAction
from launch_ros.actions import PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from nav2_common.launch import RewrittenYaml
from launch_ros.parameter_descriptions import ParameterFile
from launch.actions import OpaqueFunction, SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
def spawn_robot_with_params(context, mqtt_bridge_params_file, params_file, amcl_params_file, robot_name, x, y, yaw, ip):

    rewritten_controller = RewrittenYaml(
        source_file=params_file,
        root_key=robot_name,
        param_rewrites={},
        convert_types=True,
    )
    px, py, pyaw = x, y, yaw # TODO: initalpose is hardcoded for now as it needs map origin for conversion.
    match robot_name:
        case 'robot001':
            px, py, pyaw = 6.89593, 10.5518, -0.0114491
        case 'robot002':
            px, py, pyaw = 6.87761, 5.47154, -0.0114491
    rewritten_amcl = RewrittenYaml(
        source_file=amcl_params_file,
        root_key=robot_name,
        param_rewrites={
            "amcl.ros__parameters.initial_pose.x": str(px),
            "amcl.ros__parameters.initial_pose.y": str(py),
            "amcl.ros__parameters.initial_pose.yaw": str(pyaw),
            "amcl.ros__parameters.set_initial_pose": 'True',
            "amcl.ros__parameters.base_frame_id": robot_name+"/base_footprint",
            "amcl.ros__parameters.odom_frame_id": robot_name+"/odom",
            
        },
        convert_types=True,
    )
    rewritten_mqtt_bridge = RewrittenYaml(
        source_file=mqtt_bridge_params_file,
        # root_key=robot_name,
        param_rewrites={
            "ros2_mqtt_bridge.mqtt_bridge.robot_id": robot_name,
            "ros2_mqtt_bridge.mqtt_bridge.robot_ip": ip,

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.path": "path",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.holded_nodes": "holded_nodes",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.current_pose": "current_pose",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.odometry": "odom",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.remaining_path": "remaining_path",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.task_request": "task_request",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.add_nodes": "add_nodes",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.local_path_active": "local_path_active",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.task": "task",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.conflict_action": "conflict_action",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.task_action": "task_action",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.safety_status": "safety_status",
            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.robot_dimensions": "robot_dimensions",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.current_pose": f"{robot_name}/current_pose",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.holded_nodes": f"{robot_name}/holded_nodes",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.path": f"{robot_name}/path",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.safety_status": f"{robot_name}/safety_status",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.odometry": f"{robot_name}/odometry/filtered",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.remaining_path": f"{robot_name}/remaining_path",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.task_request": f"{robot_name}/task_request",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.add_nodes": f"{robot_name}/add_nodes",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.local_path_active": f"{robot_name}/local_path_active",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.task": f"{robot_name}/task",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.conflict_action": f"{robot_name}/conflict_action",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.task_action": f"{robot_name}/task_action",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.safety_status": f"{robot_name}/safety_status",
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.dimensions": f"{robot_name}/robot_dimensions",
        },
        convert_types=True,
    )
    controller_yaml_path = rewritten_controller.perform(context)
    amcl_yaml_path = rewritten_amcl.perform(context)
    mqtt_bridge_yaml_path = rewritten_mqtt_bridge.perform(context)

    print(f"\n[DEBUG] {robot_name} YAML → {controller_yaml_path} AMCL → {amcl_yaml_path} \n")

    return [
        TimerAction(
            # Give Gazebo 5 s to fully load before attempting first spawn
            period=5.0,
            actions=[
                GroupAction([

                    PushRosNamespace(robot_name),

                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("bopt_description"),
                                "launch",
                                "spawn_robot.launch.py"  # ← only spawns, no new world
                            )
                        ),
                        launch_arguments={
                            "robot_name": robot_name,
                            "x": str(x),
                            "y": str(y),
                            "yaw": str(yaw),
                        }.items()
                    ),

                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("bopt_controller"),
                                "launch",
                                "controller.launch.py"
                            )
                        ),
                        launch_arguments={
                            "robot_name": robot_name,
                        }.items()
                    ),

                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("bopt_localization"),
                                "launch",
                                "robot_localize.launch.py"
                            )
                        ),
                        launch_arguments={
                            "robot_name": robot_name,
                            "amcl_config": amcl_yaml_path
                        }.items()
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("ros2_mqtt_bridge"),
                                "launch",
                                "mqtt_bridge.launch.py"
                            )
                        ),
                        launch_arguments={
                            "config_file": mqtt_bridge_yaml_path,
                        }.items()
                    ),
                ])
            ]
        ),

    ]


def generate_launch_description():
    spawn_world =IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("bopt_description"),
                    "launch",
                    "gazebo_model.launch.py"
                )
            )
    )
    map_server=IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("bopt_localization"),
                    "launch",
                    "map.launch.py"
                )
            )
        )
    
    controller_params_file = os.path.join(
        get_package_share_directory("bopt_description"),
        "config",
        "controller.yaml"
    )
    amcl_params_file = os.path.join(
        get_package_share_directory("bopt_localization"),
        "config",
        "amcl.yaml"
    )
    mqtt_bridge_params_file = os.path.join(
        get_package_share_directory("ros2_mqtt_bridge"),
        "config",
        "properties.yaml"
    )
    num_robots = 2   # 🔥 CHANGE THIS ONLY

    actions = []
    for i in range(num_robots):
        robot_name = f"robot{i+1:03d}"

        cols = 1
        spacing = -5.0

        x = (i % cols) * spacing
        y = (i // cols) * spacing
        yaw = 0.0

        delay = i * 15.0  # 15 s between each robot to avoid spawn race conditions

        ip_alias = f"127.0.0.{i+2}"
        # interface = "wlp8s0"

        # # 🔥 Command to add IP alias
        # add_ip = ExecuteProcess(
        #     cmd=[
        #         "sudo", "ip", "addr", "add",
        #         f"{ip_alias}/24", "dev", interface
        #     ],
        #     output="screen"
        # )

        # # 🔥 Optional: add hostname mapping
        # add_host = ExecuteProcess(
        #     cmd=[
        #         "sudo", "bash", "-c",
        #         f"echo '{ip_alias} {robot_name}' >> /etc/hosts"
        #     ],
        #     output="screen"
        # )

        print(f"\n[DEBUG] {robot_name} IP →  {ip_alias}\n")

        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    # add_ip,
                    # add_host,
                    OpaqueFunction(
                        function=spawn_robot_with_params,
                        args=[mqtt_bridge_params_file, controller_params_file, amcl_params_file, robot_name, x, y, yaw, ip_alias]
                    )
                ]
            )
        )

    return LaunchDescription([spawn_world, map_server] + actions)