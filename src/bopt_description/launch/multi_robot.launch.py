from launch import LaunchDescription
from launch.actions import (
    GroupAction,
    TimerAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from nav2_common.launch import RewrittenYaml

import os


def spawn_robot_with_params(
    context,
    mqtt_bridge_params_file,
    amcl_params_file,
    robot_name,
    x,
    y,
    yaw,
    ip,
):

    # ============================================================
    # INITIAL POSE
    # ============================================================

    px, py, pyaw = x, y, yaw

    if robot_name == "robot001":
        px, py, pyaw = 6.89593, 10.5518, -0.0114491

    elif robot_name == "robot002":
        px, py, pyaw = 6.87761, 5.47154, -0.0114491

    # ============================================================
    # AMCL
    # ============================================================

    rewritten_amcl = RewrittenYaml(
        source_file=amcl_params_file,
        root_key=robot_name,
        param_rewrites={
            "amcl.ros__parameters.initial_pose.x": str(px),
            "amcl.ros__parameters.initial_pose.y": str(py),
            "amcl.ros__parameters.initial_pose.yaw": str(pyaw),

            "amcl.ros__parameters.set_initial_pose": "True",

            "amcl.ros__parameters.base_frame_id":
                f"{robot_name}/base_footprint",

            "amcl.ros__parameters.odom_frame_id":
                f"{robot_name}/odom",

            "amcl.ros__parameters.scan_topic":
                f"/{robot_name}/lidar/top3dl/scan",
        },
        convert_types=True,
    )

    # ============================================================
    # MQTT
    # ============================================================

    rewritten_mqtt = RewrittenYaml(
        source_file=mqtt_bridge_params_file,
        param_rewrites={
            "ros2_mqtt_bridge.mqtt_bridge.robot_id":
                robot_name,

            "ros2_mqtt_bridge.mqtt_bridge.robot_ip":
                ip,

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.path":
                "path",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.holded_nodes":
                "holded_nodes",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.current_pose":
                "current_pose",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.odometry":
                "odom",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.remaining_path":
                "remaining_path",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.task_request":
                "task_request",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.add_nodes":
                "add_nodes",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.local_path_active":
                "local_path_active",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.task":
                "task",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.conflict_action":
                "conflict_action",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.task_action":
                "task_action",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.safety_status":
                "safety_status",

            "ros2_mqtt_bridge.mqtt_bridge.ros2_topics.robot_dimensions":
                "robot_dimensions",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.current_pose":
                f"{robot_name}/current_pose",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.holded_nodes":
                f"{robot_name}/holded_nodes",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.path":
                f"{robot_name}/path",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.safety_status":
                f"{robot_name}/safety_status",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.odometry":
                f"{robot_name}/odometry/filtered",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.remaining_path":
                f"{robot_name}/remaining_path",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.task_request":
                f"{robot_name}/task_request",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.add_nodes":
                f"{robot_name}/add_nodes",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.local_path_active":
                f"{robot_name}/local_path_active",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.task":
                f"{robot_name}/task",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.conflict_action":
                f"{robot_name}/conflict_action",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.task_action":
                f"{robot_name}/task_action",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.safety_status":
                f"{robot_name}/safety_status",

            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.dimensions":
                f"{robot_name}/robot_dimensions",
        },
        convert_types=True,
    )

    amcl_yaml_path = rewritten_amcl.perform(context)
    mqtt_yaml_path = rewritten_mqtt.perform(context)

    # ============================================================
    # ROBOT DESCRIPTION
    #
    # IMPORTANT:
    # We generate the Xacro here directly.
    # No spawn_robot.launch.py is included.
    # ============================================================

    bopt_description_dir = get_package_share_directory(
        "bopt_description"
    )

    model_path = os.path.join(
        bopt_description_dir,
        "urdf",
        "robot.urdf.xacro",
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            model_path,
            " robot_name:=",
            robot_name,
            " robot_namespace:=",
            robot_name,
            " lidar_frame:=",
            f"{robot_name}/top3dl_link",
        ]),
        value_type=str,
    )

    # ============================================================
    # ROBOT STATE PUBLISHER
    #
    # This is deliberately NOT inside PushRosNamespace.
    # We explicitly give it robot_name.
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        name="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                "frame_prefix": f"{robot_name}/",

            }
        ],
        output="screen",
    )

    # ============================================================
    # GAZEBO SPAWN
    # ============================================================

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic",
            f"/{robot_name}/robot_description",

            "-name",
            robot_name,

            "-x",
            str(x),

            "-y",
            str(y),

            "-Y",
            str(yaw),
        ],
    )

    # ============================================================
    # RETURN
    # ============================================================

    return [

        # --------------------------------------------------------
        # SPAWN ROBOT DIRECTLY
        # --------------------------------------------------------

        robot_state_publisher,
        spawn_entity,

        # --------------------------------------------------------
        # EVERYTHING ELSE UNDER ROBOT NAMESPACE
        # --------------------------------------------------------

        GroupAction(
            [

                PushRosNamespace(robot_name),
                # ==================================================
                # LIDAR BRIDGE
                # ==================================================

                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name="lidar_bridge",
                    output="screen",
                    arguments=[
                        f"/{robot_name}/lidar/top3dl/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/lidar/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/lidar/back/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/lidar/left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/lidar/right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/lidar/front/left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/lidar/front/right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                        f"/{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    ],
                ),


                # ==================================================
                # BOPT CONTROLLER
                # ==================================================

                Node(
                    package="bopt_controller",
                    executable="bopt_controller",
                    name="bopt_controller",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                        }
                    ],
                ),

                # ==================================================
                # ODOMETRY
                # ==================================================

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

                # ==================================================
                # ROS2 CONTROL
                # ==================================================

                TimerAction(
                    period=5.0,
                    actions=[

                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "joint_state_broadcaster",
                                "--controller-manager",
                                f"/{robot_name}/controller_manager",
                                "--controller-manager-timeout",
                                "30",
                            ],
                            output="screen",
                        ),

                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "traction_joint_controller",
                                "--controller-manager",
                                f"/{robot_name}/controller_manager",
                                "--controller-manager-timeout",
                                "30",
                            ],
                            output="screen",
                        ),

                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "steering_joint_controller",
                                "--controller-manager",
                                f"/{robot_name}/controller_manager",
                                "--controller-manager-timeout",
                                "30",
                            ],
                            output="screen",
                        ),

                        Node(
                            package="controller_manager",
                            executable="spawner",
                            arguments=[
                                "lift_joint_controller",
                                "--controller-manager",
                                f"/{robot_name}/controller_manager",
                                "--controller-manager-timeout",
                                "30",
                            ],
                            output="screen",
                        ),
                    ],
                ),

                # ==================================================
                # LOCALIZATION
                # ==================================================

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory(
                                "bopt_localization"
                            ),
                            "launch",
                            "robot_localize.launch.py",
                        )
                    ),
                    launch_arguments={
                        "robot_name": robot_name,
                        "amcl_config": amcl_yaml_path,
                    }.items(),
                ),

                # ==================================================
                # MQTT
                # ==================================================

                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory(
                                "ros2_mqtt_bridge"
                            ),
                            "launch",
                            "mqtt_bridge.launch.py",
                        )
                    ),
                    launch_arguments={
                        "config_file": mqtt_yaml_path,
                    }.items(),
                ),
            ]
        ),
    ]


def generate_launch_description():

    # ============================================================
    # SHARED GAZEBO
    # ============================================================

    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bopt_description"),
                "launch",
                "gazebo_model.launch.py",
            )
        )
    )

    # ============================================================
    # SHARED MAP
    # ============================================================

    map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("bopt_localization"),
                "launch",
                "map.launch.py",
            )
        )
    )

    # ============================================================
    # CONFIG FILES
    # ============================================================

    amcl_params_file = os.path.join(
        get_package_share_directory("bopt_localization"),
        "config",
        "amcl.yaml",
    )

    mqtt_bridge_params_file = os.path.join(
        get_package_share_directory("ros2_mqtt_bridge"),
        "config",
        "properties.yaml",
    )

    # ============================================================
    # NUMBER OF ROBOTS
    #
    # CHANGE ONLY THIS
    # ============================================================

    num_robots = 2

    actions = []

    # ============================================================
    # GENERATE ROBOTS
    # ============================================================

    for i in range(num_robots):

        robot_name = f"robot{i + 1:03d}"

        # --------------------------------------------------------
        # POSITION
        # --------------------------------------------------------

        x = 0.0
        y = i * -5.0
        yaw = 0.0

        # --------------------------------------------------------
        # STARTUP DELAY
        # --------------------------------------------------------

        delay = i * 15.0

        # --------------------------------------------------------
        # IP
        # --------------------------------------------------------

        ip_alias = f"127.0.0.{i + 2}"

        print(
            f"\n[DEBUG] {robot_name} "
            f"IP={ip_alias} "
            f"POS=({x}, {y}, {yaw})\n"
        )

        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    OpaqueFunction(
                        function=spawn_robot_with_params,
                        args=[
                            mqtt_bridge_params_file,
                            amcl_params_file,
                            robot_name,
                            x,
                            y,
                            yaw,
                            ip_alias,
                        ],
                    )
                ],
            )
        )

    # ============================================================
    # FINAL
    # ============================================================

    return LaunchDescription(
        [
            gazebo_world,
            map_server,
        ] + actions
    )