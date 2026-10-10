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
    ekf_params_file,
    robot_name,
    gz_x,
    gz_y,
    gz_yaw,
    map_x,
    map_y,
    map_yaw,
    ip,
):
    # ============================================================
    # INITIAL MAP / AMCL POSE
    #
    # Gazebo and RViz/map use different coordinate systems.
    # AMCL receives the measured map pose; Gazebo receives the
    # measured Gazebo pose separately.
    # ============================================================

    px = map_x
    py = map_y
    pyaw = map_yaw

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
            "amcl.ros__parameters.base_frame_id": f"{robot_name}/base_footprint",
            "amcl.ros__parameters.odom_frame_id": f"{robot_name}/odom",
            "amcl.ros__parameters.scan_topic": f"/{robot_name}/lidar/top3dl/scan",
        },
        convert_types=True,
    )

    rewritten_ekf = RewrittenYaml(
        source_file=ekf_params_file,
        root_key=robot_name,
        param_rewrites={
            "ekf_filter_node.ros__parameters.odom_frame": f"{robot_name}/odom",
            "ekf_filter_node.ros__parameters.base_link_frame": f"{robot_name}/base_footprint",
            "ekf_filter_node.ros__parameters.world_frame": f"{robot_name}/odom",
            "ekf_filter_node.ros__parameters.odom0": f"/{robot_name}/odom",
            "ekf_filter_node.ros__parameters.imu0": f"/{robot_name}/imu/corrected",
        },
        convert_types=True,
    )

    # ============================================================
    # MQTT
    # ============================================================

    rewritten_mqtt = RewrittenYaml(
        source_file=mqtt_bridge_params_file,
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
            "ros2_mqtt_bridge.mqtt_bridge.mqtt_topics.dimensions": f"{robot_name}/robot_dimensions",
        },
        convert_types=True,
    )

    amcl_yaml_path = rewritten_amcl.perform(context)
    ekf_yaml_path = rewritten_ekf.perform(context)
    mqtt_yaml_path = rewritten_mqtt.perform(context)

    # ============================================================
    # ROBOT DESCRIPTION
    #
    # IMPORTANT: We generate the Xacro here directly.
    # No spawn_robot.launch.py is included.
    # ============================================================

    bopt_description_dir = get_package_share_directory("bopt_description")

    model_path = os.path.join(bopt_description_dir, "urdf", "robot.urdf.xacro")

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
    # Deliberately NOT inside PushRosNamespace — namespace is set
    # explicitly via robot_name.
    # ============================================================

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_name,
        name="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
            "frame_prefix": f"{robot_name}/",
        }],
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
            "-topic", f"/{robot_name}/robot_description",
            "-name", robot_name,
            "-x", str(gz_x),
            "-y", str(gz_y),
            "-Y", str(gz_yaw),
        ],
    )

    return [
        # Spawn robot directly (outside namespace group)
        robot_state_publisher,
        spawn_entity,

        # Everything else under robot namespace
        GroupAction([
            PushRosNamespace(robot_name),

            # --------------------------------------------------
            # BOPT CONTROLLER
            # --------------------------------------------------

            Node(
                package="bopt_controller",
                executable="bopt_main_controller",
                name="bopt_main_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "wheel_radius": 0.115,
                    "max_wheel_velocity": 7.246,
                    "max_steering_angle": 1.5708,
                    "control_dt": 0.05,
                    "lift_min": 0.0,
                    "lift_max": 0.095,
                    "command_timeout": 0.5,
                    "steering_tolerance": 0.03,
                    "steering_delay": 0.05,
                    "wheel_acceleration": 3.5,
                    "wheel_deceleration": 8.0,
                    "steering_start_threshold": 0.15,
                }],
            ),

            Node(
                package="bopt_controller",
                executable="bopt_hydraulic_controller",
                name="bopt_hydraulic_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "lift_min": 0.0,
                    "lift_max": 0.095,
                }],
            ),

            Node(
                package="bopt_controller",
                executable="bopt_nmpc_controller",
                name="bopt_nmpc_controller",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "max_steering_angle": 1.5708,
                    "require_state_check": False,
                    "invert_steering": True,
                }],
            ),

            # --------------------------------------------------
            # SENSOR BRIDGE
            # --------------------------------------------------

            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="sensor_bridge",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    f"/{robot_name}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                    f"/{robot_name}/lidar/top3dl/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/{robot_name}/lidar/front/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    # f"/{robot_name}/camera/back/image@sensor_msgs/msg/Image[gz.msgs.Image",
                    # f"/{robot_name}/camera/back/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    # f"/{robot_name}/camera/back/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
                    # f"/{robot_name}/camera/back/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                    f"/{robot_name}/lidar/left/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/{robot_name}/lidar/right/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/{robot_name}/Lidar_LFT@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/{robot_name}/Lidar_RFT@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/{robot_name}/ultrasonic/left/range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    f"/{robot_name}/ultrasonic/right/range@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                ],
            ),

            # --------------------------------------------------
            # FORKTIP LIDAR DETECTION
            # --------------------------------------------------

            Node(
                package="forktip_lidar_detection",
                executable="forktip_lidar_node_pap",
                name="quadrilateral_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),

            Node(
                package="forktip_lidar_detection",
                executable="forktip_lidar_node_ds",
                name="drop_station_publisher",
                output="screen",
                parameters=[{"use_sim_time": True}],
            ),

            # --------------------------------------------------
            # BOPT KEY & RELAY FOR TELEOP
            # --------------------------------------------------

            Node(
                package="bopt_controller",
                executable="bopt_key",
                name="bopt_key_node",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "wheelbase": 1.542,
                    "max_steering_angle": 1.5708,
                }],
            ),

            Node(
                package="bopt_controller",
                executable="bopt_twist_relay",
                name="bopt_twist_relay",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "control_mode": "auto",
                }],
            ),

            Node(
                package="workflow_node",
                executable="limit_switch_publisher",
                name="limit_switch_publisher",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                }],
            ),

            # --------------------------------------------------
            # ODOMETRY
            # --------------------------------------------------

            Node(
                package="bopt_controller",
                executable="odometry_node",
                name="bopt_odometry",
                output="screen",
                parameters=[{
                    "use_sim_time": True,
                    "robot_name": robot_name,
                    "publish_tf": False,
                }],
            ),

            # --------------------------------------------------
            # EKF
            # --------------------------------------------------

            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    ekf_yaml_path,
                    {"use_sim_time": True},
                ],
            ),
            Node(
                package="bopt_localization",
                executable="imu_covariance_relay",
                name="imu_covariance_relay",
                output="screen",
                parameters=[{
                    "input_topic": f"/{robot_name}/imu",
                    "output_topic": f"/{robot_name}/imu/corrected",
                    "frame_id": f"{robot_name}/imu_link",
                }],
            ),

            # --------------------------------------------------
            # ROS2 CONTROL
            # --------------------------------------------------

            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "joint_state_broadcaster",
                            "--controller-manager", f"/{robot_name}/controller_manager",
                            "--controller-manager-timeout", "30",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "traction_joint_controller",
                            "--controller-manager", f"/{robot_name}/controller_manager",
                            "--controller-manager-timeout", "30",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "steering_joint_controller",
                            "--controller-manager", f"/{robot_name}/controller_manager",
                            "--controller-manager-timeout", "30",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            "lift_joint_controller",
                            "--controller-manager", f"/{robot_name}/controller_manager",
                            "--controller-manager-timeout", "30",
                        ],
                        output="screen",
                    ),
                ],
            ),

            # --------------------------------------------------
            # LOCALIZATION
            # --------------------------------------------------

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("bopt_localization"),
                        "launch",
                        "robot_localize.launch.py",
                    )
                ),
                launch_arguments={
                    "robot_name": robot_name,
                    "amcl_config": amcl_yaml_path,
                }.items(),
            ),

            # --------------------------------------------------
            # MQTT
            # --------------------------------------------------

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("ros2_mqtt_bridge"),
                        "launch",
                        "mqtt_bridge.launch.py",
                    )
                ),
                launch_arguments={
                    "config_file": mqtt_yaml_path,
                }.items(),
            ),
        ]),
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
        ),
        launch_arguments={
            "world_name": "fg_warehouse.world",
        }.items(),
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

    ekf_params_file = os.path.join(
        get_package_share_directory("bopt_localization"),
        "config",
        "ekf.yaml",
    )

    # ============================================================
    # NUMBER OF ROBOTS  —  CHANGE ONLY THIS
    # ============================================================

    num_robots = 2
    

    actions = []

    # ============================================================
    # PARKING POSITIONS
    #
    # Each entry contains:
    #   Gazebo pose -> used by ros_gz_sim/create
    #   Map pose    -> used by AMCL initial_pose
    #
    # These are the measured positions from the same simulation.
    # ============================================================

    parking_positions = [
        # Parking 1
        {
            "gz_x": 113.4196392146, "gz_y": 26.0720401557, "gz_yaw": 0.0,
            "map_x": 37.313,        "map_y": 47.475,        "map_yaw": 3.141592653589793,
        },
        # Parking 2
        {
            "gz_x": 113.4422792924, "gz_y": 24.3584991638, "gz_yaw": 0.0,
            "map_x": 37.323,        "map_y": 49.178,        "map_yaw": 3.141592653589793,
        },
        # Parking 3
        {
            "gz_x": 113.4412040673, "gz_y": 22.6844212333, "gz_yaw": 0.0,
            "map_x": 37.338,        "map_y": 50.878,        "map_yaw": 3.141592653589793,
        },
        # Parking 4
        {
            "gz_x": 113.4658507557, "gz_y": 20.9647966720, "gz_yaw": 0.0,
            "map_x": 37.336,        "map_y": 52.581,        "map_yaw": 3.141592653589793,
        },
    ]

    for i in range(num_robots):
        robot_name = f"robot{i + 1:03d}"

        parking = parking_positions[i]

        gz_x   = parking["gz_x"]
        gz_y   = parking["gz_y"]
        gz_yaw = parking["gz_yaw"]
        map_x   = parking["map_x"]
        map_y   = parking["map_y"]
        map_yaw = parking["map_yaw"]

        delay     = i * 10.0
        ip_alias  = f"127.0.0.{i + 2}"

        print(
            f"\n[DEBUG] {robot_name}\n"
            f"  Gazebo = ({gz_x}, {gz_y}, {gz_yaw})\n"
            f"  Map    = ({map_x}, {map_y}, {map_yaw})\n"
            f"  IP     = {ip_alias}\n"
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
                            ekf_params_file,
                            robot_name,
                            gz_x, gz_y, gz_yaw,   # Gazebo pose
                            map_x, map_y, map_yaw, # Map / AMCL pose
                            ip_alias,
                        ],
                    )
                ],
            )
        )

    # ============================================================
    # FINAL
    # ============================================================

    return LaunchDescription([gazebo_world, map_server] + actions)