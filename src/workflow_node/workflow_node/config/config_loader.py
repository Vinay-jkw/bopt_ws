import os
import traceback
from pathlib import Path

import yaml
from rclpy.logging import get_logger

from workflow_node.config.configs import (
    BoundingBoxConfig,
    LocationsConfig,
    MqttConfig,
    PathsConfig,
    PlannerConfig,
    PoseCorrectionConfig,
    RobotConfig,
    RosTopicsConfig,
    SubprocessesConfig,
    TcpConfig,
    ThresholdsConfig,
    TimersConfig,
    WorkflowNodeConfig,
)

_logger = get_logger('config_loader')


def load_config(yaml_path: str) -> WorkflowNodeConfig:
    """Parse a workflow node YAML file and return a typed config object.

    Raises:
        FileNotFoundError: if the YAML file does not exist.
        yaml.YAMLError: if the file is not valid YAML.
        KeyError: if a required section or field is missing.
        TypeError: if a field value cannot be converted to the expected type.
    """
    yaml_path = str(yaml_path)
    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(
            f"Workflow node config file not found: '{yaml_path}'. "
            "Check the --config argument or the package installation."
        )

    _logger.info(f"Loading config from: {yaml_path}")
    try:
        with open(yaml_path, 'r') as fh:
            raw = yaml.safe_load(fh)
    except yaml.YAMLError as error:
        raise yaml.YAMLError(
            f"Failed to parse YAML config '{yaml_path}': {error}"
        ) from error

    if raw is None:
        raise ValueError(f"Config file is empty: '{yaml_path}'")

    def _require(section: dict, key: str, section_name: str):
        if key not in section:
            raise KeyError(
                f"Required config field '{key}' missing from section "
                f"[{section_name}] in '{yaml_path}'"
            )
        return section[key]

    def _require_section(raw: dict, name: str) -> dict:
        if name not in raw:
            raise KeyError(
                f"Required config section '[{name}]' missing in '{yaml_path}'"
            )
        return raw[name]

    try:
        mqtt_raw = _require_section(raw, 'mqtt')
        tcp_raw = _require_section(raw, 'tcp')
        paths_raw = _require_section(raw, 'paths')
        robot_raw = _require_section(raw, 'robot')
        planner_raw = _require_section(raw, 'planner')
        thresh_raw = _require_section(raw, 'thresholds')
        topics_raw = _require_section(raw, 'ros_topics')
        timers_raw = _require_section(raw, 'timers')
        subproc_raw = _require_section(raw, 'subprocesses')
        pose_corr_raw = _require_section(raw, 'pose_correction')
        locations_raw = _require_section(raw, 'locations')
        bbox_raw = _require_section(robot_raw, 'bounding_box')

        cfg = WorkflowNodeConfig(
            mqtt=MqttConfig(
                broker_ip=str(_require(mqtt_raw, 'broker_ip', 'mqtt')),
                robot_ip=str(_require(mqtt_raw, 'robot_ip', 'mqtt')),
                port=int(_require(mqtt_raw, 'port', 'mqtt')),
            ),
            tcp=TcpConfig(
                host_ip=str(_require(tcp_raw, 'host_ip', 'tcp')),
                port=int(_require(tcp_raw, 'port', 'tcp')),
                retries=int(tcp_raw.get('retries', 1)),
                delay=int(tcp_raw.get('delay', 3)),
            ),
            paths=PathsConfig(
                graphml_file=str(_require(paths_raw, 'graphml_file', 'paths')),
                wn_db_file=str(_require(paths_raw, 'wn_db_file', 'paths')),
                rs_path_file=str(_require(paths_raw, 'rs_path_file', 'paths')),
            ),
            robot=RobotConfig(
                bounding_box=BoundingBoxConfig(
                    left=float(_require(bbox_raw, 'left', 'robot.bounding_box')),
                    right=float(_require(bbox_raw, 'right', 'robot.bounding_box')),
                    front=float(_require(bbox_raw, 'front', 'robot.bounding_box')),
                    back=float(_require(bbox_raw, 'back', 'robot.bounding_box')),
                )
            ),
            planner=PlannerConfig(
                turn_radius=float(_require(planner_raw, 'turn_radius', 'planner')),
                step_size=float(_require(planner_raw, 'step_size', 'planner')),
                spline_spacing=float(_require(planner_raw, 'spline_spacing', 'planner')),
                backup_distance_slow=float(_require(planner_raw, 'backup_distance_slow', 'planner')),
                backup_distance_pp=float(_require(planner_raw, 'backup_distance_pp', 'planner')),
                pallet_length=float(_require(planner_raw, 'pallet_length', 'planner')),
                pallet_half=float(_require(planner_raw, 'pallet_half', 'planner')),
                load_wheel_base_link_to_lidar=float(_require(planner_raw, 'load_wheel_base_link_to_lidar', 'planner')),
                pp_insert_enabled=bool(_require(planner_raw, 'pp_insert_enabled', 'planner')),
                scale=float(planner_raw.get('scale', 1.0)),
            ),
            thresholds=ThresholdsConfig(
                movement_needed_gap=float(_require(thresh_raw, 'movement_needed_gap', 'thresholds')),
                near_location_radius=float(_require(thresh_raw, 'near_location_radius', 'thresholds')),
                parking_near_radius=float(_require(thresh_raw, 'parking_near_radius', 'thresholds')),
                apds_dx_threshold=float(_require(thresh_raw, 'apds_dx_threshold', 'thresholds')),
                point1_filter_radius=float(_require(thresh_raw, 'point1_filter_radius', 'thresholds')),
                point2_filter_radius=float(_require(thresh_raw, 'point2_filter_radius', 'thresholds')),
                parking_max_speed=float(_require(thresh_raw, 'parking_max_speed', 'thresholds')),
            ),
            ros_topics=RosTopicsConfig(
                current_pose=str(_require(topics_raw, 'current_pose', 'ros_topics')),
                pp_status=str(_require(topics_raw, 'pp_status', 'ros_topics')),
                path=str(_require(topics_raw, 'path', 'ros_topics')),
                holded_nodes=str(_require(topics_raw, 'holded_nodes', 'ros_topics')),
                conflict_action=str(_require(topics_raw, 'conflict_action', 'ros_topics')),
                task_request=str(_require(topics_raw, 'task_request', 'ros_topics')),
                safety=str(_require(topics_raw, 'safety', 'ros_topics')),
                cmd_vel=str(_require(topics_raw, 'cmd_vel', 'ros_topics')),
                current_task=str(_require(topics_raw, 'current_task', 'ros_topics')),
                status=str(_require(topics_raw, 'status', 'ros_topics')),
                pap_field_status=str(_require(topics_raw, 'pap_field_status', 'ros_topics')),
                can_odot_data=str(_require(topics_raw, 'can_odot_data', 'ros_topics')),
                charging_state=str(_require(topics_raw, 'charging_state', 'ros_topics')),
                safety_turnoff=str(_require(topics_raw, 'safety_turnoff', 'ros_topics')),
                state=str(_require(topics_raw, 'state', 'ros_topics')),
                velocity=str(_require(topics_raw, 'velocity', 'ros_topics')),
                steering_angle=str(_require(topics_raw, 'steering_angle', 'ros_topics')),
            ),
            timers=TimersConfig(
                holded_nodes_period=float(
                    _require(timers_raw, 'holded_nodes_period', 'timers')
                ),
            ),
            subprocesses=SubprocessesConfig(
                pose_correction=list(_require(subproc_raw, 'pose_correction', 'subprocesses')),
                nmpc_controller=list(_require(subproc_raw, 'nmpc_controller', 'subprocesses')),
                nmpc_pp_slow=list(_require(subproc_raw, 'nmpc_pp_slow', 'subprocesses')),
                nmpc_pp=list(_require(subproc_raw, 'nmpc_pp', 'subprocesses')),
                lidar_clustering=list(_require(subproc_raw, 'lidar_clustering', 'subprocesses')),
                ppts=list(_require(subproc_raw, 'ppts', 'subprocesses')),
                pallet_detection=list(_require(subproc_raw, 'pallet_detection', 'subprocesses')),
                pallet_detection_tag=list(_require(subproc_raw, 'pallet_detection_tag', 'subprocesses')),
                fork_service=str(_require(subproc_raw, 'fork_service', 'subprocesses')),
                fork_service_type=str(_require(subproc_raw, 'fork_service_type', 'subprocesses')),
            ),
            pose_correction=PoseCorrectionConfig(
                timeout=int(_require(pose_corr_raw, 'timeout', 'pose_correction')),
                max_retries=int(_require(pose_corr_raw, 'max_retries', 'pose_correction')),
                delay_between_retries=int(
                    _require(pose_corr_raw, 'delay_between_retries', 'pose_correction')
                ),
            ),
            locations=LocationsConfig(
                parking_location_id=str(_require(locations_raw, 'parking_location_id', 'locations')),
            ),
        )
        _logger.info("Config loaded successfully.")
        return cfg

    except (KeyError, TypeError, ValueError) as error:
        _logger.error(f"Config validation failed — {error}")
        traceback.print_exc()
        raise


def default_config_path() -> str:
    """Return the installed share-directory path to the short-fork YAML."""
    try:
        from ament_index_python.packages import get_package_share_directory
        path = os.path.join(
            get_package_share_directory('workflow_node'),
            'config',
            'workflow_node_short_fork.yaml',
        )
        return path
    except Exception as error:
        _logger.warning(f"Could not resolve default config path: {error}")
        traceback.print_exc()
        raise
