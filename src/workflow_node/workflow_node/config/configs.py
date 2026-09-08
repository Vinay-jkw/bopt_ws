from dataclasses import dataclass, field
from typing import List


@dataclass
class MqttConfig:
    broker_ip: str
    robot_ip: str
    port: int = 1883


@dataclass
class TcpConfig:
    host_ip: str
    port: int
    retries: int = 1
    delay: int = 3


@dataclass
class PathsConfig:
    graphml_file: str
    wn_db_file: str
    rs_path_file: str


@dataclass
class BoundingBoxConfig:
    left: float
    right: float
    front: float
    back: float


@dataclass
class RobotConfig:
    bounding_box: BoundingBoxConfig


@dataclass
class PlannerConfig:
    turn_radius: float
    step_size: float
    spline_spacing: float
    backup_distance_slow: float
    backup_distance_pp: float
    pallet_length: float
    pallet_half: float
    load_wheel_base_link_to_lidar: float
    pp_insert_enabled: bool
    scale: float = 1.0


@dataclass
class ThresholdsConfig:
    movement_needed_gap: float
    near_location_radius: float
    parking_near_radius: float
    apds_dx_threshold: float
    point1_filter_radius: float
    point2_filter_radius: float
    parking_max_speed: float


@dataclass
class RosTopicsConfig:
    current_pose: str
    pp_status: str
    path: str
    holded_nodes: str
    conflict_action: str
    task_request: str
    safety: str
    cmd_vel: str
    current_task: str
    status: str
    pap_field_status: str
    can_odot_data: str
    charging_state: str
    safety_turnoff: str
    state: str
    velocity: str
    steering_angle: str


@dataclass
class TimersConfig:
    holded_nodes_period: float


@dataclass
class SubprocessesConfig:
    pose_correction: List[str]
    nmpc_controller: List[str]
    nmpc_pp_slow: List[str]
    nmpc_pp: List[str]
    lidar_clustering: List[str]
    ppts: List[str]
    pallet_detection: List[str]
    pallet_detection_tag: List[str]
    fork_service: str
    fork_service_type: str


@dataclass
class PoseCorrectionConfig:
    timeout: int
    max_retries: int
    delay_between_retries: int


@dataclass
class LocationsConfig:
    parking_location_id: str


@dataclass
class WorkflowNodeConfig:
    mqtt: MqttConfig
    tcp: TcpConfig
    paths: PathsConfig
    robot: RobotConfig
    planner: PlannerConfig
    thresholds: ThresholdsConfig
    ros_topics: RosTopicsConfig
    timers: TimersConfig
    subprocesses: SubprocessesConfig
    pose_correction: PoseCorrectionConfig
    locations: LocationsConfig
