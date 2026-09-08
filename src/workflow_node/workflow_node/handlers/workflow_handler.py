import json
import socket
import subprocess
import time
import traceback
from threading import Thread

import numpy as np
from std_msgs.msg import String

from rclpy.node import Node

from workflow_node.communication.mqtt_client import mqttClient
from workflow_node.communication.ros_interface import SafetyPublisher
from workflow_node.communication.qos_profiles import best_effort_qos
from workflow_node.config.configs import WorkflowNodeConfig
from workflow_node.handlers.action_handler import ActionHandler
from workflow_node.handlers.movement_handler import MovementHandler
from workflow_node.handlers.mqtt_handler import MqttHandler
from workflow_node.handlers.reroute_handler import RerouteHandler
from workflow_node.states import runtime_state as rs_module
from workflow_node.states.runtime_state import RuntimeState
from workflow_node.utils.db_loader import fetch_location_data_from_db
from workflow_node.utils.graph_loader import extract_waypoints, load_graphml
from workflow_node.utils.math_utils import quaternion_to_euler, rotated_rectangle_mask


class WorkflowHandler(Node):
    """Main ROS2 node for AGV workflow orchestration.

    Owns all ROS publishers, subscribers, and timers.  Delegates movement,
    action, rerouting, and MQTT logic to dedicated handler classes.
    """

    def __init__(
        self,
        movement: str,
        action: str,
        cfg: WorkflowNodeConfig,
    ) -> None:
        super().__init__('workflow_handler')

        self.cfg = cfg
        self.movement = movement
        self.action = action
        self.state = RuntimeState()

        # -----------------------------------------------------------------
        # ROS publishers
        # -----------------------------------------------------------------
        try:
            safety_pub_node = SafetyPublisher()
            self.publisher_ = safety_pub_node.publisher_  # /byd/safety
        except Exception as error:
            self.get_logger().error(
                f"Failed to create SafetyPublisher: {error}"
            )
            traceback.print_exc()
            raise

        self.task_request_pub = self.create_publisher(String, cfg.ros_topics.task_request, 10)
        self.path_publisher = self.create_publisher(String, cfg.ros_topics.path, 10)
        self.holded_nodes_publisher = self.create_publisher(
            String, cfg.ros_topics.holded_nodes, 10
        )
        self.charging_state_pub = self.create_publisher(
            String, cfg.ros_topics.charging_state, 10
        )
        self.safety_turnoff_publisher = self.create_publisher(
            String, cfg.ros_topics.safety_turnoff, 10
        )

        # -----------------------------------------------------------------
        # ROS subscribers
        # -----------------------------------------------------------------
        pose_qos = best_effort_qos()
        self.current_pose_subscriber = self.create_subscription(
            __import__('geometry_msgs.msg', fromlist=['PoseStamped']).PoseStamped,
            cfg.ros_topics.current_pose,
            self.current_pose_callback,
            pose_qos,
        )
        self.current_pose = None

        self.nmpc_pp_status_sub = self.create_subscription(
            String,
            cfg.ros_topics.pp_status,
            self.handle_pp_status,
            10,
        )
        self.nmpc_pp_status = None

        self.conflict_action_sub = self.create_subscription(
            String,
            cfg.ros_topics.conflict_action,
            self._conflict_action_callback,
            10,
        )

        # -----------------------------------------------------------------
        # Timers
        # -----------------------------------------------------------------
        self.holded_nodes_timer = self.create_timer(
            cfg.timers.holded_nodes_period, self.publish_holded_nodes
        )

        # -----------------------------------------------------------------
        # MQTT
        # -----------------------------------------------------------------
        try:
            self.mqtt_node = mqttClient(cfg.mqtt.broker_ip, cfg.mqtt.port)
            self.mqtt_node._client_host_ip = cfg.mqtt.robot_ip
            self.mqtt_node.loop_start()
        except Exception as error:
            self.get_logger().error(
                f"MQTT client setup failed "
                f"(broker={cfg.mqtt.broker_ip}:{cfg.mqtt.port}): {error}"
            )
            traceback.print_exc()
            raise

        # -----------------------------------------------------------------
        # Location data from DB
        # -----------------------------------------------------------------
        try:
            (
                self.left_easy_dock_dict,
                self.right_easy_dock_dict,
                self.dock_location_dict,
                self.dock_station_end_line_dict,
            ) = fetch_location_data_from_db(cfg.paths.wn_db_file)
            self.get_logger().info(f"dock_location_dict: {self.dock_location_dict}")

            self.location_id = self.movement
            self.get_logger().info(f"location_id: {self.location_id}")

            self.left_easy_dock = self.left_easy_dock_dict[self.location_id]
            self.right_easy_dock = self.right_easy_dock_dict[self.location_id]
            self.dock_location = self.dock_location_dict[self.location_id]
            self.dock_station_end_line = self.dock_station_end_line_dict[self.location_id]

            parking_id = cfg.locations.parking_location_id
            self.parking_location = self.dock_station_end_line_dict[parking_id]
            self.parking_dock_location = self.dock_location_dict[parking_id]

            self.get_logger().debug(
                f"left_easy_dock={self.left_easy_dock}, "
                f"right_easy_dock={self.right_easy_dock}, "
                f"dock_location={self.dock_location}, "
                f"dock_station_end_line={self.dock_station_end_line}"
            )
        except KeyError as error:
            self.get_logger().error(
                f"Location ID '{self.movement}' not found in DB: {error}. "
                "Check that the movement argument matches a valid location."
            )
            traceback.print_exc()
            raise
        except Exception as error:
            self.get_logger().error(
                f"Failed to load location data from DB '{cfg.paths.wn_db_file}': {error}"
            )
            traceback.print_exc()
            raise

        # -----------------------------------------------------------------
        # Graph
        # -----------------------------------------------------------------
        try:
            self.G_loaded = load_graphml(cfg.paths.graphml_file)
            self.waypoints, self.waypoints_positions = extract_waypoints(self.G_loaded)
        except Exception as error:
            self.get_logger().error(
                f"Failed to load graph from '{cfg.paths.graphml_file}': {error}"
            )
            traceback.print_exc()
            raise

        # -----------------------------------------------------------------
        # Delegate handlers
        # -----------------------------------------------------------------
        self.reroute_handler = RerouteHandler(self)
        self.movement_handler = MovementHandler(self)
        self.action_handler = ActionHandler(self)
        self.mqtt_handler = MqttHandler(self.mqtt_node)

        self.thread1: Thread = None
        self.result1 = None

        self.get_logger().info("WorkflowHandler initialisation complete.")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def current_pose_callback(self, msg) -> None:
        try:
            self.current_pose = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w,
            )
        except Exception as error:
            self.get_logger().error(f"current_pose_callback failed: {error}")
            traceback.print_exc()

    def handle_pp_status(self, msg: String) -> None:
        self.nmpc_pp_status = msg.data

    def _conflict_action_callback(self, msg: String) -> None:
        try:
            self.reroute_handler.handle_conflict_action(msg)
        except Exception as error:
            self.get_logger().error(f"_conflict_action_callback failed: {error}")
            traceback.print_exc()

    def publish_holded_nodes(self) -> None:
        """Timer callback: publish waypoints inside the robot bounding box."""
        if rs_module.cancel_status or rs_module.stop_event.is_set():
            return
        current_pose = self.current_pose
        if current_pose is None:
            return

        try:
            x = self.waypoints_positions[:, 0]
            y = self.waypoints_positions[:, 1]
            bb = self.cfg.robot.bounding_box
            *_, theta = quaternion_to_euler(current_pose[3], 0, 0, current_pose[2])
            mask = rotated_rectangle_mask(
                x, y,
                current_pose[0], current_pose[1],
                (bb.left, bb.right),
                (bb.front, bb.back),
                theta + np.pi,
            )
            matching_nodes = self.waypoints[mask]
            self.holded_nodes_publisher.publish(
                String(data=json.dumps({'holded_nodes': matching_nodes.tolist()}))
            )
        except Exception as error:
            self.get_logger().error(f"publish_holded_nodes failed: {error}")
            traceback.print_exc()

    # ------------------------------------------------------------------
    # Main workflow entry point
    # ------------------------------------------------------------------

    def operation(self) -> bool:
        """Execute the full workflow: movement → action."""
        try:
            self.mqtt_node.publish2topic("machine/task/status", "Task started")
        except Exception as error:
            self.get_logger().error(f"Failed to publish task start: {error}")
            traceback.print_exc()

        try:
            self.thread1 = Thread(target=self.normal_field)
            self.thread1.start()
        except Exception as error:
            self.get_logger().error(f"Failed to start normal_field thread: {error}")
            traceback.print_exc()

        self.get_logger().info(f"operation: movement={self.movement}, action={self.action}")
        try:
            self.get_logger().info(f"PAP before movement: {self._get_pap_status_once()}")
        except RuntimeError as error:
            self.get_logger().warning(f"Could not read PAP status before movement: {error}")

        try:
            if self.movement_handler.is_movement_needed():
                self.movement_handler.movement_operation(self.movement)
        except Exception as error:
            self.get_logger().error(f"movement_operation failed: {error}")
            traceback.print_exc()
            return False

        self.get_logger().info("Movement complete.")

        # try:
        #     self.action_handler.action_operation(self.action)
        # except Exception as error:
        #     self.get_logger().error(f"action_operation failed: {error}")
        #     traceback.print_exc()
        #     return False

        # self.get_logger().info("Action complete.")

        if self.state.error_status is None:
            try:
                self.task_request_pub.publish(
                    String(data=f"Task Completed:{self.state.task_id}")
                )
            except Exception as error:
                self.get_logger().error(f"Failed to publish task completion: {error}")
                traceback.print_exc()

        return True

    # ------------------------------------------------------------------
    # Safety field helpers
    # ------------------------------------------------------------------

    def normal_field(self) -> None:
        msg = String()
        msg.data = 'normal'
        try:
            for _ in range(101):
                self.publisher_.publish(msg)
        except Exception as error:
            self.get_logger().error(f"normal_field publish failed: {error}")
            traceback.print_exc()

    def pickdrop_field(self) -> None:
        msg = String()
        msg.data = 'pickdrop'
        try:
            for _ in range(101):
                self.publisher_.publish(msg)
        except Exception as error:
            self.get_logger().error(f"pickdrop_field publish failed: {error}")
            traceback.print_exc()

    # ------------------------------------------------------------------
    # Charging / CAN
    # ------------------------------------------------------------------

    def switch_off_contactor_and_reset_charging_state(self) -> None:
        try:
            subprocess.call("cansend can0 209#0000000000000000", shell=True)
        except Exception as error:
            self.get_logger().error(f"cansend failed: {error}")
            traceback.print_exc()
        try:
            msg = String()
            msg.data = "off"
            self.charging_state_pub.publish(msg)
        except Exception as error:
            self.get_logger().error(f"charging_state publish failed: {error}")
            traceback.print_exc()

    # ------------------------------------------------------------------
    # TCP socket (legacy — kept for operational parity)
    # ------------------------------------------------------------------

    def send_data(self, message: bytes, retries: int = 1, delay: int = 3) -> str:
        ip = self.cfg.tcp.host_ip
        port = self.cfg.tcp.port
        for attempt in range(retries):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((ip, port))
                    s.sendall(message)
                    data = s.recv(1024).decode()
                    self.get_logger().debug(f"TCP received: {data}")
                    return data
            except socket.error as e:
                self.get_logger().warning(f"TCP attempt {attempt + 1} failed: {e}")
                time.sleep(delay)
        return "Failed to connect after several attempts."

    # ------------------------------------------------------------------
    # Internal convenience
    # ------------------------------------------------------------------

    def _get_pap_status_once(self) -> bool:
        from workflow_node.states.pap_status import get_pap_status
        return get_pap_status()
