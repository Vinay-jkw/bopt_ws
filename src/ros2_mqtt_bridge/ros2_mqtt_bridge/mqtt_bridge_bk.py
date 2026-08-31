import argparse
import signal

import yaml
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import paho.mqtt.client as mqtt
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, QoSDurabilityPolicy
import json
import time
import os
import subprocess
import ast
import traceback

# # MQTT Configuration
# MQTT_BROKER = os.getenv('MQTT_BROKER')
# MQTT_PORT = 1883
# RECONNECT_DELAY = 5  
# ROBOT_ID = os.getenv('ROBOT_ID')
# ROBOT_IP = os.getenv('ROBOT_IP')

class RosMqttBridge(Node):
    def __init__(self, params):
        self.task_id=None
        super().__init__('ros_mqtt_bridge')

        # Define QoS profiles
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        qos_profile_confact = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        qos_profile1 = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.task_process = None
        self.mqtt_broker = params['mqtt_broker']
        self.mqtt_port = params['mqtt_port']
        self.reconnect_delay = params['reconnect_delay']
        self.robot_id = params['robot_id']
        self.robot_ip = params['robot_ip']
        self.robot_length = params['robot_length']
        self.robot_width = params['robot_width']
        self.ros2_topics = params['ros2_topics']
        self.mqtt_topics = params['mqtt_topics']
        self.publish_dimensions_period = params['publish_dimensions_period']
        self.path_list_file = params['path_list_file']
        print("Robot ID:", self.robot_id)
        print("Robot IP:", self.robot_ip)
        

        # Declare ROS parameters for dynamic robot dimensions
        self.declare_parameter("robot_length", self.robot_length)
        self.declare_parameter("robot_width", self.robot_width)
        self.in_lane = False
        self.lane_id = None
        self.on_buffer = False
        self.buffer_id = None
        self.terminate = False
        self.remaining_path = []
        # ROS 2 Subscribers
        self.create_subscription(PoseStamped, self.ros2_topics['current_pose'], self.pose_callback, qos_profile1)
        self.create_subscription(Odometry, self.ros2_topics['odometry'], self.odometry_callback, qos_profile)
        # self.create_subscription(String, self.ros2_topics['remaining_path'], self.remaining_path_callback, qos_profile)
        self.create_subscription(String, self.ros2_topics['task_request'], self.task_request_callback, qos_profile)
        self.create_subscription(String, self.ros2_topics['add_nodes'], self.add_node_callback, qos_profile)
        self.create_subscription(Bool, self.ros2_topics['local_path_active'], self.local_path_active_callback, qos_profile)
        self.create_subscription(
            String,
            "/path",
            lambda msg: self.mqtt_client.publish('FBPT011/path', msg.data, qos=2), 
            qos_profile
        )
        self.create_subscription(
            String,
            "/holded_nodes",
            lambda msg: self.mqtt_client.publish('FBPT011/holded_nodes', msg.data),
            qos_profile
        )
        self.create_subscription(
            String,
            "machine/error/status",
            self.machine_error_callback,
            10
        )
        self.lane_status_sub = self.create_subscription(
            String,
            f'{self.robot_id}/lane_status',
            self._lane_status_to_mqtt,
            10
        )
        # ROS 2 Publishers
        # self.safe_exit_pub = self.create_publisher(String, 'Veronica/safe_exit', qos_profile)
        self.task_pub = self.create_publisher(String, self.ros2_topics['task'], qos_profile)
        self.command_pub = self.create_publisher(String, self.ros2_topics['conflict_action'], qos_profile)
        self.task_action_pub = self.create_publisher(String, self.ros2_topics['task_action'], qos_profile)
        self.dimension_pub = self.create_publisher(String, self.ros2_topics['robot_dimensions'], qos_profile)
        self.velo_pub = self.create_publisher(Float64, '/velocity', qos_profile)
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', qos_profile)
        
        self.mqtt_status_pub = self.create_publisher(
            String,
            "/mqtt/status",
            QoSProfile(
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL  # 🔥 IMPORTANT
            )
        )

        # Timer to periodically publish dimensions (every 5 seconds)
        self.create_timer(self.publish_dimensions_period, self.dimensions_callback)
        self.lane_state_pub = self.create_publisher(String, f"/{self.robot_id}/lane_cmd_state", qos_profile)
        # self.create_timer(0.01, self.publish_lane_state)
        # Initialize MQTT Client with error handling
        self.mqtt_client = mqtt.Client()
        # self.mqtt_client.will_set("/mqtt/status", "disconnected", qos=1, retain=True)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_disconnect = self.on_disconnect
        self.connect_mqtt()
        self.mqtt_client.publish(self.mqtt_topics['action_response'], f"{self.robot_ip}/action_response:{self.robot_id}=('cancel_mna',{self.task_id})", qos=2)
        
        self.create_timer(1.0, self.publish_mqtt_status)
        
        # To track manager heartbeat
        self.last_manager_heartbeat = time.time()
        
        self.manager_alive = True
        self.HEARTBEAT_TIMEOUT = 1  # 500 ms
        self.create_timer(0.1, self.check_manager_heartbeat)  # 10 Hz

    def publish_mqtt_status(self):
        msg = String()

        if self.mqtt_client.is_connected() and self.manager_alive:
            msg.data = "connected"
        else:
            msg.data = "disconnected"

        self.mqtt_status_pub.publish(msg)
        
    def check_manager_heartbeat(self):
        alive = (time.time() - self.last_manager_heartbeat) < self.HEARTBEAT_TIMEOUT

        if not alive and self.manager_alive:
            self.manager_alive = False

            msg = String()
            msg.data = "disconnected"
            self.mqtt_status_pub.publish(msg)

            self.get_logger().error("Manager heartbeat LOST!")

        elif alive and not self.manager_alive:
            self.manager_alive = True

            msg = String()
            msg.data = "connected"
            self.mqtt_status_pub.publish(msg)

            self.get_logger().info("Manager heartbeat RESTORED")
        

    def connect_mqtt(self):
        """Attempt to connect to the MQTT broker with a limited retry mechanism."""
        while True:
            try:
                self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, keepalive=5)
                self.mqtt_client.loop_start()
                self.get_logger().info("Connected to MQTT Broker!")
                return
            except Exception as e:
                self.get_logger().warn(f"MQTT connection failed: {e}. Retrying in {self.reconnect_delay}s...")
                time.sleep(self.reconnect_delay)

        self.get_logger().error("Failed to connect to MQTT Broker after multiple attempts.")
       
    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("Connected to MQTT Broker!")

        msg = String()
        msg.data = "connected"
        self.mqtt_status_pub.publish(msg)

        client.subscribe(self.mqtt_topics['task'], qos=2)
        client.subscribe(self.mqtt_topics['conflict_action'], qos=2)
        client.subscribe(self.mqtt_topics['task_action'], qos=2)
        client.subscribe("manager/heartbeat", qos=1)

    def on_disconnect(self, client, userdata, rc):
        self.get_logger().warn("MQTT Disconnected!")

        msg = String()
        msg.data = "disconnected"
        self.mqtt_status_pub.publish(msg)
    
    def on_message(self, client, userdata, msg):
        """Handle incoming MQTT messages and forward them to ROS topics."""
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        ros_msg = String()
        ros_msg.data = payload
        try:
            if topic == "manager/heartbeat":
                data = json.loads(msg.payload.decode())
                if data.get("source") == "manager":
                    self.last_manager_heartbeat = time.time()
                    # self.get_logger().info("Received manager heartbeat.")
                    self.last_manager_heartbeat = time.time()
            if topic == f"{self.robot_id}/task":
                self.task_pub.publish(ros_msg)
                task_data = json.loads(ros_msg.data)
                
                command_type = task_data.get("command", "")
                # path_list = task_data.get("path", [])
                task_list = task_data.get("task", [])
                self.task_id=task_list[0]

                # try:
                #     with open(self.path_list_file, "w") as f:
                #         json.dump({"path": path_list}, f, indent=4)
                #     self.get_logger().info("Saved path_list to JSON file.")
                # except Exception as e:
                #     self.get_logger().error(f"Failed to write path_list to file: {e}")

                if command_type == "new_task" and isinstance(task_list, list):
                    self.get_logger().info(f"Received task: {task_list}")
                    command = ["ros2", "run", "workflow_node", "workflow_node"] + [str(i) for i in task_list]
                    self.get_logger().info(f"Executing command: {' '.join(command)}")
                    if self.task_process is not None and self.task_process.poll() is None:
                        os.killpg(os.getpgid(self.task_process.pid), signal.SIGINT)
                        # os.killpg(os.getpgid(self.task_process.pid), signal.SIGTERM)
                        self.task_process.kill()
                        self.task_process.wait()
                        self.velo_pub.publish(Float64(data=0.0))
                        self.steer_pub.publish(Float64(data=0.0))
                        self.task_process = None
                    self.task_process = subprocess.Popen(command, preexec_fn=os.setsid)
                else:
                    self.get_logger().warn("Invalid task format received.")

            elif topic == f"{self.robot_id}/conflict_action":
                # Directly publish the conflict action to the ROS topic without waiting for any acknowledgment.
                self.command_pub.publish(ros_msg)
                print(ros_msg)

            elif topic == f"{self.robot_id}/task_action":
                print("task_Action")
                self.task_action_pub.publish(ros_msg)
                data =payload.strip().split("/")[1]
                if data == "cancel_mna":
                    if self.task_process is not None and self.task_process.poll() is None:
                        print("Cancelling task process...")
                        os.killpg(os.getpgid(self.task_process.pid), signal.SIGINT)
                        # os.killpg(os.getpgid(self.task_process.pid), signal.SIGTERM)
                        self.task_process.kill()
                        self.task_process.wait()
                        self.velo_pub.publish(Float64(data=0.0))
                        self.steer_pub.publish(Float64(data=0.0))
                        self.task_process = None
                    else: subprocess.run(['pkill', '-SIGINT', 'workflow_node'])
                    self.remaining_path = []
                    self.publish_remaining_path_on_mqtt()
                    self.local_path_active_callback(Bool(data=False))
                    print("task_Action")
                    self.mqtt_client.publish(self.mqtt_topics['action_response'],f"{self.robot_ip}/action_response:{self.robot_id}=('cancel_mna',{self.task_id})", qos=2)
                    print("Movement operation Cancelled successfully.")
                    self.mqtt_client.publish(self.mqtt_topics['task_request'], f"Task Cancelled:{self.task_id}", qos=2)
                    self.task_id = None
                elif data == "pause_task":
                    if self.task_process is not None and self.task_process.poll() is None:
                        print("Cancelling task process...")
                        os.killpg(os.getpgid(self.task_process.pid), signal.SIGINT)
                        # os.killpg(os.getpgid(self.task_process.pid), signal.SIGTERM)
                        self.task_process.kill()
                        self.task_process.wait()
                        self.velo_pub.publish(Float64(data=0.0))
                        self.steer_pub.publish(Float64(data=0.0))
                        self.task_process = None
                    else: subprocess.run(['pkill', '-SIGINT', 'workflow_node'])
                    self.remaining_path = []
                    self.publish_remaining_path_on_mqtt()
                    self.local_path_active_callback(Bool(data=False))
                    print("task_Action")
                    self.mqtt_client.publish(self.mqtt_topics['action_response'],f"{self.robot_ip}/action_response:{self.robot_id}=('pause_task',{self.task_id})", qos=2)
                    print("Movement operation Paused successfully.")
                elif data.startswith("resume_task"):
                    action, rdata = data.split('=')
                    rdata = eval(rdata)
                    if not rdata: return
                    if self.task_id is None: 
                        self.task_id = int(rdata[0])
                    if self.task_id == rdata[0]:
                        if self.task_process is not None and self.task_process.poll() is None:
                            os.killpg(os.getpgid(self.task_process.pid), signal.SIGINT)
                            # os.killpg(os.getpgid(self.task_process.pid), signal.SIGTERM)
                            self.task_process.kill()
                            self.task_process.wait()
                            self.velo_pub.publish(Float64(data=0.0))
                            self.steer_pub.publish(Float64(data=0.0))
                            self.task_process = None
                        command = ["ros2", "run", "workflow_node", "workflow_node"] + [str(i) for i in rdata]
                        self.task_process = subprocess.Popen(command, preexec_fn=os.setsid)
                        self.mqtt_client.publish(self.mqtt_topics['action_response'],f"{self.robot_ip}/action_response:{self.robot_id}=('resume_task',{self.task_id})", qos=2)
            elif topic == f"{self.robot_id}/buffer_status":
                # Forward the raw JSON on to ROS 2 for your robot to pick up
                try:
                    data = json.loads(payload)
                    # If you also want to capture buffer‐fields, do so here:
                    self.on_buffer = data.get("value", False)
                    self.buffer_id = data.get("buffer_id", None)
                    self.terminate = data.get("terminate", None)
                except json.JSONDecodeError:
                    self.get_logger().error(f"Invalid JSON in buffer_status: {payload}")
                    # still forward the raw string so RobotClient can see it, if you like:
                    # self.buffer_status_pub.publish(String(data=payload))
            elif topic == f"{self.robot_id}/lane_cmd":
            # 1) Decode JSON and assign into class variables:
                try:
                    data = json.loads(payload)
                    # e.g. {"in_lane": true, "lane_id": "L1", "robot_id": "Veronica"}
                    self.in_lane = data.get("in_lane", False)
                    self.lane_id = data.get("lane_id", None)
                except json.JSONDecodeError:
                    self.get_logger().error(f"Invalid JSON in lane_cmd: {payload}")
                    # still forward the raw string so RobotClient can see it, if you like:
                    # 2) Immediately re‐publish to ROS “/Veronica/lane_cmd” so RobotClient sees it once:
                    # self.lane_cmd_pub.publish(String(data=payload))
                    print(f"[bridge] republished on ROS /{self.robot_id}/lane_cmd ▶ {payload}")
                return  # bail out after handling lane_cmd
        except Exception as e:
            self.get_logger().error(f"Exception occured in on_message: {topic}, {payload}, {e}")


    def machine_error_callback(self, msg:String):
        """
        Forward machine error from ROS2 TO MQTT FOR safety.
        """

        error_payload = f"{self.robot_ip}/{msg.data}"
        #Publish to same MQTT topic name
        self.mqtt_client.publish(
            "machine/error/detected",
            error_payload,
            qos=2
        )

        self.get_logger().info(
            f"Forwarded machine error to Mqtt : {error_payload}"
        )
    
    def _lane_status_to_mqtt(self, msg: String):
        """
        ANY time our RobotClient publishes on ROS “<ROBOT_ID>/lane_status” (i.e. a JSON‐string
        containing {"robot_id":…,"in_lane":…, "lane_id":…}), we forward it verbatim onto MQTT
        under the topic "<ROBOT_ID>/lane_status".
        """

        # msg.data is already JSON, so we just republish as‐is:
        lane_json = msg.data
        self.mqtt_client.publish(f"{self.robot_id}/lane_status", lane_json, qos=1)
        self.get_logger().debug(f"↔️ Forwarded lane_status → MQTT: {lane_json}")

    def local_path_active_callback(self, msg: Bool):
        """
        Callback for local path active messages from ROS. This forwards the local
        path active status to the MQTT broker under the topic "Veronica/local_path_active".
        """
        active = msg.data
        # Package the boolean status into JSON. You could also choose to send simply "true"/"false" as text.
        payload = json.dumps({"local_path_active": active})
        self.mqtt_client.publish(self.mqtt_topics['local_path_active'], payload)
        # self.get_logger().info(f"Forwarded local_path_active status: {active} to MQTT.")


    def pose_callback(self, msg):
        """Publish the current pose data to the MQTT broker."""
        pose_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "position": {
                "x": msg.pose.position.x,
                "y": msg.pose.position.y,
            },
            "orientation": {
                "z": msg.pose.orientation.z,
                "w": msg.pose.orientation.w
            }
        }
        json_payload = json.dumps(pose_data)
        self.mqtt_client.publish(self.mqtt_topics['current_pose'], json_payload)

    def task_request_callback(self, msg):
        """Forward ROS task request messages to the MQTT topic."""
        self.get_logger().info(f"Received ROS task request: {msg.data}. Forwarding to MQTT.")
        self.mqtt_client.publish(self.mqtt_topics['task_request'], msg.data, qos=2)
    
    def dimensions_callback(self):
        """Publish robot dimensions to both MQTT and ROS."""
        try:
            length = self.get_parameter("robot_length").value
            width = self.get_parameter("robot_width").value
            dimensions = {"length": length, "width": width}
            json_payload = json.dumps(dimensions)

            self.mqtt_client.publish(self.mqtt_topics['dimensions'], json_payload)
            self.dimension_pub.publish(String(data=json_payload))
        except Exception as e:
            self.get_logger().warn(f"Failed to get dimensions: {e}")

    def odometry_callback(self, msg):
        """Publish robot odometry data to the MQTT broker."""
        def clean_value(value):
            return 0.0 if abs(value) < 1e-10 else value

        odometry_data = {
            "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w
            },
            "velocity": {
                "linear_x": clean_value(msg.twist.twist.linear.x),
                "linear_y": clean_value(msg.twist.twist.linear.y),
                "angular_z": clean_value(msg.twist.twist.angular.z),
            },
            "pose_covariance": [clean_value(value) for value in msg.pose.covariance],
            "twist_covariance": [clean_value(value) for value in msg.twist.covariance]
        }

        json_payload = json.dumps(odometry_data)
        self.mqtt_client.publish(self.mqtt_topics['odometry'], json_payload)


    def remaining_path_callback(self, msg):
        """
        1. Convert the textual list in msg.data to a Python list/tuple structure.
        2. Store the result in self.remaining_path.
        3. Ask the publisher to send it.
        """
        raw = msg.data.strip()           # ensure no leading/trailing whitespace
        if not raw:                      # empty ⇒ []
            self.remaining_path = []
            self.publish_remaining_path_on_mqtt()
            return

        # Try the robust literal-eval route first
        # self.remaining_path = ast.literal_eval(f'[{raw}]')

        try:
            # Wrap in [] so "58, ('temp…'), 61" becomes "[58, ('temp…'), 61]"
            self.remaining_path = eval(f'[{raw}]')
        except (SyntaxError, ValueError):
            # Fallback: split plain numeric IDs that have commas but no tuples
            self.remaining_path = [
                part.strip() for part in raw.split(',') if part.strip()
            ]

        # Hand off to the publisher
        self.publish_remaining_path_on_mqtt()


    def publish_remaining_path_on_mqtt(self):
        """
        1. Collapse each element of self.remaining_path to a single string,
        e.g. ('temp_58_1', (17.94, 8.48)) → "('temp_58_1', (17.94, 8.48))"
        2. JSON-encode with a timestamp and publish.
        """
        if self.remaining_path == [""]:
            self.remaining_path = []

        remaining_path_strs = [str(item) for item in self.remaining_path]

        payload = json.dumps({
            "timestamp": time.time(),
            "remaining_path": remaining_path_strs
        })

        self.mqtt_client.publish(self.mqtt_topics['remaining_path'], payload)
            
    def add_node_callback(self, msg):
        """
        Callback for messages arriving on the ROS side for the add_nodes topic.
        These messages are forwarded to the MQTT topic Veronica/add_nodes.
        """
        # self.get_logger().info(f"Received add_nodes message on ROS: {msg.data}. Forwarding to MQTT.")
        self.mqtt_client.publish(self.mqtt_topics['add_nodes'], msg.data)

    def publish_lane_state(self):
        """
        Every 0.1 seconds this runs. It takes the latest
        self.in_lane / self.lane_id / self.on_buffer / self.buffer_id
        and emits them, as JSON, on "/Veronica/lane_cmd_state".
        """
        state = {
        "robot_id": self.robot_id,
        "in_lane": self.in_lane,
        "lane_id": self.lane_id,
        "on_buffer": self.on_buffer,
        "buffer_id": self.buffer_id,
        "terminate": self.terminate
        }
        ros_msg = String()
        ros_msg.data = json.dumps(state)
        self.lane_state_pub.publish(ros_msg)
        
        
def load_node_params(defaults, config_keys, config_file_arg):
    """
    Load and merge YAML parameters for one node.

    :param defaults: dict of default param_name -> default_value
    :param config_keys: list of nested keys to reach this node's section in the YAML
                        e.g. ['apds', 'lidar_clustering', 'lidar_clustering_combined_once']
    :param config_file_arg: path from --config-file, or empty string
    :return: merged dict of param_name -> value
    """
    # 1) decide which file to use
    if config_file_arg and os.path.isfile(config_file_arg):
        path = config_file_arg
    else:
        cwd_path = os.path.join(os.getcwd(), 'amr_config.yaml')
        path = cwd_path if os.path.isfile(cwd_path) else None

    # 2) load YAML if available
    if path:
        with open(path, 'r') as f:
            full_cfg = yaml.safe_load(f) or {}
        # drill down into the nested keys
        node_cfg = full_cfg
        for key in config_keys:
            node_cfg = node_cfg.get(key, {})
        if not isinstance(node_cfg, dict):
            node_cfg = {}
        # merge, giving precedence to file
        merged = defaults.copy()
        merged.update(node_cfg)
        return merged

    # 3) fallback: no file found
    return defaults.copy()



def main():
    rclpy.init()
    parser = argparse.ArgumentParser(
        description="Mqtt_bridge ROS2 node loading params from YAML"
    )
    parser.add_argument(
        '--config-file', '-c',
        default='',
        help='Full path to amr_config.yaml (overrides workspace root file)'
    )
    args, ros_cli_args = parser.parse_known_args()
    robot_id = os.getenv('ROBOT_ID', 'BYD_005')  # Default to 'FBPT011' if not set
    defaults = {
        "mqtt_broker": "192.168.68.85",  # Or use: os.getenv('MQTT_BROKER')
        "mqtt_port": 1883,
        "reconnect_delay": 5,
        "robot_id": robot_id,            # Or use: os.getenv('ROBOT_ID')
        "robot_ip": "192.168.68.64",     # Or use: os.getenv('ROBOT_IP')
        "robot_length": 0.45,
        "robot_width": 0.45,
        "ros2_topics": {
            "current_pose": "current_pose",
            "odometry": "odometry/filtered",
            "remaining_path": "remaining_path",
            "task_request": f"{robot_id}/task_request",
            "add_nodes": "add_nodes",
            "local_path_active": f"{robot_id}/local_path_active",
            "task": f"{robot_id}/task",
            "conflict_action": f"{robot_id}/conflict_action",
            "task_action": f"{robot_id}/task_action",
            "robot_dimensions": "robot_dimensions"
        },
        "mqtt_topics": {
            "task": f"{robot_id}/task",
            "conflict_action": f"{robot_id}/conflict_action",
            "task_action": f"{robot_id}/task_action",
            "action_response": "fms/action_response",
            "local_path_active": f"{robot_id}/local_path_active",
            "current_pose": f"{robot_id}/current_pose",
            "task_request": f"{robot_id}/task_request",
            "dimensions": f"{robot_id}/dimensions",
            "odometry": f"{robot_id}/odometry",
            "remaining_path": f"{robot_id}/remaining_path",
            "add_nodes": f"{robot_id}/add_nodes"
        },
        "publish_dimensions_period": 5.0,
        "path_list_file": "/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/path_list.json"
    }
    
    config_keys = [
        'ros2_mqtt_bridge',  # Top-level key for this node
        'mqtt_bridge',       # Second-level key for MQTT bridge settings
    ]

    params = load_node_params(defaults, config_keys, args.config_file)
    # print("Loaded parameters:", params)
    node = RosMqttBridge(params)
    # executor = MultiThreadedExecutor()  # One thread per callback
    # executor.add_node(node)
    try:
        # executor.spin()
        rclpy.spin(node)
    
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node (Ctrl+C)")
        node.get_logger().error(traceback.format_exc())

    except Exception as e:
        node.get_logger().error("Exception in main loop: %s" % str(e))
        node.get_logger().error(traceback.format_exc())
    finally:
        if node.task_process is not None and node.task_process.poll() is None:
            print("Cancelling task process...")
            os.killpg(os.getpgid(node.task_process.pid), signal.SIGINT)
            # os.killpg(os.getpgid(node.task_process.pid), signal.SIGTERM)
            node.task_process.kill()
            node.task_process.wait()
            node.velo_pub.publish(Float64(data=0.0))
            node.steer_pub.publish(Float64(data=0.0))
            node.task_process = None
        node.mqtt_client.publish(node.mqtt_topics['action_response'],f"{node.robot_ip}/action_response:{node.robot_id}=('cancel_mna',{node.task_id})", qos=2)
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()