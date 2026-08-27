import argparse
import os
import yaml
import signal
import sys
import psutil
import time
import threading

import networkx as nx
from paho.mqtt import client as mqtt_client
from collections import deque
import sqlite3
import pandas as pd

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Float64
from ament_index_python.packages import get_package_share_directory


# ------------------------------------------------------------
# UTILITY: KILL CHILD PROCESSES
# ------------------------------------------------------------
def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
        return

    for child in parent.children(recursive=True):
        child.terminate()

    gone, alive = psutil.wait_procs(parent.children(recursive=True), timeout=5)
    for p in alive:
        p.kill()


# ------------------------------------------------------------
# STOP ROBOT HELPER (created once, used in signal handler)
# ------------------------------------------------------------
class StopRobotHelper:
    def __init__(self, node: Node):
        self.node = node

        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = node.create_publisher(String, '/state', 10)
        self.vel_pub = node.create_publisher(Float64, '/velocity', 10)
        self.steer_pub = node.create_publisher(Float64, '/steering_angle', 10)

        # Prebuild messages
        self.stop_twist = Twist()
        self.stop_twist.linear.x = 0.0
        self.stop_twist.angular.z = 0.0

        self.state_msg = String()
        self.state_msg.data = "auto"

        self.zero_float = Float64()
        self.zero_float.data = 0.0

    def stop_robot(self):
        for _ in range(60):
            self.cmd_vel_pub.publish(self.stop_twist)
            self.state_pub.publish(self.state_msg)
            self.vel_pub.publish(self.zero_float)
            self.steer_pub.publish(self.zero_float)
            time.sleep(0.01)


# -------- Global reference for the signal handler --------
_stop_helper: StopRobotHelper = None


# ------------------------------------------------------------
# SIGNAL HANDLER (safe)
# ------------------------------------------------------------
def signal_handler(sig, frame):
    print("\n[Signal] Stop requested, halting robot...")

    # stop robot safely
    if _stop_helper:
        _stop_helper.stop_robot()

    kill_child_processes(os.getpid())

    rclpy.shutdown()
    sys.exit(0)


# ------------------------------------------------------------
# PARAM LOADER
# ------------------------------------------------------------


def load_node_params(defaults, keys, config_file):

    # --------------------------------------------------
    # 1) Decide which YAML file to load
    # --------------------------------------------------
    yaml_data = {}

    if config_file:
        # User explicitly provided config --> must exist
        if not os.path.isfile(config_file):
            raise FileNotFoundError(f"Config file not found: {config_file}")
        yaml_path = config_file

        with open(yaml_path, "r") as f:
            yaml_data = yaml.safe_load(f) or {}

    else:
        # Use package's default YAML
        try:
            pkg_share = get_package_share_directory("workflow_node")
            yaml_path = os.path.join(pkg_share, "amr_config.yaml")
        except:
            yaml_path = None

        # If YAML exists, load it
        if yaml_path and os.path.isfile(yaml_path):
            with open(yaml_path, "r") as f:
                yaml_data = yaml.safe_load(f) or {}

    # --------------------------------------------------
    # 2) Drill into nested keys in YAML
    # --------------------------------------------------
    cfg = yaml_data
    for k in keys:
        cfg = cfg.get(k, {})

    # --------------------------------------------------
    # 3) Merge defaults + YAML
    # --------------------------------------------------
    merged = defaults.copy()
    if isinstance(cfg, dict):
        merged.update(cfg)

    # --------------------------------------------------
    # 4) Resolve paths (YAML OR DEFAULTS)
    # --------------------------------------------------
    pkg_share = get_package_share_directory("workflow_node")

    path_keys = ["database_path", "graphml_path", "constructed_rs_path"]

    for key in path_keys:
        if key not in merged:
            continue

        val = merged[key]

        if not isinstance(val, str):
            continue

        # If it starts with "/" but file does not exist -> treat as relative
        if val.startswith("/") and not os.path.exists(val):
            val = val.lstrip("/")

        # Pure relative paths -> append to package share
        if not os.path.isabs(val):
            val = os.path.join(pkg_share, val)

        merged[key] = val

    return merged




# ------------------------------------------------------------
# SAFETY PUBLISHER
# ------------------------------------------------------------
class SafetyPublisher(Node):
    def __init__(self):
        super().__init__('Task_allocator')

        # Only ONE publisher for /byd/safety
        self.safety_pub = self.create_publisher(String, '/byd/safety', 10)

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_pub = self.create_publisher(String, '/byd/current_task', 10)
        self.state_pub = self.create_publisher(String, '/byd/status', 10)

        print("[SafetyPublisher] Publishers created")


# ------------------------------------------------------------
# MQTT CLIENT
# ------------------------------------------------------------
class mqttClient(mqtt_client.Client):

    def __init__(self, broker, port=1883, use_async_connect=False):
        super().__init__(
            mqtt_client.CallbackAPIVersion.VERSION2,
            str(id(self)),
            clean_session=True,
            protocol=mqtt_client.MQTTv311
        )

        self.broker = broker
        self._client_host_ip = None

        if use_async_connect:
            self.connect_async(broker, port)
        else:
            self.connect(broker, port)

        self._topic_callbacks = {}
        self._queued_subs = deque()
        self._queued_pubs = deque()

    # ----------------------
    def on_connect(self, client, userdata, flags, rc, props):
        if rc == 0:
            print("[MQTT] Connected")

            subs, self._queued_subs = self._queued_subs, None
            pubs, self._queued_pubs = self._queued_pubs, None

            while subs:
                self.subscribe2topic(*subs.pop())

            while pubs:
                self.publish2topic(*pubs.pop())
        else:
            print(f"[MQTT] Failed connect, code={rc}")

    # ----------------------
    def on_message(self, client, userdata, msg):
        payload = msg.payload.decode()
        div = payload.find('/')
        if div != -1 and msg.topic in self._topic_callbacks:
            head, body = payload[:div], payload[div + 1:]
            threading.Thread(
                target=self._topic_callbacks[msg.topic],
                args=(head, body)
            ).start()

    # ----------------------
    def subscribe2topic(self, topic, cb, qos=0):
        if self._queued_subs is None:
            if topic not in self._topic_callbacks:
                self.subscribe(topic, qos)
            self._topic_callbacks[topic] = cb
        else:
            self._queued_subs.append((topic, cb, qos))

    # ----------------------
    def publish2topic(self, topic, message, qos=0, ignore_result=False):
        if self._queued_pubs is not None:
            self._queued_pubs.append((topic, message, qos, ignore_result))
            return

        prefix = self._client_host_ip or self.broker
        payload = prefix + '/' + message

        while True:
            result = self.publish(topic, payload, qos)
            if ignore_result:
                break
            if result[0] == 0:
                print(f"[MQTT] Sent `{payload}` → `{topic}`")
                break
            print(f"[MQTT] Retry send → `{topic}`")


# ------------------------------------------------------------
# WORKFLOW HANDLER
# ------------------------------------------------------------
class WorkflowHandler(Node):
    def __init__(self, movement, action, params):
        super().__init__('workflow_handler')

        self.robot_id = params['robot_id']
        self.robot_ip = params['robot_ip']
        self.mqtt_broker = params['mqtt_broker']
        self.database_path = params['database_path']
        self.graphml_path = params['graphml_path']
        self.constructed_rs_path = params['constructed_rs_path']
        self.parking_gap_threshold = params['parking_gap_threshold']
        self.movement_gap_threshold = params['movement_gap_threshold']
        self.rs_path_scale = params['rs_path_scale']
        self.rs_path_turn_radius = params['rs_path_turn_radius']
        self._params_spline_path = params['spline_path']
        self._params_parking_control = params['parking_control']
        self._params_pp_control = params['pp_control']
        self.movement = movement
        self.action = action
        self.operation_state = '0'

        self.task_request_pub = self.create_publisher(
            String,
            f"{self.robot_id}/task_request",
            10
        )

        # MQTT
        self.mqtt = mqttClient(self.mqtt_broker)
        self.mqtt._client_host_ip = self.robot_ip
        self.mqtt.loop_start()

        print(f"[WorkflowHandler] Started for movement={movement}, action={action}")

        # ------------------------------------------------------------
# LOAD LOCATION DATA (DB)
# ------------------------------------------------------------
        self.error_status = None        

# Extract location data from DB
        try:
            (
                self.left_easy_dock_dict,
                self.right_easy_dock_dict,
                self.dock_location_dict,
                self.dock_station_end_line_dict,
                self.station_level_dict
            ) = self.fetch_location_data_from_db()

            self.get_logger().info(f"Location dictionary loaded: {self.dock_location_dict}")

        except Exception as e:
            self.error_status = f"DB_LOAD_ERROR: {e}"
            self.get_logger().error(f"Failed to load DB location data: {e}")
            raise


# ------------------------------------------------------------
# VALIDATE MOVEMENT ID
# ------------------------------------------------------------

        self.location_id = self.movement  # movement passed from CLI

        if self.location_id not in self.station_level_dict:
            raise KeyError(f"Unknown location_id '{self.location_id}' (not in station_level_dict)")


        self.action_level = self.station_level_dict[self.location_id]
        self.get_logger().info(f"Action level for {self.location_id}: {self.action_level}")


# ------------------------------------------------------------
# FETCH LOCATION PARAMETERS
# ------------------------------------------------------------

        try:
            self.left_easy_dock = self.left_easy_dock_dict[self.location_id]
            self.right_easy_dock = self.right_easy_dock_dict[self.location_id]
            self.dock_location = self.dock_location_dict[self.location_id]
            self.dock_station_end_line = self.dock_station_end_line_dict[self.location_id]

            self.get_logger().info(
                f"[Dock Params] Left={self.left_easy_dock}, "
                f"Right={self.right_easy_dock}, "
                f"Dock={self.dock_location}, "
                f"EndLine={self.dock_station_end_line}"
            )

        except KeyError as e:
            self.error_status = f"LOCATION_PARAM_ERROR: {e}"
            self.get_logger().error(f"Missing key in location dictionaries: {e}")
            raise


# ------------------------------------------------------------
# LOAD GRAPHML & CLEAN POSITIONS
# ------------------------------------------------------------

        self.path_in_nodes = []

        try:
            G_loaded = nx.read_graphml(self.graphml_path)
        except Exception as e:
            self.error_status = f"GRAPHML_LOAD_ERROR: {e}"
            self.get_logger().error(f"Failed to load GraphML: {e}")
            raise


        # Convert string positions to numeric tuples
        for node, data in G_loaded.nodes(data=True):
            pos_str = data.get('position')
            if pos_str:
                try:
                    # ensure tuple(float,float)
                    G_loaded.nodes[node]['position'] = tuple(map(float, pos_str.split(',')))
                except Exception:
                    self.get_logger().warn(f"Node {node} has invalid position format: '{pos_str}'")

        self.G_loaded = G_loaded
        self.get_logger().info("GraphML loaded and node positions parsed successfully.")

    def fetch_location_data_from_db(self):
        """
        Loads all docking/location configuration from SQLite DB and returns:
            left_easy_dock_dict,
            right_easy_dock_dict,
            dock_location_dict,
            dock_station_end_line_dict,
            station_level_dict
        """

        db_path = self.database_path  # use param, not hardcoded
        self.get_logger().info(f"[DB] Loading location data from {db_path}")


        # Tables & queries
        queries = {
            'left_easy_dock': 'SELECT * FROM left_easy_dock;',
            'right_easy_dock': 'SELECT * FROM right_easy_dock;',
            'dock_location': 'SELECT * FROM dock_location;',
            'dock_station_end_line': 'SELECT * FROM dock_station_end_line;'
        }

        dataframes = {}

        try:
            conn = sqlite3.connect(db_path)

            # Load all tables
            for table_name, query in queries.items():
                try:
                    df = pd.read_sql(query, conn)
                    dataframes[table_name] = df
                    self.get_logger().info(f"[DB] Loaded table: {table_name} ({len(df)} rows)")
                except Exception as e:
                    self.get_logger().error(f"[DB] Failed loading table '{table_name}': {e}")
                    dataframes[table_name] = pd.DataFrame()  # keep empty to avoid crash

        except Exception as e:
            self.get_logger().error(f"[DB] Could not open database: {e}")
            raise
        finally:
            try:
                conn.close()
            except:
                pass

        # ----------------------------------
        # Convert dataframe → dict helper
        # ----------------------------------
        def create_location_dict(df, name_col, coord_cols, table_name):
            if df.empty:
                self.get_logger().warn(f"[DB] Table '{table_name}' is empty, returning empty dict")
                return {}

            # Check necessary columns exist
            for col in [name_col] + coord_cols:
                if col not in df.columns:
                    self.get_logger().error(
                        f"[DB] Missing column '{col}' in table '{table_name}'"
                    )
                    return {}

            # Convert to dict[name] = [x, y, z, w]
            return df.set_index(name_col)[coord_cols].astype(float).apply(
                lambda row: row.tolist(), axis=1
            ).to_dict()

        # ----------------------------------
        # Build dictionaries
        # ----------------------------------

        left_easy_dock_dict = create_location_dict(
            dataframes['left_easy_dock'],
            name_col='easy_dock_name',
            coord_cols=['x', 'y', 'z', 'W'],
            table_name='left_easy_dock'
        )

        right_easy_dock_dict = create_location_dict(
            dataframes['right_easy_dock'],
            name_col='easy_dock_name',
            coord_cols=['x', 'y', 'z', 'W'],
            table_name='right_easy_dock'
        )

        dock_location_dict = create_location_dict(
            dataframes['dock_location'],
            name_col='dock_name',
            coord_cols=['x_m', 'y_m', 'pose_z', 'pose_w'],
            table_name='dock_location'
        )

        dock_station_end_line_dict = create_location_dict(
            dataframes['dock_station_end_line'],
            name_col='station_name',
            coord_cols=['station_x', 'station_y', 'pose_z', 'pose_w'],
            table_name='dock_station_end_line'
        )

        # ----------------------------------
        # Station → Level dictionary
        # ----------------------------------
        station_level_dict = {}
        df = dataframes['dock_station_end_line']
        if not df.empty and 'station_name' in df.columns and 'level' in df.columns:
            station_level_dict = dict(zip(df['station_name'], df['level'].astype(int)))
        else:
            self.get_logger().warn("[DB] station_level_dict could not be created (missing columns)")

        # ----------------------------------
        # Final Log
        # ----------------------------------
        self.get_logger().info(f"[DB] Dicts loaded successfully.")

        return (
            left_easy_dock_dict,
            right_easy_dock_dict,
            dock_location_dict,
            dock_station_end_line_dict,
            station_level_dict
        )

# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser()
    parser.add_argument('--config-file', '-c', default='', help="Path to YAML")
    parser.add_argument('movement', help="Movement identifier")
    parser.add_argument('action', help="Action identifier")
    parser.add_argument('state', nargs='?', default='0', help="State ID")
    parsed = parser.parse_args()
    from pathlib import Path
    cwd = Path().resolve()
    # Load params
    defaults = {
        "database_path": "/map_details/piyush_demo_wn.db",
        "graphml_path": "map_details/piyush_demo_waypoints.graphml",
        "constructed_rs_path": "constructed_rs_path.pkl",   #/home/ashu/Ankit/fb_stacker.bak/workflow_node
        "robot_id": "EP_006",
        "mqtt_broker": "192.168.0.53",
        "robot_ip": "192.168.0.50",
        "parking_gap_threshold": 0.2,
        "movement_gap_threshold": 0.2,
        "rs_path_scale": 1,
        "rs_path_turn_radius": 0.75,
        "spline_path": {"scale": 1.0, "runway_length": 0.0, "spacing": 0.01, "runway_steps": 20},
        "parking_control": {"scale": 1, "step_size": 0.01},
        "pp_control": {"scale": 1, "step_size": 0.01}
    }
    params = load_node_params(defaults, ["workflow_node", "workflow_node"], parsed.config_file)

    print(f"[PID] {os.getpid()} running workflow_node")

    # Create root node for stop helper
    root_node = rclpy.create_node("stop_root")
    global _stop_helper
    _stop_helper = StopRobotHelper(root_node)

    # SIGNAL HOOKS
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Main workflow handler
    wh = WorkflowHandler(parsed.movement, parsed.action, params)
    wh.operation_state = parsed.state

    print("[workflow_node] Initialization complete.")

        # -----------------------------
    # EXECUTION SEQUENCE
    # -----------------------------

    operations = {
        1: lambda: wh.operation(),
        # Add more steps here if needed:
        # 2: lambda: wh.docking(),
        # 3: lambda: wh.parking(),
    }

    start_point = wh.operation_state  # int from CLI
    start_point = int(start_point) if start_point else 1

    # Run operations in sorted order
    for state in sorted(operations.keys()):
        if state >= start_point:
            result = operations[state]()
            wh.get_logger().info(f"Executed operation {state}, result={result}")

    print("here")

    wh.sequence_complete = True
    wh.destroy_node()
    root_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
