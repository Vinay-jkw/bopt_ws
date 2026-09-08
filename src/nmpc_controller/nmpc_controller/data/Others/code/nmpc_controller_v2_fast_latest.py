import argparse
import math
import json
import sqlite3
from typing import Literal
import pandas as pd
import os
import time
import pickle
import yaml
import matplotlib.pyplot as plt
import numpy as np
from rsplan import planner
from scipy.interpolate import splprep, splev
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
import signal
from nav_msgs.msg import Path  # Import the Path message
from visualization_msgs.msg import Marker  # Import the Marker message
import networkx as nx
from scipy.interpolate import CubicSpline
import ast

# SCALE = 60
# MIN_VELOCITY = 0.2  # Define a minimum velocity limit
# MAX_VELOCITY = 1.42 # 0.3 # Define a maximum velocity limit
# SLOW_DOWN_DISTANCE = 2.0 # Distance within which the robot starts to slow down
# VELOCITY_SMOOTHING_FACTOR = 0.01  # Smoothing factor for velocity adjustments
# STOPPING_VELOCITY = 0.3


class RobotClient(Node):
    def __init__(self, params):
        super().__init__('robot_client')
        self.robot_id = params['robot_id']
        self.linear_velocity_publisher_topic = params['linear_velocity_publisher_topic']
        self.steering_angle_publisher_topic = params['steering_angle_publisher_topic']
        self.state_publisher_topic = params['state_publisher_topic']
        self.min_velocity = params['min_velocity']
        self.max_velocity = params['max_velocity']
        self._max_vel, self._min_vel = self.max_velocity, self.min_velocity
        self.slow_down_distance = params['slow_down_distance']
        self.velocity_smoothing_factor = params['velocity_smoothing_factor']
        self.stopping_velocity = params['stopping_velocity']
        self.goal_tolerance = params['goal_tolerance']
        self.path_file = params['path_file']
        self.speed_control_lookahead = params['speed_control_lookahead']
        self.path_tracking_lookahead = params['path_tracking_lookahead']
        self.path_file_in_nodes = params['path_file_in_nodes']
        self.graphml_file = params['graphml_file']
        G_loaded =  nx.read_graphml(self.graphml_file)

        for node, data in G_loaded.nodes(data=True):
            pos_str = data.get('position', '')
            if pos_str:
                G_loaded.nodes[node]['position'] = tuple(map(float, pos_str.split(',')))
        self.G_loaded = G_loaded
        self.data_updated = False
        self.declare_parameter('goal_tolerance', self.goal_tolerance)
        self.declare_parameter('path_file', self.path_file)
        self.declare_parameter('path_file_in_nodes', self.path_file_in_nodes)
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.horizon = 5
        self.buffer_action_phase = 0
        self.steering_angles = []
        self.in_lane = False
        self.lane_id = None
        self.on_buffer = False
        self.redirect_dot_prod=None
        self.redirect = False
        self.buffer_id = None
        self.is_halted = False
        self.path = []
        self.path_index = 0
        self.path_received = False
        self.goal_reached = False
        self.goal_tolerance = None
        self.path_file = None
        self.conflict_action=None
        self.lane_action=None
        self.pallet_detected = False
        self.finished = False
        self.terminate = False
        self.has_send_stop_command = False
        self.is_paused = False

        # Load the lookup table
        with open('/home/fbots/bopt_v2_ws/src/nmpc_controller/nmpc_controller/mpc_lookup_table_2.51.pkl', 'rb') as f:
            self.lookup_table = pickle.load(f)

        self.velocity_publisher = self.create_publisher(Float64, self.linear_velocity_publisher_topic, 10)
        self.steering_angle_publisher = self.create_publisher(Float64, self.steering_angle_publisher_topic, 10)
        self.state_publisher = self.create_publisher(String, self.state_publisher_topic, 10)
        self.path_publisher = self.create_publisher(Path, '/visualization_path', 10)  # Path publisher
        self.target_point_publisher = self.create_publisher(Marker, '/target_point_marker',
                                                            10)  # Marker publisher for target point
        self.add_nodes_publisher = self.create_publisher(String, '/add_nodes', 10)
        self.lane_status_pub = self.create_publisher(String, f'{self.robot_id}/lane_status', 10)
        self.create_timer(0.05, self._publish_lane_status)
        self.logging_publisher = self.create_publisher(String, '/remaining_path', 10)
        self.publisher_u = self.create_publisher(Marker, 'box_visualization_marker', 10)


        self.local_path_active_pub = self.create_publisher(Bool, f'{self.robot_id}/local_path_active', 10)
        self.local_path_active = False

        self.path_in_nodes = []  # List of node IDs (e.g., ['25', '26', '27'])
        # self.path_index_in_nodes = 0
        self.path_in_nodes_received=False

        self.read_path_in_nodes_from_file(self.get_parameter('path_file_in_nodes').value)
        print("-----------------READ PATH FILE IN NODES---------------------")

        self.current_pose = None
        self.current_velocity = 0.0  # Initialize the current velocity to zero

        qos_settings = QoSProfile(depth=10)
        qos_settings.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            qos_settings)

        self.pd_sub = self.create_subscription(
            Bool,
            '/pallet_detected',
            self.pallet_detection_callback,
            1)

        self.subscription = self.create_subscription(
            Float64,
            '/byd/wheel_velocity',
            self.wv_callback,
            10  # QoS history depth
        )
                # Create a subscriber to the /conflict_action topic
        self.conflict_action_subscription = self.create_subscription(
            String,
            f'{self.robot_id}/conflict_action',
            self.conflict_action_callback,
            10
        )

        self.task_action_subscription = self.create_subscription(
            String,
            f'{self.robot_id}/task_action',
            self.task_action_callback,
            10
        )

        self.lane_cmd_state_subscription = self.create_subscription(
            String,
            f"/{self.robot_id}/lane_cmd_state",
            self.lane_state_callback,
            10
        )
        self.wheel_velocity = 0.0
     
        self.timer = self.create_timer(0., self.follow_path)

        self.read_path_from_file(self.get_parameter('path_file').value)
        # For speed control
        self.sc_lookahead = self.speed_control_lookahead  # meter
        # For path tracking
        self.pt_lookahead = self.path_tracking_lookahead
        self.path_last_point = []

        self.left_easy_dock_dict, self.right_easy_dock_dict, self.dock_location_dict, self.dock_station_end_line_dict = self.fetch_location_data_from_db()
    
    def path_callback(self, msg: String): 
        if not msg.data.strip(): # Ignore empty pathsollow 
            self.get_logger().warn("⚠️ Received an empty path update. Ignoring.") 
            return 

        self.path_in_nodes = msg.data.split(',') 
        self.get_logger().info(f'✅ Updated path_in_nodes: {self.path_in_nodes}') 
    

    def wv_callback(self, msg):
        self.wheel_velocity = msg.data

    def set_parameters(self, path_file, goal_tolerance):
        self.goal_tolerance = goal_tolerance
        self.path_file = path_file
        self.read_path_from_file(self.path_file)
        self.get_logger().info(f'Path file set to: {self.path_file}')
        self.get_logger().info(f'Goal tolerance set to: {self.goal_tolerance}')

    def lane_state_callback(self, msg: String):
        """
        Receives at 10 Hz the JSON that came from RosMqttBridge.publish_lane_state().
        Parse it to update self.in_lane, self.lane_id, self.on_buffer, self.buffer_id.
        """

        try:
            data = json.loads(msg.data)
            self.in_lane = data.get("in_lane", False)
            self.lane_id = data.get("lane_id", None)
            self.on_buffer = data.get("on_buffer", False)
            self.buffer_id = data.get("buffer_id", None)
            self.terminate = data.get("terminate", False)
            self.data_updated = True
        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid JSON in lane_cmd_state: {msg.data}")

    def add_runway_to_path(self, path, runway_length=0.6):
        """
        Add a runway to the end of the received path to prevent the lookahead point from vanishing on approach.
        The runway is added in the direction of the last two points in the path.

        :param path: The original path (list of (x, y) points).
        :param runway_length: Length of the runway to add (in meters).
        :return: The extended path with runway added.
        """
        if len(path) < 2:
            self.get_logger().warn("Path is too short to add a runway.")
            return path

        # Get the last two points on the path
        point1 = path[-2]
        point2 = path[-1]

        # Calculate the direction of the last two points
        dx = point2[0] - point1[0]
        dy = point2[1] - point1[1]

        # Normalize the direction vector
        length = np.sqrt(dx**2 + dy**2)
        dx /= length
        dy /= length

        # Extend the path by adding the runway
        last_point = path[-1]
        runway_end_x = last_point[0] + runway_length * dx
        runway_end_y = last_point[1] + runway_length * dy

        # Add the runway point to the path
        extended_path = path + [(runway_end_x, runway_end_y)]
        return extended_path


    def send_command(self, velocity, steering_angle):
        self.velocity_publisher.publish(Float64(data=velocity))
        self.steering_angle_publisher.publish(Float64(data=steering_angle))
        self.get_logger().info(
            f'Sending command - Velocity: {velocity:.2f} m/s, Steering Angle: {steering_angle:.2f} degrees')
        state_msg = String()
        state_msg.data = 'Custom'
        self.state_publisher.publish(state_msg)

    def send_stop_command(self, velocity, steering_angle):
        self.velocity_publisher.publish(Float64(data=velocity))
        self.steering_angle_publisher.publish(Float64(data=steering_angle))
        self.get_logger().info(
            f'Sending command - Velocity: {velocity:.2f} m/s, Steering Angle: {steering_angle:.2f} degrees')
        state_msg = String()
        state_msg.data = 'Auto'
        self.state_publisher.publish(state_msg)

    def _publish_lane_status(self):
        """
        This function is called at 20 Hz. It serializes `self.in_lane`/`self.lane_id`
        as JSON and publishes it on the ROS topic `<ROBOT_ID>/lane_status`.
        The ROS–MQTT bridge will pick this up and forward it to MQTT.
        """
        if not self.data_updated:
            self.get_logger().warn("No lane status data updated. Skipping publish.")
            return
        payload = {
            "robot_id": self.robot_id,  # e.g. “R1”
            "in_lane": self.in_lane,  # current boolean flag
            "lane_id": self.lane_id,
            "on_buffer": self.on_buffer,
            "buffer_id": self.buffer_id,
            "is_halted": self.is_halted,  # e.g. “L1” or None
            "terminate": self.terminate,
            "redirect": self.redirect
        }
        ros_msg = String()
        ros_msg.data = json.dumps(payload)
        self.lane_status_pub.publish(ros_msg)

    def read_path_in_nodes_from_file(self, file_path):
        with open(file_path, 'rb') as f:
            self.path_in_nodes = pickle.load(f)
        self.path_in_nodes_received = True
        self.destination=self.path_in_nodes[-1]
        print("DEstinatioon",self.destination)
        
    def read_path_from_file(self, file_path):
        with open(file_path, 'rb') as f:
            self.path = pickle.load(f)
        self.path_received = True
        self.final_point = self.path[-1]
        # Add runway to the path to ensure lookahead point doesn't vanish
        self.path = self.add_runway_to_path(self.path)
        self.path_last_point = self.path[-1]

        self.publish_path_for_visualization()

    def publish_path_for_visualization(self):
        """Convert the loaded path to a Path message and publish it."""
        if not self.path:
            self.get_logger().warn("No path loaded to publish.")
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'  # Set the frame_id to match your RViz2 configuration
        path_msg.header.stamp = self.get_clock().now().to_msg()
        # print(self.path)
        for point in self.path:
            # print(type(point[0]))
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0  # Assuming 2D path
            pose.pose.orientation.w = 1.0  # Assuming no rotation for visualization
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Path published for visualization in RViz2.')

    def publish_local_path_active(self, active: bool):
        """Publishes the active state of the local path on {robot_id}/local_path_active."""
        msg = Bool()
        msg.data = active
        self.local_path_active_pub.publish(msg)
        self.local_path_active=active
        print("---------------------",self.local_path_active)
        self.get_logger().info(f"Local path active set to {active}")

    def generate_temporary_path_for_buffer(self, robot_ip, current_node, buffer_node, old_path):
        """Generate a temporary path that includes the buffer node before continuing to the destination."""
        try:
            if current_node not in old_path:
                self.logger.debug("Node %s is not in the given old path.", current_node)
                return []

            # Step 1️⃣: Get the index of the current node in old path
            current_index = old_path.index(current_node)

            # Step 2️⃣: Inject the buffer node **right after** the current node
            temporary_path = (
                    old_path[: current_index + 1]
                    + [buffer_node]
                    + old_path[current_index:]
            )

            self.logger.debug(
                "Generated temporary path via buffer: %s", temporary_path
            )
            return temporary_path

        except ValueError:
            self.logger.debug(
                "Error: %s not found in old path.", current_node
            )
            return []

        except nx.NetworkXNoPath:
            self.logger.debug(
                "No valid path found from %s to buffer %s.", current_node, buffer_node
            )
            return []
        
    def fetch_location_data_from_db(self):
        # Path to the SQLite database file
        db_path = '/home/fbots/bopt_v2_ws/src/workflow_node/map_details/testing_ground_wn.db'

        # Connect to the SQLite database
        conn = sqlite3.connect(db_path)

        # Queries to fetch all data from the specified tables
        queries = {
            'left_easy_dock': 'SELECT * FROM left_easy_dock;',
            'right_easy_dock': 'SELECT * FROM right_easy_dock;',
            'dock_location': 'SELECT * FROM dock_location;',
            'dock_station_end_line': 'SELECT * FROM dock_station_end_line;'
        }

        # Dictionary to store the data from each table
        dataframes = {}

        # Execute each query and store the results in a DataFrame
        for table_name, query in queries.items():
            dataframes[table_name] = pd.read_sql(query, conn)
        print(dataframes)

        # Execute each query and store the results in a DataFrame
        for table_name, query in queries.items():
            dataframes[table_name] = pd.read_sql(query, conn)

        # Function to convert DataFrame to desired dictionary
        def create_location_dict(df, name_col, coord_cols):
            return df.set_index(name_col)[coord_cols].apply(lambda row: row.tolist(), axis=1).to_dict()

        # 1. left_easy_dock
        left_easy_dock_dict = create_location_dict(
            dataframes['left_easy_dock'],
            name_col='easy_dock_name',
            coord_cols=['x', 'y', 'z', 'W']
        )

        # 2. right_easy_dock
        right_easy_dock_dict = create_location_dict(
            dataframes['right_easy_dock'],
            name_col='easy_dock_name',
            coord_cols=['x', 'y', 'z', 'W']
        )

        # 3. dock_location
        dock_location_dict = create_location_dict(
            dataframes['dock_location'],
            name_col='dock_name',
            coord_cols=['x_m', 'y_m', 'pose_z', 'pose_w']
        )

        # 4. dock_station_end_line
        dock_station_end_line_dict = create_location_dict(
            dataframes['dock_station_end_line'],
            name_col='station_name',
            coord_cols=['station_x', 'station_y', 'pose_z', 'pose_w']
        )

        # Close the database connection
        conn.close()
        return left_easy_dock_dict, right_easy_dock_dict, dock_location_dict, dock_station_end_line_dict


    def transform_point(self, point, robot_x, robot_y, robot_orientation):
        cos_theta = np.cos(robot_orientation)
        sin_theta = np.sin(robot_orientation)

        dx = point[0] - robot_x
        dy = point[1] - robot_y
        transformed_x = dx * cos_theta + dy * sin_theta
        transformed_y = -dx * sin_theta + dy * cos_theta

        return [transformed_x, transformed_y]
    
    def euclidean_distance(self,coord1, coord2):
            return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

    def find_nearest_graph_node(self, current_position):
        """
        Finds the closest node in the path_in_nodes list based on current_position.
        It uses get_node_position() to obtain the node's coordinates from the graph
        or, if missing, from the database.
        """
        min_distance = float('inf')
        nearest_node = None
        print("path in _nodes", self.path_in_nodes)
        print("finding nearest node")
        for node in self.clean_path(self.path_in_nodes):
            try:
                node_pos = self.get_node_position(node)
                distance = self.euclidean_distance(node_pos, current_position)
                if distance < min_distance:
                    min_distance = distance
                    nearest_node = node
            except KeyError:
                self.get_logger().info(f"Node {node} not found in graph or DB, skipping.")
                continue
        return nearest_node

    def find_nearest_key(self, position):
        nearest_key = None
        min_position_difference = float('inf')

        for key in self.lookup_table.keys():
            position_difference = np.sqrt((key[0] - position[0]) ** 2 + (key[1] - position[1]) ** 2)

            if position_difference < min_position_difference:
                min_position_difference = position_difference
                nearest_key = key

        return nearest_key
    

    
    
    def find_left_or_right_dock(self, left_dock_pose, right_dock_pose, dock_pose, pose_to_compare, is_last_pose):

        def orientation_difference_deg(q1, q2):
            angle1 = math.degrees(self.quaternion_to_euler(q1[1], 0, 0, q1[0])[2])
            angle2 = math.degrees(self.quaternion_to_euler(q2[1], 0, 0, q2[0])[2])
            print(angle1, angle2, '====================')
            diff = abs(angle1 - angle2) % 360
            return min(diff, 360 - diff)  # Ensure the difference is within [0, 180]

        start = dock_pose
        end = pose_to_compare
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle_rad = math.atan2(dy, dx)
        if is_last_pose == False:
            angle_rad += math.pi
        quaternion = self.euler_to_quaternion(0, 0, angle_rad)

        # Calculate Euler angles (in degrees) for debug
        euler_angle_deg = math.degrees(angle_rad) % 360

        last_orientation = quaternion[2:]  # Assuming quaternion z, w components
        left_orientation = left_dock_pose[2:]
        right_orientation = right_dock_pose[2:]

        # Convert dock orientations to degrees for debug
        left_dock_euler_deg = math.degrees(self.quaternion_to_euler(left_orientation[1], 0,0, left_orientation[0])[2])
        right_dock_euler_deg = math.degrees(self.quaternion_to_euler(right_orientation[1], 0,0, right_orientation[0])[2])

        last_orientation_deg = math.degrees(self.quaternion_to_euler(last_orientation[1], 0,0, last_orientation[0])[2])
       

        left_diff = orientation_difference_deg(last_orientation, left_orientation)
        right_diff = orientation_difference_deg(last_orientation, right_orientation)

        # Debug Statements
        print(f"Dock-Pose to Compare Vector: dx = {dx}, dy = {dy}")
        print(f"Calculated Angle (radians): {angle_rad}")
        print(f"Calculated Angle (degrees): {euler_angle_deg}")
        print(f"Last Orientation (quaternion): {last_orientation}, Euler Degrees: {last_orientation_deg}")
        print(f"Left Dock Orientation (quaternion): {left_orientation}, Euler Degrees: {left_dock_euler_deg}")
        print(f"Right Dock Orientation (quaternion): {right_orientation}, Euler Degrees: {right_dock_euler_deg}")
        print(f"Left Dock Difference: {left_diff}")
        print(f"Right Dock Difference: {right_diff}")

        if left_diff < right_diff:
            print("Choosing left dock")
            return left_dock_pose
        else:
            print("Choosing right dock")
            return right_dock_pose
        
    def filter_points_within_radius(self, location_to_compare, points, radius):
        """
        Filters out points that lie within a certain radius of the given location.
        
        Args:
        - location_to_compare (list): The reference location in the format [x, y, z, w].
        - points (list of lists): A list of points where each point is [x, y, z, w].
        - radius (float): The radius within which points will be removed.
        
        Returns:
        - list of lists: The filtered list of points that lie outside the given radius.
        """
        x, y = location_to_compare[0], location_to_compare[1]  # Extract x, y from location_to_compare
        filtered_points = []
        for point in points:
            point_x, point_y = point[0], point[1]
            distance = math.sqrt((point_x - x)**2 + (point_y - y)**2)
            if distance > radius:
                filtered_points.append(point)
        return filtered_points

    def generate_local_path(self):
        self.get_logger().info("Conflict detected: Splitting path and generating a local path...")
        state = self.current_pose
        if state is None:
            return

        robot_x = state.pose.position.x  
        robot_y = state.pose.position.y  
        # Use the modified find_nearest_graph_node to get the current node.
        current_node = self.find_nearest_graph_node((robot_x, robot_y))
        if current_node:
            print(f"Nearest Graph Node Found: {current_node}")
            local_path = self.generate_temporary_path(self.path_in_nodes, current_node)
            # print("New local path:", local_path, file=open('/home/fbots/bopt_v2_ws/nm.log', 'a+'))

            # Publish True immediately after generating temporary nodes.
            self.publish_local_path_active(True)
            
            # Convert this local path into a path with angles (quaternions) for the NMPC:
            path_with_quaternions = self.handle_local_path(local_path)
            dest = self.path_in_nodes[-1]
            path_with_quaternions = self.filter_points_within_radius(self.dock_location_dict[dest][:2], path_with_quaternions, 2.0)
            path_with_quaternions = self.filter_points_within_radius([robot_x, robot_y], path_with_quaternions, 1.5)
            # print("self.destination:", self.destination, file=open('/home/fbots/bopt_v2_ws/nm.log', 'a+'))
            # print("path_with_quaternions:", path_with_quaternions, file=open('/home/fbots/bopt_v2_ws/nm.log', 'a+'))
            if not path_with_quaternions:
                path_with_quaternions.append(self.dock_location_dict[dest][:2])    
            end_easy_dock = self.find_left_or_right_dock(self.left_easy_dock_dict[dest], self.right_easy_dock_dict[dest], self.dock_location_dict[dest], path_with_quaternions[-1], is_last_pose=True)
            
            path_with_quaternions.append(end_easy_dock)

            # Generate a spline path based on the temporary path:
            spline_path = self.generate_spline_path(self.get_node_position(current_node), path_with_quaternions)

            # Update the internal state with the new temporary path:
            self.path = spline_path
            self.path_in_nodes = local_path

            # Optionally save to files if needed:
            with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path_in_nodes.pkl', 'wb') as file:
                pickle.dump(self.path_in_nodes, file)
            with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path.pkl', 'wb') as file:
                pickle.dump(spline_path, file)

            self.path_received = True
            self.path_in_nodes_received = True
            self.path_index = 0
            self.publish_path_for_visualization()
            self.get_logger().info("New local path and spline path successfully generated.")
        else:
            print("Could not determine the current node. Cannot generate a new local path.")

    def calculate_curvature_finite_diff(self, path):
        if len(path) < 3:
            return 0
        # path = path[int((len(path)*0.66)):] #looking at last third of path

        try:
            path = np.array(path)
            x = path[:, 0]
            y = path[:, 1]

            dx = np.gradient(x)
            dy = np.gradient(y)
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)

            denominator = (dx ** 2 + dy ** 2) ** 1.5 + 1e-6  # adding epsilon to avoid division by zero
            curvature = np.abs(dx * ddy - dy * ddx) / denominator
            return np.max(curvature)
        except Exception as e:
            print(f"Error in calculate_curvature_finite_diff: {e}")
            return 0

    def get_target_point(self, robot_x, robot_y):
        lookahead_distance = 0.3

        velocity_factor = 0.7
        lookahead_distance = lookahead_distance + velocity_factor * abs(self.wheel_velocity)

        min_lookahead = 0.4  # meters
        max_lookahead = 3.0  # meters

        lookahead_distance = max(min_lookahead, min(lookahead_distance, max_lookahead))


        if self.wheel_velocity < 0:
            lookahead_distance += 0.2

        for i in range(self.path_index, len(self.path)): 
            point = self.path[i]
            distance = math.hypot(point[0] - robot_x, point[1] - robot_y)
            if distance > lookahead_distance:
                self.path_index = i
                break

        curvature = self.calculate_curvature(0.15)
        # self.curvatures.append(curvature)  # Store the calculated curvature

        if curvature < 0.02:  # If curvature is low, increase the lookahead distance
            lookahead_distance += 1.15
        
        

        for i in range(self.path_index, len(self.path)):
            point = self.path[i]
            distance = math.hypot(point[0] - robot_x, point[1] - robot_y)
            if distance > lookahead_distance:
                self.path_index = i
                return point
        return self.path[-1]
    
    def get_node_position(self, node):
        """
        Returns the (x, y) position of a node.
        First, it checks the loaded graph. 
        If the node is not found there, it looks up in the fetched DB dictionaries.
        """
        if node in self.G_loaded.nodes:
            return self.G_loaded.nodes[node]['position']
        elif node in self.dock_location_dict:
            # Assuming dock_location_dict values are lists [x, y, ...]
            return self.dock_location_dict[node][:2]
        elif node in self.left_easy_dock_dict:
            return self.left_easy_dock_dict[node][:2]
        elif node in self.right_easy_dock_dict:
            return self.right_easy_dock_dict[node][:2]
        elif node in self.dock_station_end_line_dict:
            return self.dock_station_end_line_dict[node][:2]
        else:
            raise KeyError(f"Node '{node}' not found in graph or any database dictionaries!")

    def quaternion_to_euler(self,w, x, y, z):
            """
            Convert a quaternion to Euler angles (roll, pitch, yaw).
            """
            # Roll (x-axis rotation)
            t0 = 2.0 * (w * x + y * z)
            t1 = 1.0 - 2.0 * (x * x + y * y)
            roll = math.atan2(t0, t1)
            
            # Pitch (y-axis rotation)
            t2 = 2.0 * (w * y - z * x)
            t2 = 1.0 if t2 > 1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch = math.asin(t2)
            
            # Yaw (z-axis rotation)
            t3 = 2.0 * (w * z + x * y)
            t4 = 1.0 - 2.0 * (y * y + z * z)
            yaw = math.atan2(t3, t4)
            
            return (roll), (pitch), (yaw)   

    def generate_spline_path(self, start_pose, graph_path, scale=1.0, runway_length=0.0, spacing=0.01):
        

        # Ensure the start is the first waypoint
        graph_path.insert(0, start_pose)

        # Extract coordinates (scaled)
        points = [(wp[0] * scale, wp[1] * scale) for wp in graph_path]
        unique_points = [points[0]]
        for p in points[1:]:
            if p != unique_points[-1]:
                unique_points.append(p)
            else:
                print(f"Duplicate point {p} removed.")
        points = unique_points
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        # If there's only one point or none, no spline to create
        if len(points) < 2:
            print("Not enough points to generate a spline.")
            return points

        # Parameterize by cumulative distance along the path
        distances = [0.0]
        for i in range(1, len(points)):
            dist = math.hypot(points[i][0] - points[i-1][0],
                            points[i][1] - points[i-1][1])
            distances.append(distances[-1] + dist)
        total_dist = distances[-1]

        # Extract the orientation of the last waypoint to compute the slope
        # Assuming quaternion in the form (qz, qw)
        last_wp = graph_path[-1]
        qz, qw = last_wp[2], last_wp[3]

        # Convert quaternion to euler angles to get final yaw
        # For example, you might already have a helper like:
        # final_yaw = self.quaternion_to_euler(qw, 0, 0, qz)[2]
        # Here is a dummy example for yaw only:
        # (Replace this with your own quaternion->yaw function)
        final_yaw = self.quaternion_to_euler(qw, 0, 0, qz)[2] + math.pi

        # We want the boundary condition derivative at the start to be 0 
        # (tangent = 0) and at the end to match final_yaw
        final_slope_x = math.cos(final_yaw)
        final_slope_y = math.sin(final_yaw)

        # Create cubic splines for x and y as functions of distance
        cs_x = CubicSpline(distances, xs, bc_type=((1, 0), (1, final_slope_x)))
        cs_y = CubicSpline(distances, ys, bc_type=((1, 0), (1, final_slope_y)))

        # ----------------------------------------------------------------
        #    SAMPLE THE SPLINE BASED ON THE DESIRED SPACING
        # ----------------------------------------------------------------
        # Instead of a fixed number of samples, we'll create a distance array
        # with increments of 'spacing' until we cover total_dist.
        # For example, if spacing=0.01, then we get a point every 0.01 (units).
        sample_distances = np.arange(0, total_dist, spacing)
        # Make sure we include the very last point (total_dist) exactly
        if sample_distances[-1] < total_dist:
            sample_distances = np.append(sample_distances, total_dist)

        # Evaluate spline at each distance
        smoothed_coords = []
        for d in sample_distances:
            x_val = cs_x(d)
            y_val = cs_y(d)
            smoothed_coords.append((float(x_val), float(y_val)))

        # ----------------------------------------------------------------
        #    ADD THE RUNWAY
        # ----------------------------------------------------------------
        # Compute final heading at the end of the path
        # If you want it strictly to match final_yaw, you can do so
        runway_steps = 10  # number of sub-steps for runway
        step_length = runway_length / runway_steps if runway_steps > 0 else 0.0

        if runway_length > 0:
            final_point = smoothed_coords[-1]
            for i in range(1, runway_steps + 1):
                x_new = final_point[0] + i * step_length * math.cos(final_yaw)
                y_new = final_point[1] + i * step_length * math.sin(final_yaw)
                smoothed_coords.append((float(x_new), float(y_new)))

        return smoothed_coords

    def generate_rs_path(self, start_pose, graph_path, turn_radius, rev_drive, SCALE=1):
        
        rs_path_waypoints = []
        print('gp:::',graph_path)
        for waypoint in graph_path:
            if waypoint == graph_path[-1] and rev_drive == True:
                rs_path_waypoints.append([waypoint[0]*SCALE, waypoint[1]*SCALE, self.quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2], turn_radius, 0.0*SCALE])
            elif waypoint == graph_path[-1] and rev_drive == False:
                rs_path_waypoints.append([waypoint[0]*SCALE, waypoint[1]*SCALE, self.quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2], turn_radius, 0.0*SCALE])
            else:
                rs_path_waypoints.append([waypoint[0]*SCALE, waypoint[1]*SCALE, self.quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2], turn_radius, 0.0])
        

        
        step_size = 0.01 * SCALE

        waypoints = []
        current_pose = (*start_pose[:2], self.quaternion_to_euler(start_pose[3],0,0, start_pose[2])[2])

        for end_pose in rs_path_waypoints:
            end_x, end_y, end_yaw, turn_radius, runway_length = end_pose
            end_pose_tuple = (end_x, end_y, end_yaw)
            path = planner.path(current_pose, end_pose_tuple, turn_radius, runway_length, step_size)
            waypoints.extend([(wp.x, wp.y) for wp in path.waypoints()])
            current_pose = end_pose_tuple

        return waypoints 

    def calculate_path_distance(self, path, threshold=None):
        """
        Calculate the total distance of a path represented as a list of (x, y) tuples.
        """
        if len(path) < 2:
            return 0.0

        total_distance = 0.0
        for i in range(1, len(path)):
            dx = path[i][0] - path[i-1][0]
            dy = path[i][1] - path[i-1][1]
            total_distance += math.sqrt(dx**2 + dy**2)
            if threshold is not None and total_distance >= threshold:
                return total_distance

        return total_distance

    

    def send_robot_to_buffer_local_path(self, buffer_node, phase: Literal['BE', 'BDI', 'BL', 'BDO', 'DE'] = 'BE'):
        """
        phase: 'BE' - buffer easy dock, 'BDI' - buffer dock in, 'BL' - buffer end line, 'BDO' - buffer dock out, 'DE' - destination easy dock.
        """
        if phase not in ['BE', 'BDI', 'BL', 'BDO', 'DE']:
            self.get_logger().error(f"Invalid phase '{phase}' provided. Must be one of 'BE', 'BD', 'BL', or 'DE'.")
            return

        if buffer_node is None and phase == 'BE':
            self.get_logger().error("Buffer node is None. Cannot proceed with sending robot to buffer.")
            return
        # 1) find which graph node we’re currently closest to
        state = self.current_pose
        if state is None:
            print("No state===============")
            return
        self.get_logger().info(f"Current Buffer {buffer_node} and phase {phase}.")

        # Generate a spline path based on the temporary path:
        if phase == 'BE' and not self.buffer_action_phase:
            robot_x = state.pose.position.x
            robot_y = state.pose.position.y
            current_node = self.find_nearest_graph_node((robot_x, robot_y))
            if not current_node:
                self.get_logger().info("No current node found returning from this statement")
                return

            # 2) get the original final destination
            final_dest = self.destination
            buffer_location_nearest_node = self.find_nearest_connected_node(self.dock_location_dict[buffer_node][:2])
            # 3) compute a valid graph path: current → buffer → final
            self.first_leg = self.find_dynamic_valid_path_with_turns(source=current_node, target=buffer_location_nearest_node) + [buffer_node]
            self.second_leg = self.find_dynamic_valid_path_with_turns(source=buffer_location_nearest_node, target=self.path_in_nodes[-2]) + [final_dest]
            with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/first_leg.json', 'wb') as file:
                file.write(json.dumps(self.first_leg).encode('utf-8'))
            with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/second_leg.json', 'wb') as file:
                file.write(json.dumps(self.second_leg).encode('utf-8'))
            path_with_quaternions = self.find_path_and_angles_between_points_for_conflict(self.G_loaded, self.first_leg[:-1])
            if self.calculate_path_distance(path_with_quaternions, 3.0) > 2.0:
                path_with_quaternions = self.filter_points_within_radius(self.dock_location_dict[buffer_node][:2], path_with_quaternions, 2.0)
            current_pose = (state.pose.position.x, state.pose.position.y, state.pose.orientation.z, state.pose.orientation.w)
            if not path_with_quaternions:
                path_with_quaternions.append(self.dock_location_dict[buffer_node])
            end_easy_dock = self.find_left_or_right_dock(self.left_easy_dock_dict[buffer_node], self.right_easy_dock_dict[buffer_node], self.dock_location_dict[buffer_node], path_with_quaternions[-1], is_last_pose=True)
            path_with_quaternions.append(end_easy_dock)
            self.path = self.generate_spline_path(self.get_node_position(current_node), path_with_quaternions)
            self.path = self.add_runway_to_path(self.path)
            self.path_in_nodes = self.first_leg.copy()
            with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path_in_nodes.pkl', 'wb') as file:
                pickle.dump(self.path_in_nodes, file)
            self.destination = buffer_node
            self.goal_tolerance = 0.4
        elif phase == 'BDI' and self.buffer_action_phase & 0x1:
            current_pose = (state.pose.position.x, state.pose.position.y, state.pose.orientation.z, state.pose.orientation.w)
            self.path = self.generate_rs_path(current_pose, [self.dock_location_dict[self.first_leg[-1]]], 0.75, False, SCALE=1.0)
            self.goal_tolerance = 0.25
        elif phase == 'BL' and self.buffer_action_phase & 0x3:
            current_pose = (state.pose.position.x, state.pose.position.y, state.pose.orientation.z, state.pose.orientation.w)
            # self.path = self.generate_rs_path(current_pose, [self.dock_station_end_line_dict[buffer_node]], 0.75, False, SCALE=1.0)
            self.path = self.generate_spline_path(current_pose[:2], [self.dock_station_end_line_dict[self.first_leg[-1]]])
        elif phase == 'BDO' and self.buffer_action_phase & 0x7:
            current_pose = (state.pose.position.x, state.pose.position.y, state.pose.orientation.z, state.pose.orientation.w)
            # self.path = self.generate_rs_path(current_pose, [self.dock_location_dict[buffer_node]], 0.75, False, SCALE=1.0)
            self.path_in_nodes = self.second_leg.copy()
            self.destination = self.path_in_nodes[-1]
            with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path_in_nodes.pkl', 'wb') as file:
                pickle.dump(self.path_in_nodes, file)
            self.path = self.generate_spline_path(current_pose[:2], [self.dock_location_dict[self.first_leg[-1]]])
        elif phase == 'DE' and self.buffer_action_phase & 0xF:
            final_dest = self.second_leg[-1]
            current_pose = (state.pose.position.x, state.pose.position.y, state.pose.orientation.z, state.pose.orientation.w)
            # try:
            #     second_leg_nearest_node = int(self.path_in_nodes.index(buffer_node))+1
            # except ValueError:
            #     second_leg_nearest_node = 0
            # self.get_logger().info(f"Second leg nearest node: {second_leg_nearest_node}")
            path_with_quaternions = self.find_path_and_angles_between_points_for_conflict(self.G_loaded, self.second_leg[:-1])
            if self.calculate_path_distance(path_with_quaternions, 3.0) > 2.0:
                path_with_quaternions = self.filter_points_within_radius(current_pose[:2], path_with_quaternions, 2.0)
            if self.calculate_path_distance(path_with_quaternions, 3.0) > 2.0:
                path_with_quaternions = self.filter_points_within_radius(self.dock_location_dict[final_dest][:2], path_with_quaternions, 2.0)
            if not path_with_quaternions:
                path_with_quaternions.append(self.dock_location_dict[final_dest])
            self.get_logger().info(f"Path with quaternions: {path_with_quaternions}")
            end_easy_dock = self.find_left_or_right_dock(self.left_easy_dock_dict[final_dest], self.right_easy_dock_dict[final_dest], self.dock_location_dict[final_dest], path_with_quaternions[-1], is_last_pose=True)
            path_with_quaternions.append(end_easy_dock)
            
            # Generate a spline path based on the temporary path:
            self.path = self.generate_spline_path(current_pose[:2], path_with_quaternions)
            self.path = self.add_runway_to_path(self.path)

            # self.path_in_nodes = self.second_leg
            # with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path_in_nodes.pkl', 'wb') as file:
            #     pickle.dump(self.path_in_nodes, file)
            self.buffer_action_phase = 0
            self.first_leg, self.second_leg = None, None

        with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path.pkl', 'wb') as file:
                pickle.dump(self.path, file)
        self.path_index = 0
        if phase != 'DE':
            self.buffer_action_phase = (self.buffer_action_phase << 1) | 1
        # 9) flags, persist, and publish to RViz
        self.path_received = True
        self.path_in_nodes_received = True
        self.final_point = self.path[-1]
        self.path_last_point = self.path[-1]
        self.publish_path_for_visualization()
        self.update_remaining_path_in_nodes(*current_pose)


    def generate_temporary_path1(self, original_path, current_node, deviation_distance=0.7, nodes_to_split=8):
        
        if current_node not in original_path:
            print(f"[Workflow] Node {current_node} is not in the given path.")
            return []

        current_index = original_path.index(current_node)

        if current_index + 1 >= len(original_path):
            print(f"[Workflow] No next node found after {current_node}.")
            return []

        # Get the next few nodes
        next_nodes = original_path[current_index + 1:current_index + nodes_to_split]

        temporary_nodes = []
        for i, node in enumerate(next_nodes):
            if node not in self.G_loaded.nodes:
                print(f"[Workflow] Node {node} not found in the graph.")
                continue

            node_position = self.G_loaded.nodes[node].get("position")
            prev_position = self.G_loaded.nodes[current_node].get("position")

            dx = node_position[0] - prev_position[0]
            dy = node_position[1] - prev_position[1]

            length = math.sqrt(dx**2 + dy**2)
            if length == 0:
                continue

            perp_x = -dy / length * deviation_distance
            perp_y = dx / length * deviation_distance

            temp_x = node_position[0] + perp_x
            temp_y = node_position[1] + perp_y
            temp_node_id = f"temp_{current_node}_{i + 1}"

            temporary_nodes.append((temp_node_id, (temp_x, temp_y)))
            current_node = node

            # Publish the temporary nodes if any were created.
        if temporary_nodes:
            msg = String()
            # Convert the list of temporary nodes to JSON string for structured output.
            msg.data = json.dumps(temporary_nodes)
            self.add_nodes_publisher.publish(msg)
            self.get_logger().info(f"Published temporary nodes on topic '/add_nodes': {temporary_nodes}")

        return [original_path[current_index]] + temporary_nodes + original_path[current_index + nodes_to_split:-1]
    

    def generate_temporary_path(
        self,
        original_path: list,
        current_node: str,
        deviation_distance: float = 1.4,
        node_to_split: int = 8,
        angle_threshold: float = 60.0,
        reverse_output: bool = False
    ) -> list:
        """
        Create a temporary (deviated) path, prune away “anomalous” temp-nodes,
        and optionally return the final path reversed.

        A temp-node is **removed** when:
        • it is a local peak in the angle sequence
            (angle[i-1] < angle[i] > angle[i+1])
        • and the peak angle is larger than `angle_threshold` degrees.

        Parameters
        ----------
        original_path : list[str]
            Ordered list of real node-ids that make up the reference path.
        current_node  : str
            Node in `original_path` where we start inserting deviations.
        deviation_distance : float
            Perpendicular offset (metres, map units, etc.) for temp deviation.
        node_to_split : int
            How many real nodes ahead of `current_node` should be deviated.
        angle_threshold : float
            Peak angle (deg) above which a local-peak temp-node is discarded.
        reverse_output : bool
            If True the final path is returned in reverse order.

        Returns
        -------
        list
            Path containing a mix of real node-ids (str) and temp-node tuples:
            ('temp_<base>_<k>', (x, y))
        """

        # ------------------------------------------------------------------
        # 0. Sanity checks
        # ------------------------------------------------------------------
        if current_node not in original_path:
            self.get_logger().warn(f"[Workflow] Node {current_node} is not in the given path.")
            return []

        cur_idx = original_path.index(current_node)
        if cur_idx + 1 >= len(original_path):
            self.get_logger().warn(f"[Workflow] No next node found after {current_node}.")
            return []

        # ------------------------------------------------------------------
        # 1. Generate temporary deviation nodes
        # ------------------------------------------------------------------
        next_nodes = original_path[cur_idx + 1 : cur_idx + node_to_split]
        temp_nodes = []                       # [('temp_id', (x, y)), …]

        for k, node in enumerate(next_nodes, start=1):
            if node not in self.G_loaded.nodes:
                self.get_logger().warn(f"[Workflow] Node {node} not found in the graph.")
                continue

            # fetch coordinates --------------------------------------------------
            node_pos = self.G_loaded.nodes[node]["position"]
            prev_pos = self.G_loaded.nodes[current_node]["position"]

            # Convert [x,y] stored as list / tuple / string ⇒ tuple[float,float]
            def _as_xy(p):
                if isinstance(p, str):   # "5.3,9.1"
                    x, y = map(float, p.replace('[','').replace(']','').split(','))
                    return (x, y)
                return tuple(p)

            node_x,  node_y  = _as_xy(node_pos)
            prev_x,  prev_y  = _as_xy(prev_pos)

            dx, dy   = node_x - prev_x, node_y - prev_y
            length   = math.hypot(dx, dy)
            if length == 0.0:            # identical points → skip
                current_node = node
                continue

            # perpendicular offset ----------------------------------------------
            perp_x = -dy / length * deviation_distance
            perp_y =  dx / length * deviation_distance

            temp_xy = (node_x + perp_x, node_y + perp_y)
            temp_id = f"temp_{current_node}_{k}"
            temp_nodes.append((temp_id, temp_xy))

            current_node = node      # advance anchor for next segment
            
        print(temp_nodes)
        # ------------------------------------------------------------------
        # 2. Assemble the raw path  (real + temp)
        # ------------------------------------------------------------------
        raw_path = (
            # [original_path[cur_idx]] +
            temp_nodes +
            original_path[cur_idx + node_to_split :-1]
        )

        # ------------------------------------------------------------------
        # 3. Helper – compute coordinates & turning angles for a path
        # ------------------------------------------------------------------
        def _coords_and_angles(path):
            """Return (coords, angles) for mixed path.
            coords[i] => (x, y);
            angles[i] => None for endpoints, else interior turning angle (deg)."""
            coords = []
            for elem in path:
                if isinstance(elem, str):        # real node
                    coords.append(_as_xy(self.G_loaded.nodes[elem]["position"]))
                else:                            # temp tuple
                    coords.append(elem[1])

            angles = [None] * len(coords)
            for i in range(1, len(coords) - 1):
                p0, p1, p2 = coords[i-1], coords[i], coords[i+1]
                v1 = (p1[0]-p0[0], p1[1]-p0[1])
                v2 = (p2[0]-p1[0], p2[1]-p1[1])
                dot = v1[0]*v2[0] + v1[1]*v2[1]
                m1  = math.hypot(*v1)
                m2  = math.hypot(*v2)
                if m1 * m2:
                    cos_a = max(-1.0, min(1.0, dot / (m1*m2)))
                    angles[i] = math.degrees(math.acos(cos_a))
            return coords, angles

        # ------------------------------------------------------------------
        # 4. Prune anomalous temp-nodes (local peaks above threshold)
        # ------------------------------------------------------------------
        pruned_path = list(raw_path)   # make a mutable copy
        changed = True
        while changed:
            changed = False
            coords, ang = _coords_and_angles(pruned_path)

            for i in range(1, len(pruned_path)-1):
                if isinstance(pruned_path[i], tuple):  # only test temp nodes
                    if ang[i] is None or ang[i-1] is None or ang[i+1] is None:
                        continue
                    if ang[i-1] < ang[i] > ang[i+1] and ang[i] > angle_threshold:
                        # remove local-peak temp node
                        del pruned_path[i]
                        changed = True
                        break           # re-start scan after any change

        # ------------------------------------------------------------------
        # 5. Publish the *kept* temp nodes (if any) -------------------------
        kept_temp_nodes = [elem for elem in pruned_path if isinstance(elem, tuple)]
        if kept_temp_nodes:
            msg = String()
            msg.data = json.dumps(kept_temp_nodes)
            self.add_nodes_publisher.publish(msg)
            self.get_logger().info(f"Published {len(kept_temp_nodes)} temp nodes to /add_nodes")

        # ------------------------------------------------------------------
        # 6. Reverse if requested and return
        # ------------------------------------------------------------------
        final_path = ([original_path[-1] + list(reversed(pruned_path)) ]) if reverse_output else (pruned_path + [original_path[-1]])
        return final_path
    
    def find_nearest_connected_node(self, source):
        """
        Given a source that can either be a node identifier or a position (dict/tuple),
        find the nearest connected node in the graph.
        """
        # If source is a dictionary (i.e. position), convert to tuple
        if isinstance(source, dict):
            source_pos = (source.get("x"), source.get("y"))
        # If source is already a tuple, use it directly
        elif isinstance(source, (tuple, list)):
            source_pos = source
        else:
            # Otherwise, assume source is a node id and try to fetch its position
            if source not in self.G_loaded.nodes:
                print(f"[System Manager] Source node {source} not found in the graph.")
                return None
            try:
                source_pos = self.G_loaded.nodes[source].get("pos") or self.G_loaded.nodes[source].get("position")
            except KeyError:
                print(f"[System Manager] Node {source} has no position data.")
                return None

        # If source_pos is stored as a string, convert it to a tuple
        if isinstance(source_pos, str):
            source_pos = eval(source_pos)

        # print(f"[System Manager] Source position in meters: {source_pos}")

        nearest_node = None
        min_distance = float('inf')

        # Iterate over all nodes in the graph to find the nearest connected node.
        for node in self.G_loaded.nodes:
            if node != source and node not in nx.isolates(self.G_loaded):
                try:
                    target_pos = self.G_loaded.nodes[node].get("pos") or self.G_loaded.nodes[node].get("position")
                    if isinstance(target_pos, str):
                        target_pos = eval(target_pos)
                    distance = self.euclidean_distance(source_pos, target_pos)
                    if distance < min_distance:
                        min_distance = distance
                        nearest_node = node
                except KeyError as e:
                    print(f"[System Manager] Node {node} missing position data: {e}")
                    continue  # Skip nodes without position data

        # print(
        #     f"[System Manager] Nearest connected node to source is {nearest_node} with distance {min_distance:.2f} meters.")
        return nearest_node

    def find_dynamic_valid_path_with_turns(self, source, target):
        """
        Find the dynamic valid path between source and target while minimizing turns.
        Uses A* with a custom heuristic that includes the dynamic weight and turn penalty.
        """
        try:
            # Check if source or target are isolated and adjust them
            if source in nx.isolates(self.G_loaded):
                nearest_source = self.find_nearest_connected_node(source)
                if nearest_source:
                    source = nearest_source

            final_target = None
            if target in nx.isolates(self.G_loaded):
                nearest_target = self.find_nearest_connected_node(target)
                if nearest_target:
                    final_target = target
                    target = nearest_target

            # A* with custom heuristic
            # path = nx.astar_path(
            #     self.G_loaded,
            #     source=source,
            #     target=target,
            #     heuristic=lambda u, goal: self.heuristic_with_turn_penalty(u, goal),
            #     weight=lambda u, v, d: self.get_dynamic_edge_weight(u, v, d)
            # )
            path = nx.shortest_path(
                self.G_loaded,
                source=source,
                target=target
            )

            if final_target:
                path.append(final_target)

            # After the path is found, assign decreasing weights to edges
            self.assign_decreasing_edge_weights(path)

            print(f"[System Manager] Found dynamic path with turn penalties: {path}")
            return path

        except nx.NetworkXNoPath as e:
            print(f"[System Manager] No path found: {e}")
            return []
    
    def assign_decreasing_edge_weights(self, path):
        """
        Update the graph edges along the given path with decreasing weights.
        The first edge from the robot gets the highest weight.
        """
        num_edges = len(path) - 1
        if num_edges <= 0:
            return

        for i in range(num_edges):
            u = path[i]
            v = path[i + 1]
            new_weight = num_edges - i
            if self.G_loaded.has_edge(u, v):
                self.G_loaded[u][v]['distance'] = new_weight
                # print(f"[System Manager] Updated edge ({u}, {v}) weight to {new_weight}.")

    def publish_rectangle(self, cx, cy, left, right, top, bottom, angle_rad):
        # Rectangle parameters
        # cx, cy = 0.0, 0.0          # Center
        # left, right = 2.0, 4.0     # Asymmetric horizontal sides
        # top, bottom = 1.0, 2.0     # Asymmetric vertical sides
        # angle_deg = 30.0           # Rotation angle
        # angle_rad = math.radians(angle_deg)

        # Rectangle corners relative to center (before rotation)
        corners = [
            (-left, -bottom),
            ( right, -bottom),
            ( right,  top),
            (-left,  top),
            (-left, -bottom),  # close the rectangle
        ]

        # Rotate and translate corners
        rotated_corners = []
        for x, y in corners:
            x_rot = x * math.cos(angle_rad) - y * math.sin(angle_rad)
            y_rot = x * math.sin(angle_rad) + y * math.cos(angle_rad)
            rotated_corners.append((x_rot + cx, y_rot + cy))

        # Create Marker
        marker = Marker()
        marker.header.frame_id = "map"  # or "base_link", "odom", etc.
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rectangle"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set pose (identity)
        marker.pose.orientation.w = 1.0

        # Set scale (line width)
        marker.scale.x = 0.05

        # Color (green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add rotated points
        for x, y in rotated_corners:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self.publisher_u.publish(marker)

    def check_point_in_asymmetric_rectangle(self, x, y, cx, cy, width, height, theta):
        # theta = np.radians(angle_deg)
        left, right = (width/2,)*2 if isinstance(width, (float, int)) else width
        top, bottom = (height/2,)*2 if isinstance(height, (float, int)) else height
        # Translate point to rectangle's center
        x_prime = x - cx
        y_prime = y - cy

        # Rotate the point by -theta (opposite of rectangle rotation)
        x_rot = x_prime * np.cos(-theta) - y_prime * np.sin(-theta)
        y_rot = x_prime * np.sin(-theta) + y_prime * np.cos(-theta)

        if -left < x_rot < right and -bottom < y_rot < top:
            return True
        elif (np.isclose(x_rot, -left) or np.isclose(x_rot, right) or
            np.isclose(y_rot, -bottom) or np.isclose(y_rot, top)):
            return True
        else:
            return False

    def update_remaining_path_in_nodes(self,robot_x,robot_y, pose_z, pose_w):
        """
        This function updates the list of remaining path nodes as the robot moves.
        """
        if not self.path_in_nodes:
            return  # No path available
        
        # if self.path_in_nodes[1] == self.buffer_id and self.euclidean_distance(self.dock_station_end_line_dict[self.buffer_id][:2], (robot_x, robot_y)) > 0.1:
        #     self.get_logger().info("Buffer node reached, skipping path update.")
        #     return
            
        if len(self.path_in_nodes) < 3:
            self.publish_remaining_path()
            return
        # Set a distance threshold to determine when a node is reached
        distance_threshold = 0.9 #1.0# meters (can be adjusted based on your needs)

        # Check if any of the upcoming nodes are temporary (e.g. their id starts with "temp").
        local_path_active = any(
            isinstance(node, tuple) and str(node[0]).startswith("temp")
            for node in self.path_in_nodes
        ) or 0 < self.buffer_action_phase <= 0xF and not self.terminate # (self.on_buffer and ((self.buffer_id in self.path_in_nodes) or (not self.buffer_id in self.path_in_nodes and not self.is_halted))) # 0 < self.buffer_action_phase <= 0x7 # vnot (self.buffer_action_phase and self.buffer_action_phase != 0xF) 
        try:
            print(f"Check: {0 < self.buffer_action_phase <= 0xF and not self.terminate}, Terminale {not self.terminate}, On Buffer: {self.on_buffer}, Local path active: {local_path_active}, buffer id: {self.buffer_id}, Is Halt: {self.is_halted}, Path in Nodes: {self.path_in_nodes}", file=open("/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/local_path_active.log", "a"))
        except Exception as e:
            print(f"Error writing to log file: {e}")        
        self.publish_local_path_active(local_path_active)

           # ---------------------------------------------------------
        # Find the *earliest* node that is currently within reach.
        # ---------------------------------------------------------
        reached_index = None                      # index of first reachable node
        for idx, node in enumerate(self.path_in_nodes):
            # Get node coordinates
            if isinstance(node, tuple) and str(node[0]).startswith("temp"):
                node_x, node_y = node[1]          # (x, y) stored in the tuple
            else:
                node_x, node_y = self.get_node_position(node)
    
            distance_to_node = math.hypot(robot_x - node_x, robot_y - node_y)
    
            # if distance_to_node <= distance_threshold:
            #     reached_index = idx               # this is the first reachable node
            #     break
            if self.check_point_in_asymmetric_rectangle(node_x, node_y, robot_x, robot_y, (2.0, 2.0), (0.4, 2.6), self.quaternion_to_euler(pose_w, 0, 0, pose_z)[2]-math.pi/2):
                reached_index = idx
                break
        self.publish_rectangle(robot_x, robot_y, 2.0, 2.0, 0.4, 2.6, self.quaternion_to_euler(pose_w, 0, 0, pose_z)[2]-math.pi/2)
            
        
        # ---------------------------------------------------------
        # Remove everything up to (and including) the reached node.
        # ---------------------------------------------------------
        if reached_index is not None:
            self.get_logger().info(
                f"Node {self.path_in_nodes[reached_index]} reached "
                f"(skipped {reached_index} earlier node(s))."
            )

            new_path = self.path_in_nodes[reached_index + 1 :]
            if len(new_path) < 2:
                new_path = self.path_in_nodes[-2:]
            self.path_in_nodes = new_path
        self.publish_remaining_path()


    def publish_remaining_path(self):
        remaining_path_str = ",".join(json.dumps(item) for item in self.path_in_nodes)
        self.logging_publisher.publish(String(data=remaining_path_str))
        self.get_logger().info(f"Remaining path in nodes: {remaining_path_str}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return [qx, qy, qz, qw]

    def calculate_curvature(self, lookahead):
        """
        lookahead is in meters
        :param lookahead:
        :return:
        """
        segmented_path = self.path[self.path_index:self.path_index + int(lookahead * 100)]

        if len(segmented_path) < 3:
            print("Not enough points in segmented_path for curvature calculation")
            return 0.01

        # Divide all elements in path by 40
        scaled_segmented_path = np.array(segmented_path)
        scaled_segmented_path.tolist()

        return self.calculate_curvature_finite_diff(scaled_segmented_path)
    
    def find_path_and_angles_between_points_for_conflict(self, G,path):

        def get_position(node):
            """
            Return the (x, y) position in the screen coordinate frame
            for either a normal graph node or a (temp_name, (x, y)) local node.
            """
            if isinstance(node, str):
                # gx, gy = G.nodes[node]['position'] or G.nodes[node]["pos"]
                # if node in G.nodes:
                    # Normal graph node
                return self.get_node_position(node)

                # else:
                #     raise ValueError(f"Node '{node}' not found in graph or movement locations!")
            
                # return (gx,gy)
            elif isinstance(node, tuple) and len(node) == 2:
                # Local path node: e.g. ("temp_68_1", (126.82, 34.69))
                name, coords = node
                x_coord, y_coord = coords
                return (x_coord, y_coord)
            else:
                raise ValueError(f"Unsupported node format: {node}")

        # Build the coordinate list
        path_coords = [get_position(node) for node in path]
        # path_coords=[self.get_node_position(node) for node in path]

        # Now that we have x,y for each node in path_coords,
        # we can do your existing angle computations:
        path_coords_with_quaternions = []
        for i in range(len(path_coords) - 1):
            start = path_coords[i]
            end = path_coords[i + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            angle_rad = math.atan2(dy, dx)
            quaternion = self.euler_to_quaternion(0, 0, angle_rad)
            path_coords_with_quaternions.append((start, quaternion[-2:]))

        # Append the last point with the last quaternion
        if path_coords_with_quaternions:
            last_quaternion = path_coords_with_quaternions[-1][1]
            path_coords_with_quaternions.append((path_coords[-1], last_quaternion))

        # Convert structure to [x, y, qz, qw, ... ] or whatever you need
        l = []
        for (coord, quaternion) in path_coords_with_quaternions:
            l.append(list(coord) + list(quaternion))
        return l
    
    def handle_local_path(self,local_path):
        if not local_path:
            self.logger.warning("Received an empty path.")
            return

        print(f"Received local path with {len(local_path)} nodes: {local_path}")

        # Separate string nodes and temporary points
        parsed_path = []
        for node in local_path:
            if isinstance(node, tuple) and len(node) == 2:  # Temporary point
                parsed_path.append(node[0])
            elif isinstance(node, str):  # Regular node in the graph
                parsed_path.append(node)

        print(f"Parsed path for spline generation: {parsed_path}")

        
        path_with_quaternions = self.find_path_and_angles_between_points_for_conflict(self.G_loaded,local_path)
        # path_coords_with_quaternions = self.filter_points_within_radius(local_path[0], path_coords_with_quaternions, 0.25)
        # path_coords_with_quaternions = self.filter_points_within_radius(local_path[-1], path_coords_with_quaternions, 0.10)
        
        return path_with_quaternions

    def handle_conflict(self):
        if self.conflict_action == "halt":
            self.send_command(0.0, 0.0)
            self.is_halted = True
            self.conflict_action = None
        elif self.conflict_action == "continue to move":
            self.conflict_action = None
        elif self.conflict_action == "node split":
            if len(self.path_in_nodes)<=2:
                self.conflict_action = None
                return
            self.send_command(0.0, 0.0)
            # time.sleep(2)
            self.path_index = 0
            self.path_received = False
            self.goal_reached = False
            self.path_in_nodes_received = False
            # self.is_aligned = False
            self.get_logger().info("Node split detected. Generating new local path...")
            # Generate the local (temporary) path:
            self.generate_local_path()
            self.conflict_action = None        
        elif self.conflict_action == "resume":
            self.is_halted = False
            self.conflict_action = None
            self.get_logger().info("Resuming robot after resuming resume command...")
            
    def handle_lane_action(self):
        if self.lane_action == "buffer":
            if self.buffer_id is None or self.local_path_active:
                print("ret buf", open('/home/fbots/bopt_v2_ws/nm.log', 'a+'))
                self.get_logger().error("Buffer ID is None, cannot send robot to buffer.")
                return
            self.path_index = 0
            self.path_received = False
            self.goal_reached = False
            self.path_in_nodes_received = False
            self.get_logger().info("Buffer action detected... Sending Robot to buffer...")
            self.send_robot_to_buffer_local_path(self.buffer_id, 'BE')
            self.lane_action = None
        elif self.lane_action == "redirect":
            has_node_split = any(
                isinstance(node, tuple) and str(node[0]).startswith("temp")
                for node in self.path_in_nodes
            )
            # if self.redirect_dot_prod is None:
            #     current_pose = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
            #     next_buffer_node_pose = self.get_node_position(self.clean_path(self.path_in_nodes[:-1])[0])
            #     dest_path = self.find_dynamic_valid_path_with_turns(source=self.find_nearest_connected_node(current_pose), target=self.second_leg[-2])
            #     next_dest_node_pose = self.get_node_position(dest_path[1] if len(dest_path) > 1 else (dest_path[0] or next_buffer_node_pose))
            #     dest_vec = (next_buffer_node_pose[0]-current_pose[0], next_buffer_node_pose[1]-current_pose[1])
            #     buff_vec = (next_dest_node_pose[0]-current_pose[0], next_dest_node_pose[1]-current_pose[1])
            #     dest_vec_mod = math.hypot(*dest_vec)
            #     buff_vec_mod = math.hypot(*buff_vec)
            #     dest_vec = (dest_vec[0]/dest_vec_mod, dest_vec[1]/dest_vec_mod)
            #     buff_vec = (buff_vec[0]/buff_vec_mod, buff_vec[1]/buff_vec_mod)
            #     self.redirect_dot_prod = np.dot(dest_vec, buff_vec)
            if self.buffer_action_phase in [0x1, 0x3] or has_node_split:
                self.get_logger().error("Buffer ID is None, cannot send robot to buffer.")
                return
            
            self.path_index = 0
            self.path_received = False
            self.goal_reached = False
            self.path_in_nodes_received = False
            self.get_logger().info("Buffer action detected... Sending Robot to buffer...")
            self.redirect = True
            self.send_robot_redirect_path()
            self.redirect_dot_prod = None
            self.lane_action = None
            
    
    def clean_path(self,raw):
        cleaned = []
        for node in raw:
            try:
                istup0 = isinstance(ast.literal_eval(node), list)
            except:
                istup0 = isinstance(node, list)
            if istup0:
                try:
                    node0 = ast.literal_eval(node)
                except:
                    node0 = node
                if node0[0].startswith("temp"):
                    s = node0[0]
                    parts = s.split("_")
                    nearest_node = parts[1]
                    cleaned.append(nearest_node)
            else:
                cleaned.append(node)
        return cleaned

    def send_robot_redirect_path(self):

        state = self.current_pose
        if state is None:
            print("No state===============")
            return
        
        current_pose = (state.pose.position.x, state.pose.position.y, state.pose.orientation.z, state.pose.orientation.w)
        # current_node = self.find_nearest_graph_node(current_pose[:2])
        current_node = self.clean_path([self.path_in_nodes[0]])[0]
        position_current_node = self.get_node_position(current_node)
        print(current_node)
        if not current_node:
            self.get_logger().warn("No current node found returning from this statement")
            return
        
        if not self.second_leg:
            self.get_logger().warn("Can not find final destination to create redirect path!")
            return

        # 2) get the original final destination
        final_dest = self.second_leg[-1]
        buff_dest = self.first_leg[-1]
        # ra = self.quaternion_to_euler(current_pose[-1], 0, 0, current_pose[-2])[-1]
        # rv = np.cos(ra), np.sin(ra)
        # ledist = np.hypot(self.left_easy_dock_dict[final_dest][0] - current_pose[0], self.left_easy_dock_dict[final_dest][1] - current_pose[1])
        # redist = np.hypot(self.right_easy_dock_dict[final_dest][0] - current_pose[0], self.right_easy_dock_dict[final_dest][1] - current_pose[1])
        # edloc = self.left_easy_dock_dict[final_dest] if ledist <= redist else self.right_easy_dock_dict[final_dest]
        # ea = self.quaternion_to_euler(edloc[-1], 0, 0, edloc[-2])[-1]
        # ev = np.cos(ra), np.sin(ea)

        
        # dest_location_nearest_node = self.find_nearest_connected_node(self.dock_location_dict[final_dest][:2])
        # print('current_node:', current_node, 'self.second_leg[-2]:', self.second_leg[-2], file=open('/home/fbots/bopt_v2_ws/nm.log', 'a+'))
        node_path = self.find_dynamic_valid_path_with_turns(source=current_node, target=self.second_leg[-2]) + [final_dest]
        # print('node_path:', node_path, file=open('/home/fbots/bopt_v2_ws/nm.log', 'a+'))
        path_with_quaternions = self.find_path_and_angles_between_points_for_conflict(self.G_loaded, node_path[:-1])
        if self.calculate_path_distance(path_with_quaternions, 3.0) > 2.0:
            path_with_quaternions = self.filter_points_within_radius(self.dock_location_dict[final_dest][:2], path_with_quaternions, 2.0)
        if self.calculate_path_distance(path_with_quaternions, 3.0) > 2.0:
            path_with_quaternions = self.filter_points_within_radius(position_current_node, path_with_quaternions, 2.5)
        if not path_with_quaternions:
            path_with_quaternions.append(self.dock_location_dict[final_dest])
        
        end_easy_dock = self.find_left_or_right_dock(self.left_easy_dock_dict[final_dest], self.right_easy_dock_dict[final_dest], self.dock_location_dict[final_dest], path_with_quaternions[-1], is_last_pose=True)
        path_with_quaternions.append(end_easy_dock)

        edloc = self.find_left_or_right_dock(self.left_easy_dock_dict[buff_dest], self.right_easy_dock_dict[buff_dest], self.dock_location_dict[buff_dest], current_pose, True)
        path_with_quaternions = [edloc] + path_with_quaternions

        self.path = self.generate_spline_path(current_pose[:2], path_with_quaternions)
        self.path_in_nodes = node_path
        self.destination = final_dest
        with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path_in_nodes.pkl', 'wb') as file:
            pickle.dump(node_path, file)
        with open('/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path.pkl', 'wb') as file:
                pickle.dump(self.path, file)
        self.path_index = 0
        self.path_received = True
        self.path_in_nodes_received = True
        self.final_point = self.path[-1]
        self.path_last_point = self.path[-1]
        self.buffer_action_phase = 0
        self.publish_path_for_visualization()
        self.update_remaining_path_in_nodes(*current_pose)


    def get_dynamic_edge_weight(self, u, v, d):
        base_weight = d.get("distance", 0)
        penalty = 0
        node_v_pos = self.G_loaded.nodes[v].get("position") or self.G_loaded.nodes[v].get("pos")
        if node_v_pos:
            for status in self.robot_statuses.values():
                position = status.get("position")
                if position:
                    x = position.get("x")
                    y = position.get("y")
                    if x is not None and y is not None:
                        robot_pos = (x, y)
                        if self.euclidean_distance(robot_pos, node_v_pos) < 5:
                            penalty += 10
        return base_weight + penalty

    def heuristic_with_turn_penalty(self, u, goal):
        """
        Custom heuristic that considers Euclidean distance and turn penalty.
        u is the current node, goal is the target node.
        """
        pos_u = self.G_loaded.nodes[u].get("position") or self.G_loaded.nodes[u].get("pos")
        pos_goal = self.G_loaded.nodes[goal].get("position") or self.G_loaded.nodes[goal].get("pos")

        if pos_u and pos_goal:
            distance = self.euclidean_distance(pos_u, pos_goal)
        else:
            distance = float('inf')


        neighbors = list(self.G_loaded.neighbors(u))
        prev_node = None
        next_node = None

        if neighbors:
            if len(neighbors) > 1:
                next_node = neighbors[1]
                prev_node = neighbors[0]
            else:
                next_node = neighbors[0]  # Single neighbor case (perhaps linear movement)

        # If we have valid previous and next nodes, calculate the turn penalty
        turn_penalty = 0
        if prev_node and next_node:
            turn_angle = self.calculate_turn_angle(prev_node, u, next_node)
            turn_penalty = 0*turn_angle  # Adjust the weight of the turn penalty

        # print("Turn penalty:", turn_penalty)

        return distance + turn_penalty


    def follow_path(self):

        if not self.data_updated:
            self.get_logger().info("Data not updated, skipping follow_path execution.")
            return

        if self.is_paused:
            if not self.has_send_stop_command:
                self.stop_robot()
                self.has_send_stop_command = True
            return

        if self.conflict_action: self.handle_conflict()
        if self.redirect:
            self.redirect = False
        if self.lane_action:
            self.handle_lane_action()


        state = self.current_pose
        if state is None:
            return

        if self.goal_reached or not self.path_received or self.pallet_detected:
            self.send_stop_command(0.0, 0.0)  # Stop the robot
            if self.buffer_action_phase:
                phases = {0x1: 'BDI', 0x3: 'BL', 0x7: 'BDO', 0xF: 'DE'}
                # if self.buffer_action_phase == 0x3 and self.lane_action == 'redirect':
                #     self.logging_publisher.publish(String(data=''))
                self.send_robot_to_buffer_local_path(self.buffer_id, phases[self.buffer_action_phase])
                robot_x = state.pose.position.x  # Convert back to pixels
                robot_y = state.pose.position.y  # Convert back to pixels
                robot_oz = state.pose.orientation.z
                robot_ow = state.pose.orientation.w
                self.update_remaining_path_in_nodes(robot_x,robot_y, robot_oz, robot_ow)
                
                self.goal_reached = False
                return
            self.finished = True
            return

        robot_x = state.pose.position.x  # Convert back to pixels
        robot_y = state.pose.position.y  # Convert back to pixels
        robot_oz = state.pose.orientation.z
        robot_ow = state.pose.orientation.w
        robot_orientation = self.get_yaw_from_pose(state)
        robot_state = [robot_x, robot_y, robot_orientation]
        target_point_on_plan = self.get_target_point(robot_x, robot_y)

        ttp = self.transform_point(target_point_on_plan, robot_x, robot_y, robot_orientation)
        self.update_remaining_path_in_nodes(robot_x,robot_y, robot_oz, robot_ow)

        if self.is_halted:
            self.send_command(0.0, 0.0)
            return

        
        
        if self.local_path_active:
            self.max_velocity=1.42 #0.7
        else:
            self.max_velocity=self._max_vel
        s, v, d = self.lookup_table.get(self.find_nearest_key(ttp))
        velocity, steering_angle = (self.max_velocity / 0.2) * v, math.degrees(s)
        

        # self.steering_angles.append(steering_angle)  # Store the steering angle

        distance_to_goal = math.hypot(self.final_point[0] - robot_x, self.final_point[1] - robot_y)
        self.get_logger().info(f'DTG: {distance_to_goal}')
        if distance_to_goal <= self.goal_tolerance:
            self.goal_reached = True
            self.send_stop_command(0.0, 0.0)  # Stop the robot
            self.get_logger().info('Goal reached, shutting down...')
            # self.destroy_node()
            # rclpy.shutdown()
            # sys.exit(0)
            return
        distance_to_lp = math.hypot(self.path_last_point[0] - robot_x, self.path_last_point[1] - robot_y)
        # self.get_logger().info(f'DTG: {distance_to_goal}')
        if distance_to_lp <= self.goal_tolerance:
            self.goal_reached = True
            self.send_stop_command(0.0, 0.0)  # Stop the robot
            self.get_logger().info('Goal reached, shutting down...')
            # self.destroy_node()
            # rclpy.shutdown()
            # sys.exit(0)
            return
        

        # Apply the S-curve velocity smoother
        self.current_velocity = self.apply_s_curve_velocity_smoother(self.current_velocity, velocity)

        # Ensure the velocity is within safe limits based on the steering angle
        safe_velocity = self.calculate_safe_velocity(self.current_velocity ,steering_angle)
        # safe_velocity = self.current_velocity
        if self.current_velocity < 0:
            self.current_velocity = -min(abs(self.current_velocity), abs(safe_velocity))
        else:
            self.current_velocity = min(abs(self.current_velocity), abs(safe_velocity))

        # print(self.current_velocity, '=====')


        
        # Calculate the target velocity based on the distance to the goal
        if distance_to_goal <= self.slow_down_distance:
            target_velocity = (distance_to_goal / (self.slow_down_distance)) * self.stopping_velocity
            if velocity < 0:
                self.current_velocity = -max(self.min_velocity, abs(target_velocity))
            else:
                self.current_velocity = max(self.min_velocity, target_velocity)
            
        
        # if self.local_path_active:
        #     print("local path active")
        #     self.current_velocity/=2
       

        self.send_command(self.current_velocity, steering_angle)
        self.publish_target_point_marker(target_point_on_plan)

    def apply_s_curve_velocity_smoother(self, current_velocity, target_velocity, transition_duration=0.58):
        if current_velocity == target_velocity:
            return current_velocity

        delta_velocity = target_velocity - current_velocity
        time_step = 0.05  # Assuming each follow_path call is approximately 50ms apart
        progress = min(time_step / transition_duration, 1.0)
        s_curve_factor = (progress ** 3) * (10 - 15 * progress + 6 * (progress ** 2))

        return current_velocity + delta_velocity * s_curve_factor

    def publish_target_point_marker(self, target_point):
        """Publish the target point as a marker for visualization in RViz2."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker to the target point
        marker.pose.position.x = target_point[0]
        marker.pose.position.y = target_point[1]
        marker.pose.position.z = 0.0  # Assuming 2D path
        marker.pose.orientation.w = 1.0  # No rotation

        # Set the scale of the marker (radius)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color of the marker (RGBA)
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        self.target_point_publisher.publish(marker)
        self.get_logger().info(
            f'Target point published at ({target_point[0]}, {target_point[1]}) for visualization in RViz2.')

    def calculate_safe_velocity(self, cv, steering_angle):
        # Calculate the safe velocity based on the steering angle
        mu = 1.0  # Coefficient of friction
        g = 9.81  # Acceleration due to gravity (m/s^2)
        L = 1.36  # Wheelbase (m)

        # Turn radius based on steering angle
        # turn_radius = abs(L / np.tan(steering_angle + 1e-6))  # Adding a small value to avoid division by zero
        # print('tr', turn_radius)
        curvature = self.calculate_curvature(1.2)
        print(abs(curvature), 'curvature=====')        

        # Maximum safe speed calculation
        
        safe_velocity = 1.0 * cv * (1 / (1 + np.exp(-(0.9 * np.sqrt(mu * (g / abs(curvature)))))))
        return safe_velocity  # Scale the velocity to match the units used in your system

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def conflict_action_callback(self, msg):
        sys_action = msg.data.strip().lower()
        if sys_action in ['redirect', 'buffer']:
            self.lane_action = sys_action
        self.conflict_action = sys_action
        self.get_logger().info(str(msg))


    def task_action_callback(self, msg):
        data =msg.data.strip().split("/")[1]
        if data=="pause_task":
            self.is_paused=True
            self.has_send_stop_command = False
        elif data=="resume_task":
            cp = (self.current_pose.pose.position.x, self.current_pose.pose.position.y)
            d, idx = math.hypot(self.path[self.path_index][0]-cp[0], self.path[self.path_index][1]-cp[1]), self.path_index
            for i, path in enumerate(self.path):
                m = math.hypot(path[0]-cp[0], path[1]-cp[1])
                if m < d:
                    d = m; idx = i
                    print("index:", idx ,file=open("/home/fbots/nm.log", "a+"))
            print("1. path_index:", self.path_index, ", idx:", idx,file=open("/home/fbots/nm.log", "a+"))
            if idx != self.path_index: self.path_index = idx
            print("2. path_index:", self.path_index, ", idx:", idx,file=open("/home/fbots/nm.log", "a+"))
            self.is_paused=False
            self.has_send_stop_command = False
        self.get_logger().info("abc: "+str(msg.data))
        
    def current_pose_callback(self, msg):
        self.current_pose = msg

    def pallet_detection_callback(self, msg):
        self.pallet_detected = msg.data
        self.get_logger().info(str(msg))

    def stop_robot(self):
        """Send stop command to the robot and log the action."""
        self.send_stop_command(0.0, 0.0)  # Stop the robot
        self.get_logger().info('Stopping the robot...')

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


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='RobotClient Node')
    parser.add_argument('--path_file', type=str, required=True, help='Path to the file containing the path data')
    parser.add_argument('--goal_tolerance', type=float, default=0.1, help='Goal tolerance distance')
    parser.add_argument(
        '--config-file', '-c',
        default='',
        help='Full path to amr_config.yaml (overrides workspace root file)'
    )
    parsed_args = parser.parse_args()
    # args, ros_cli_args = parser.parse_known_args()
    defaults = {
        "robot_id": "machine_X",
        "linear_velocity_publisher_topic": "/velocity",
        "steering_angle_publisher_topic": "/steering_angle",
        "state_publisher_topic": "/state",
        "min_velocity": 0.2,
        "max_velocity": 1.42,
        "slow_down_distance": 2.0,
        "velocity_smoothing_factor": 0.01,
        "stopping_velocity": 0.3,
        "goal_tolerance": 0.1,
        "path_file": "/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path.pkl",
        "graphml_file": '/home/fbots/bopt_v2_ws/src/workflow_node/map_details/testing_ground_waypoints.graphml',
        "path_file_in_nodes": "/home/fbots/bopt_v2_ws/src/workflow_node/workflow_node/constructed_rs_path_in_nodes.pkl",
        "speed_control_lookahead": 1.0,
        "path_tracking_lookahead": 0.2
    }
    config_keys = [
        'nmpc_controller',
        'nmpc_controller_v2_fast',
    ]

    params = load_node_params(defaults, config_keys, parsed_args.config_file)

    client = RobotClient(params)
    client.set_parameters(
        path_file=parsed_args.path_file,
        goal_tolerance=parsed_args.goal_tolerance
    )

    def signal_handler(sig, frame):
        client.get_logger().info('Signal received, shutting down...')
        client.stop_robot()
        client.destroy_node()
        rclpy.shutdown()

    # Register the signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)

    executor = MultiThreadedExecutor()  # One thread per callback
    executor.add_node(client)
    try:
        while rclpy.ok() and not client.finished:
            executor.spin_once()
            # rclpy.spin_once(client)
        # rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard interrupt received, shutting down...')
        client.stop_robot()
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
