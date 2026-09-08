import random
import subprocess
import signal
import sys
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
import json
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import psutil
import sqlite3
import os
import pickle
import numpy as np
import math
import datetime
from threading import Thread
from rclpy.clock import ROSClock
from tf2_ros import TransformListener, Buffer
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import re
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
import os
import random
import subprocess
import time
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
import json
import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import psutil
import sqlite3
import os
import pickle
import numpy as np
import math
from threading import Thread, Lock
from rclpy.clock import ROSClock
from tf2_ros import TransformListener, Buffer
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from collections import deque
import psutil
import socket
import can
import networkx as nx


ws_path = os.getenv('WS_PATH')
databasepath = '/home/byd3/bopt_ws/src/task_allocator/Vehicles.db'  # os.getenv('DATABASE_PATH_VEH')
databasepathuser = os.getenv('DATABASE_PATH_USERS')
bt_path = os.getenv('BT_PATH')


# Define V2 base paths
base_path = '/home/byd3/python3.10_env'
waypoints_file = os.path.join(base_path, 'waypoints.json')
python_executable = os.path.join(base_path, 'myenv/bin/python')
deep_pallet_script = os.path.join(base_path, 'deep_pallet_pickup_plan.py')
constructed_rs_path = os.path.join('/home/byd3/bopt_ws/src/load_transporter_node/load_transporter_node', 'constructed_rs_path.pkl')


class SafetyPublisher(Node):
    def __init__(self):
        super().__init__('Task_allocator')
        self.publisher_ = self.create_publisher(String, '/byd/safety', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_publisher = self.create_publisher(String, '/byd/current_task', 10)
        self.state_publisher = self.create_publisher(String, '/byd/status', 10)
        print("publishers_created")


class LoadTransporter(Node):
    def __init__(self, source=None, destination=None, navigator=None):
        super().__init__('load_transporter')

        self.ltn_feedback_publisher = self.create_publisher(String, '/load_transporter_feedback', 10)
        self.ltn_task_publisher = self.create_publisher(String, '/load_transporter_current_task', 10)
        self.ltn_current_state_publisher = self.create_publisher(String, '/load_transporter_current_state', 10)

        SERVER_HOST = '192.168.68.60'  # Replace with your server's IP address
        SERVER_PORT = 12345
        self.source_name = source
        self.destination_name = destination
        if self.source_name and self.destination_name:
            pub_str = (f"Source set to {self.source_name}, destination set to {self.destination_name}")
            # self.feedback_publisher(pub_str)  # Assuming feedback_publisher is a method to publish this info

        self.databasepath = databasepath
        self.next_task = None
        self.station_locations, self.default_easy_station_locations, self.default_flow_matrix, self.dock_station_locations, self.rev_flow_matrix, self.rev_easy_station_location, self.flow_matrix_park, self.pick_flow, self.rev_pick_flow = self.get_locations_and_waypoints()
        self.flow_matrix = self.default_flow_matrix
        self.easy_station_locations = self.default_easy_station_locations

        self.navigator = navigator
        publisher = SafetyPublisher()
        self.publisher_ = publisher.publisher_  # safety pub
        self.cmd_vel_publ = publisher.cmd_vel_pub
        self.t_publisher = publisher.task_publisher
        self.state_pub = publisher.state_publisher
        self.sequence_complete = False
        self.bt_path = bt_path
        self.pallet_picked = False
        self.current_pose = None
        self.success = False
        self.pallet_manager_lift = None

        self.pallet_manager_drop = None

        self.current_state_of_task = 0

        self.thread2 = Thread(target=self.field_back, )
        self.next_task_source = None


        self.col_end_points = {
            'L1': [5.1383112282655645, 9.614453833006769, 0.9999790643824872, -0.006470764771313809],
            'L2': [6.483013715241465, 0.8472426834756197, -0.69890328475984, 0.7152161900795493]
        }
        self.dock_station_locations = {
            'L1': [7.998071727308294, 9.651465832613393, 0.9999790643824872, -0.006470764771313809],
            'L2': [6.417038099524563, 3.706481606358906, -0.69890328475984, 0.7152161900795493]
        }
        self.right_easy_dock_locations = {
            'L1': [8.748172727158448, 10.421237545541194, -0.7025164558201769, 0.7116674991186949],
            'L2': [5.639708361994442, 4.448747380291231, 0.01153496597242057, 0.9999334700668916]
        }
        self.left_easy_dock_locations = {
            'L1': [8.767843440236094, 8.90136483276324, -0.7116674991186948, -0.702516455820177],
            'L2': [7.159303873456889, 4.483811343889026, -0.9999334700668916, 0.011534965972420631]
        }
                               


        # # Set source related locations
        self.source_left_dock_pose = self.left_easy_dock_locations[self.source_name]
        self.source_right_dock_pose = self.right_easy_dock_locations[self.source_name]
        self.source_dock_pose = self.dock_station_locations[self.source_name]
        self.source_col_end_pose = self.col_end_points[self.source_name]



        # Set destination related locations
        self.destination_left_dock_pose = self.left_easy_dock_locations[self.destination_name]
        self.destination_right_dock_pose = self.right_easy_dock_locations[self.destination_name]
        self.destination_dock_pose = self.dock_station_locations[self.destination_name]
        self.destination_col_end_pose = self.col_end_points[self.destination_name]

        print('euw')                
        self.G_loaded = self.graph_loader()

    def graph_loader(self):
        G_loaded = nx.read_graphml('/home/byd3/bopt_ws/src/load_transporter_node/load_transporter_node/updated_kasna_graph.graphml')
        for node, data in G_loaded.nodes(data=True):
            pos_str = data['pos']
            G_loaded.nodes[node]['pos'] = tuple(map(float, pos_str.split(',')))

            # Parse orientation
            orientation_str = data['orientation']
            G_loaded.nodes[node]['orientation'] = tuple(map(float, orientation_str.split(',')))

        self.graph = G_loaded
        return G_loaded

    def find_path_and_angles_between_points(self, G, point1, point2):
        def euclidean_distance(coord1, coord2):
            return math.sqrt((coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2)

        def closest_node(G, point):
            min_distance = float('inf')
            closest = None
            for node, data in G.nodes(data=True):
                pos = data['pos']
                distance = euclidean_distance(pos, point)
                if distance < min_distance:
                    min_distance = distance
                    closest = node
            return closest

        node1 = closest_node(G, point1)
        node2 = closest_node(G, point2)

        path = nx.shortest_path(G, source=node1, target=node2)
        path_coords = [G.nodes[node]['pos'] for node in path]
        path_quaternions = [G.nodes[node]['orientation'] for node in path]

        path_coords_with_quaternions = list(zip(path_coords, path_quaternions))

        l = []
        for coord, quaternion in path_coords_with_quaternions:
            l.append(list(coord) + list(quaternion))

        return l

    def send_data(self, message, retries=1, delay=3):
        """
        Attempt to send data to a specified IP and port with retries.

        Args:
        ip (str): The IP address of the host.
        port (int): The port number to connect to.
        message (bytes): The data to send.
        retries (int): Number of retries for the connection.
        delay (int): Delay between retries in seconds.

        Returns:
        str: The response from the server or an error message.
        """
        ip = '192.168.68.60'
        port = 12345
        for attempt in range(retries):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.connect((ip, port))
                    s.sendall(message)
                    data = s.recv(1024).decode()
                    print(f"Received: {data}")
                    return data
            except socket.error as e:
                print(f"Attempt {attempt + 1} failed: {e}")
                time.sleep(delay)  # Delay between retries

        return "Failed to connect after several attempts."

    def compute_positional_difference(self, pose1, pose2):
        return math.sqrt((pose1[0] - pose2[0]) ** 2 +
                         (pose1[1] - pose2[1]) ** 2)

    def angle_to_quaternion(self, angle_rad):
        # Convert angle from degrees to radians
        # angle_rad = math.radians(angle_deg)
        # angle_rad += 3.141592653

        # Calculate the quaternion components
        w = math.cos(angle_rad / 2)
        x = 0  # No rotation around the x-axis
        y = 0  # No rotation around the y-axis
        z = math.sin(angle_rad / 2)  # Rotation around the z-axis

        return (z, w)

    def quaternion_to_angle(self, z, w):
        """Converts a quaternion into an angle in 2D space."""
        return math.degrees(2 * math.atan2(z, w))

    def compute_angular_difference(self, pose1, pose2):
        print(pose1, pose2)

        robot_theta = self.quaternion_to_angle(pose1[2], pose1[3])
        target_theta = self.quaternion_to_angle(pose2[2], pose2[3])

        # Calculate difference in orientation
        delta_theta = robot_theta - target_theta

        # Normalize the result to [0, 360) range
        delta_theta = (delta_theta + 360) % 360

        delta_theta = abs(delta_theta)

        return delta_theta


    

    def v2_control(self, waypoints, type):
        '''

        :param waypoints_to_navigate:
        :param type: slow, fast, deep_pickup, deep_stack
        :return:
        '''
        SCALE = 1
        

        if type == 'fast':
            rs_path_waypoints = []

            for waypoint in waypoints:
                if waypoint == waypoints[-1]:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 1.5 * SCALE,
                                            0.5 * SCALE])
                else:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 1.5 * SCALE,
                                            0.0])

            data = {
                "rs_path_waypoints": rs_path_waypoints
            }

            # Write the data to a JSON file
            with open(waypoints_file, 'w') as file:
                json.dump(data, file, indent=4)

            command = [python_executable, deep_pallet_script]
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_fast",
                                    "--path_file", constructed_rs_path], capture_output=True, text=True)
        elif type == 'fast_rev':
            rs_path_waypoints = []

            for waypoint in waypoints:
                if waypoint == waypoints[-1]:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]), 1.5 * SCALE,
                                            0.5 * SCALE])
                else:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]), 1.5 * SCALE,
                                            0.0])

            data = {
                "rs_path_waypoints": rs_path_waypoints
            }

            # Write the data to a JSON file
            with open(waypoints_file, 'w') as file:
                json.dump(data, file, indent=4)

            command = [python_executable, deep_pallet_script]
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_fast",
                                    "--path_file", constructed_rs_path], capture_output=True, text=True)
        elif type == 'slow':
            print('too fast')
            rs_path_waypoints = []

            for waypoint in waypoints:
                if waypoint == waypoints[-1]:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 0.9 * SCALE,
                                            0.0 * SCALE])
                else:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 0.9 * SCALE,
                                            0.0])

            data = {
                "rs_path_waypoints": rs_path_waypoints
            }

            # Write the data to a JSON file
            with open(waypoints_file, 'w') as file:
                json.dump(data, file, indent=4)

            command = [python_executable, deep_pallet_script]
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_slow",
                                    "--path_file", constructed_rs_path], capture_output=True, text=True)
        elif type == 'dp':
            rs_path_waypoints = []

            for waypoint in waypoints:
                if waypoint == waypoints[-1]:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 0.9 * SCALE,
                                            0.5 * SCALE])
                else:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 1.2 * SCALE,
                                            0.0])

            data = {
                "rs_path_waypoints": rs_path_waypoints
            }

            # Write the data to a JSON file
            with open(waypoints_file, 'w') as file:
                json.dump(data, file, indent=4)

            command = [python_executable, deep_pallet_script]
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_dp",
                                    "--path_file", constructed_rs_path], capture_output=True, text=True)
        elif type == 'dd':
            rs_path_waypoints = []

            for waypoint in waypoints:
                if waypoint == waypoints[-1]:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 0.9 * SCALE,
                                            0.5 * SCALE])
                else:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                            self.quaternion_to_angle_rad(waypoint[2], waypoint[3]) + 3.14, 1.2 * SCALE,
                                            0.0])

            data = {
                "rs_path_waypoints": rs_path_waypoints
            }

            # Write the data to a JSON file
            with open(waypoints_file, 'w') as file:
                json.dump(data, file, indent=4)

            command = [python_executable, deep_pallet_script]
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_dd",
                                    "--path_file", constructed_rs_path], capture_output=True, text=True)

    
    def pp_control(self, lateral_offset):
        print('inside pp')
        SCALE = 1
        rs_path_waypoints = []
        LATERAL_OFFSET = lateral_offset
        BACKUP_DISTANCE = 0.6
        PALLET_LENGTH = 1.3
        current_pose = get_current_pose()
        rx = current_pose[0]
        ry = current_pose[1]
        robotO = self.quaternion_to_angle_rad(current_pose[2], current_pose[3])

        backup = [
            ((rx / SCALE) * SCALE,
                (ry / SCALE) * SCALE,
                robotO,
                0.9 * SCALE, 0 * SCALE),
            

            ((rx / SCALE - math.cos(robotO) * BACKUP_DISTANCE) * SCALE,
                (ry / SCALE - math.sin(robotO) * BACKUP_DISTANCE) * SCALE,
                robotO,
                0.9 * SCALE, 1 * SCALE),  # x, y, orientation, tr, runway
            
        ]

        data = {
            "rs_path_waypoints": backup
        }

        # Write the data to a JSON file
        with open(waypoints_file, 'w') as file:
            json.dump(data, file, indent=4)

        command = [python_executable, deep_pallet_script]
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        print(f"Script output:\n{result.stdout}")

        result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_slow",
                                    "--path_file",
                                    constructed_rs_path
                                    ], capture_output=True, text=True)


        current_pose = get_current_pose()
        rx = current_pose[0]
        ry = current_pose[1]
        robotO = self.quaternion_to_angle_rad(current_pose[2], current_pose[3])

        lateral_pallet_picking = [
            ((rx/ SCALE) * SCALE,
                (ry / SCALE) * SCALE,
                robotO,
                0.3 * SCALE, 0 * SCALE),

            (rx + BACKUP_DISTANCE * math.cos(robotO) - LATERAL_OFFSET * math.sin(robotO),
                ry + BACKUP_DISTANCE * math.sin(robotO) + LATERAL_OFFSET * math.cos(robotO),
                robotO, 0.3 * SCALE, 0 * SCALE),  # x, y, orientation, tr, runway


            ( rx + BACKUP_DISTANCE * math.cos(robotO) - LATERAL_OFFSET * math.sin(robotO) + PALLET_LENGTH * math.cos(robotO),
                ry + BACKUP_DISTANCE * math.sin(robotO) + LATERAL_OFFSET * math.cos(robotO) + PALLET_LENGTH * math.sin(robotO),
                robotO,
            0.3 * SCALE, -1 * SCALE),
        ]

        data = {
            "rs_path_waypoints": lateral_pallet_picking
        }

        # Write the data to a JSON file
        with open(waypoints_file, 'w') as file:
            json.dump(data, file, indent=4)

        command = [python_executable, deep_pallet_script]
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        print(f"Script output:\n{result.stdout}")

        result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_pp",
                                    "--path_file",
                                    constructed_rs_path
                                    ], capture_output=True, text=True)


    def go_to_easy_dock_for_pickup_using_flow(self):
        self.sequence_complete = True

        #Check if at parking location
        self.current_state_of_task = 1
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        # self.send_data(b"Task started")
        # self.send_data(b"1")

        self.pub_fb("Localizing")
        current_formatted_pose = get_current_pose()

        point1 = current_formatted_pose[:2]
        point2 = self.source_dock_pose[:2]

        path_coords_with_quaternions = self.find_path_and_angles_between_points(self.G_loaded, point1, point2)

        self.pub_fb("Navigating through waypoints")

        
        # #find if dock to attach is left or right 
        print('pbd', path_coords_with_quaternions)

        easy_dock = self.find_left_or_right_dock(self.source_left_dock_pose, self.source_right_dock_pose, path_coords_with_quaternions[-1])

        path_coords_with_quaternions.append(easy_dock)
        
        waypoints = []
        waypoints.append(current_formatted_pose)
        waypoints.extend(path_coords_with_quaternions)

        self.v2_control(waypoints, 'fast')

        self.pub_fb('at station dock')

        return True
    
    def find_left_or_right_dock(self, left_dock_pose, right_dock_pose, pose_to_compare):
        def quaternion_to_degrees(z, w):
            return math.degrees(2 * math.atan2(z, w)) % 360  # Ensure the angle is within [0, 360)

        def orientation_difference_deg(q1, q2):
            angle1 = quaternion_to_degrees(q1[0], q1[1])
            angle2 = quaternion_to_degrees(q2[0], q2[1])
            diff = abs(angle1 - angle2) % 360
            return min(diff, 360 - diff)  # Ensure the difference is within [0, 180]

        last_pose = pose_to_compare
        last_orientation = last_pose[2:]

        left_orientation = left_dock_pose[2:]
        right_orientation = right_dock_pose[2:]

        left_diff = orientation_difference_deg(last_orientation, left_orientation)
        right_diff = orientation_difference_deg(last_orientation, right_orientation)

        print(f"Last orientation: {last_orientation}")
        print(f"Left dock orientation: {left_orientation}, Difference: {left_diff}")
        print(f"Right dock orientation: {right_orientation}, Difference: {right_diff}")

        if left_diff < right_diff:
            print("Choosing left dock")
            return left_dock_pose
        else:
            print("Choosing right dock")
            return right_dock_pose

    def align(self, pose):
         subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                        str(pose[0]), str(pose[1]), str(pose[2]), str(pose[3])])

    def precise_control_to_pickup_dock(self):
        self.current_state_of_task = 2
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        # self.send_data(b"2")

        self.thread1 = Thread(target=self.field_change, )  # to change later
        self.thread1.start()
        waypoints = []
        waypoints.append(get_current_pose())
        waypoints.append(self.source_dock_pose)

        self.v2_control(waypoints, 'slow') #return status for error handling

        self.pub_fb("Navigating to Dock Pose")

        self.align(self.source_dock_pose)

        waypoints = []
        waypoints.append(get_current_pose())
        waypoints.append(self.source_col_end_pose)

        self.v2_control(waypoints, 'dp') #return status for error handling
        self.thread2 = Thread(target=self.field_back, )
        # Precise control to dock
        return True

    def quaternion_to_angle_rad(self, z, w):
        return 2 * math.atan2(z, w)

    def go_for_pallet_pickup(self):
        self.current_state_of_task = 3
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=str(self.current_state_of_task), )
        self.current_state_thread.start()
        self.send_data(b"3")
        #to change later
        sta = self.run_drive_for_pallet()#self.source_col_end_pose)
        print(sta)
        if sta == 'False':
            self.pallet_picked = False
            self.pallet_manager_lift.run_drive_back()
            return False
        self.fork_up()

        # switching field on
        self.pallet_picked = True
        return True


    def drive_back_to_flow_after_pickup(self):
        self.current_state_of_task = 4
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"4")


        point1 = self.source_dock_pose[:2]
        point2 = self.destination_dock_pose[:2]

        path_coords_with_quaternions = self.find_path_and_angles_between_points(self.G_loaded, point1, point2)


        waypoints = []

        # if self.source_name == 'P1':
        waypoints.append(get_current_pose())
        waypoints.append(self.source_dock_pose)
        #figure out left or right dock
        easy_dock = self.find_left_or_right_dock(self.source_left_dock_pose, self.source_right_dock_pose, path_coords_with_quaternions[1])
        waypoints.append(easy_dock)

        # waypoints.append(self.source_left_dock_pose)

        self.v2_control(waypoints, 'slow')

        self.thread2 = Thread(target=self.field_back, )
        self.thread2.start()

        return True

    def go_to_drop_off_easy_dock_using_flow(self):
        self.current_state_of_task = 5
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"5")

        # Go to easy dock of drop using flow
       
        self.pub_fb("Navigating through waypoints")

        current_formatted_pose = get_current_pose()

        point1 = current_formatted_pose[:2]
        point2 = self.destination_dock_pose[:2]

        path_coords_with_quaternions = self.find_path_and_angles_between_points(self.G_loaded, point1, point2)

        easy_dock = self.find_left_or_right_dock(self.destination_left_dock_pose, self.destination_right_dock_pose, path_coords_with_quaternions[-1])

        path_coords_with_quaternions.append(easy_dock)
        
        waypoints = []
        waypoints.append(current_formatted_pose)
        waypoints.extend(path_coords_with_quaternions)

        self.v2_control(waypoints, 'fast')


        # self.v2_control(path_coords_with_quaternions, 'fast')

        robot_state = 'at station dock'

        return True

    def precise_control_to_drop_off_dock(self):
        # Precise control to drop-off dock
        self.current_state_of_task = 6
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"6")

        self.thread1 = Thread(target=self.field_change, )  # to change later
        self.thread1.start()
        time.sleep(1)

        waypoints = []
        waypoints.append(get_current_pose())
        waypoints.append(self.destination_dock_pose)

        self.v2_control(waypoints, 'slow') #return status for error handling

        self.align(self.destination_dock_pose)

        waypoints = []
        waypoints.append(get_current_pose())
        waypoints.append(self.destination_col_end_pose)

        self.v2_control(waypoints, 'dd') #return status for error handling


        global task_complete
        self.thread2 = Thread(target=self.field_back, )
        # Precise control to dock

        return True

    def check_pallet_present(self):
        result = subprocess.run(["ros2", "run", "pallet_detection", "pallet_check"], capture_output=True, text=True)
        print(result.stderr)
        che = result.stderr
        res = che.split("\n")
        print(res)
        print("pallet_present----->", res[-2])
        return res[-2]

    def go_for_pallet_drop(self):
        # Go for pallet drop
        self.current_state_of_task = 7
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"7")
        #find better way to check if pallet is present
        res = self.check_pallet_present()
        fun_res = True
        if res == 'True':
            fun_res = False
            self.send_data(b"Pallet_Already_Present")

            return False
        else:
            # result = self.run_drop_pallet()
            # if result == False:
            #     self.send_data(b"Pallet_Already_Present")

            #     return False

            self.fork_down()

        return fun_res
    


    def drive_back_to_flow_after_drop(self):
        # Drive back from drop off
        self.current_state_of_task = 8
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"8")

        # sou = None
        # des = None
        # try:
        #     data_rec = self.send_data(b"Request Info")

        #     if data_rec != "Parking":
        #         sou, des = data_rec.split(" ")
        #         des = self.destination_name
        #         self.next_task_source = sou
        #     elif data_rec == "Parking":
        #         self.next_task = "Parking"
        # except Exception as e:
        #     print(e)

        waypoints = []

        waypoints.append(get_current_pose())
        waypoints.append(self.destination_dock_pose)
        #figure out left or right dock
        waypoints.append(self.destination_left_dock_pose)


        self.v2_control(waypoints, 'slow')
        self.thread2 = Thread(target=self.field_back, )
        self.thread2.start()

        self.sequence_complete = True
        return True


    def task_park(self):

        robot_state = 'localizing and parking'
        
        result_to_poses = self.navigate_through_poses(self.navigator,
                                                      [[11.81, 21.28, -0.71, 0.69], [7.026, 16.08, -0.71, 0.69]])  


    def bopt_current_state_publisher(self, feedback):
        i = 0
        msg = String()
        msg.data = feedback
        print('Publishing: "%s"' % msg.data)
        while i <= 100:
            self.ltn_current_state_publisher.publish(msg)
            i += 1

    def feedback_publisher(self, feedback):
        i = 0
        msg = String()
        msg.data = feedback
        print('Publishing: "%s"' % msg.data)
        while i <= 100:
            self.ltn_feedback_publisher.publish(msg)
            i += 1

    def task_publisher(self, feedback):
        i = 0
        msg = String()
        msg.data = feedback
        print('Publishing: "%s"' % msg.data)
        while i <= 100:
            self.ltn_task_publisher.publish(msg)
            i += 1

    def euclidean_distance(self, pose1, pose2):

        dx = pose1[0] - pose2[0]
        dy = pose1[1] - pose2[1]

        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance
    
    def pub_fb(self, fb_string):
        pub_str = fb_string
        self.feedback_thread = Thread(target=self.feedback_publisher, args=(pub_str,))  # to change later
        self.feedback_thread.start()

    def field_back(self):
        i = 0
        msg = String()
        msg.data = 'lock'
        while i <= 100:
            self.publisher_.publish(msg)
            i += 1


    def field_change(self):
        i = 0
        msg = String()
        msg.data = 'pickdrop'
        while i <= 100:
            self.publisher_.publish(msg)
            i += 1


    
    def reverse_count(self, lst, start, end):
        start_index = lst.index(start)
        path = []

        # Starting from the start index and moving backwards
        i = start_index
        while True:
            path.append(lst[i])
            if lst[i] == end:
                break
            i -= 1  # move to the previous element
            if i < 0:
                i = len(lst) - 1  # if we reach the beginning of the list, wrap around to the end

        return path

    def forward_count(self, lst, start, end):
        start_index = lst.index(start)
        path = []

        # Starting from the start index and moving forwards
        i = start_index
        while True:
            path.append(lst[i])
            if lst[i] == end:
                break
            i += 1  # move to the next element
            if i >= len(lst):
                i = 0  # if we reach the end of the list, wrap around to the beginning

        return path

    

    def get_locations_and_waypoints(self):
        print(os.path.exists(self.databasepath))  # Should return True
        connection = sqlite3.connect(self.databasepath)
        cur = connection.cursor()
        station_locations = {}
        flow_matrix = {}
        flow_matrix_park = {}
        rev_flow_matrix = {}
        easy_station_locations = {}
        easy_station_location_rev = {}
        doc_station_locations = {}
        pick_flow = {}
        rev_pick_flow = {}

        for rows in cur.execute("SELECT * FROM location"):
            station_locations[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
        i = 0
        for rows in cur.execute("SELECT * FROM waypoints"):
            flow_matrix[i] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
            i += 1

        l = 0
        for rows in cur.execute("SELECT * FROM waypoints_pick"):
            pick_flow[l] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
            l += 1

        m = 14
        for rows in cur.execute("SELECT * FROM rev_waypoints_pick"):
            rev_pick_flow[m] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
            m -= 1

        k = 70
        for rows in cur.execute("SELECT * FROM waypoints_park"):
            flow_matrix_park[k] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]

            k -= 1

        j = 73
        for rows in cur.execute("SELECT * FROM waypoints_rev"):
            rev_flow_matrix[j] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]

            j -= 1
        # print("pick_flow---", pick_flow)
        for rows in cur.execute("SELECT * FROM easy_station_loc"):
            easy_station_locations[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
        for rows in cur.execute("SELECT * FROM easy_station_loc_rev"):
            easy_station_location_rev[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]

        for rows in cur.execute("SELECT * FROM doc_station_loc"):
            doc_station_locations[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
        # print(station_locations, easy_station_locations, flow_matrix, doc_station_locations)

        return station_locations, easy_station_locations, flow_matrix, doc_station_locations, rev_flow_matrix, easy_station_location_rev, flow_matrix_park, pick_flow, rev_pick_flow




    def run_no_pallet_found(self):
        current_pose = get_current_pose()
        dist = np.sqrt((self.dock_location[0] - current_pose[0]) ** 2 + (self.dock_location[1] - current_pose[1]) ** 2)

    
        subprocess.run(
            ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
                "initial_velocity:=" + str(-0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p",
                "target_distance:=" + str(dist)])

    def run_drop_pallet(self):  # check if pallet is present already and update status
        command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                   str(pallet_location[0]), str(pallet_location[1]), str(pallet_location[2]),
                   str(pallet_location[3])]
        run_command_with_retry(command)

        tf_pose = get_transformed_pose(pallet_location)
        steering_angle, vel, dist = get_precise_control_key('up', tf_pose)
        print("unclamped", steering_angle, dist)
        steering_angle = clamp(steering_angle, -2.0, 2.0)
        dist = clamp(dist, 1.3, 1.45)
        print("clamped", steering_angle, dist)
        
        result = subprocess.run(
            ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node_with_forktip_check_for_drop",
                "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p",
                "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)],
            capture_output=True, text=True)
        if result.returncode == 0:
            self.run_drive_back()
            return False

        command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                    str(pallet_location[0]),
                    str(pallet_location[1]), str(pallet_location[2]), str(pallet_location[3])]
        run_command_with_retry(command)

        current_pose = get_current_pose()
        print('pallet dropped at ->', current_pose)

        return True

    
    def run_drive_for_pallet(self):
        result = None
        print('here')
        # if cancel_status == False:
        #     subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
        #                     str(pallet_location[0]), str(pallet_location[1]),
        #                     str(pallet_location[2]), str(pallet_location[3])])
        fetched_m_offset = None

        # try:
        fetched_m_offset, fetched_angular_offset = get_pds_tag_results()

        if fetched_angular_offset == None:
            return 'False'
        elif abs(fetched_m_offset) > 0.0:
            
            # self.v2_control(lateral_pallet_picking, 'slow')
            # self.v2_control([], 'pallet_pickup', fetched_m_offset)
            self.pp_control(fetched_m_offset)


                # while check_counter < 3:
                #     # subprocess.run(["ros2", "run", "byd_bopt_pallet_picker", "correct_heading_v2",
                #     #                 str(fetched_m_offset)])
                #     # subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp",
                #     #                 "pose_correction_node", str(pallet_location[0]),
                #     #                 str(pallet_location[1]), str(pallet_location[2]),
                #     #                 str(pallet_location[3])])
                #     fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
                #     check_counter += 1
                #     if abs(fetched_m_offset) < 0.05:
                #         if abs(fetched_angular_offset) > 0.07:
                #             command = ["ros2", "run", "byd_pose_correction_node_cpp",
                #                        "pose_correction_node_rad", str(-float(fetched_angular_offset))]  # Add Retry
                #             run_command_with_retry(command)
                #         break
        # except Exception as e:
        #     # Handle any other exceptions
        #     print("Error, camera can't find pallet:", str(e))
        #     return 'False'
        # fetched_m_offset, fetched_angular_offset = get_pds_tag_results()

        if fetched_m_offset == None:
            return 'False'
        # fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
        # print('check',fetched_m_offset, fetched_angular_offset)
        # command = ["ros2", "run", "byd_bopt_pallet_picker", "drive_for_pallet"]
        # try:
        #     result = run_command_with_retry(command)
        #     stdout = result.stdout.strip()
        #     print("result stdout:", stdout)

        #     if stdout == 'True':
        #         return 'True'
        #     else:
        #         return 'False'
        # except Exception as e:
        #     print(str(e))
        #     return 'False'


    def run_drive_back(self):  # current_loc
        current_pose = get_current_pose()
        dist = np.sqrt((self.dock_location[0] - current_pose[0]) ** 2 + (self.dock_location[1] - current_pose[1]) ** 2)
        # dist = np.sqrt((dock_location[0] - pallet_location[0]) ** 2 + (dock_location[1]- pallet_location[1]) ** 2)
        # dist+=0.1
        subprocess.run(
                ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
                 "initial_velocity:=" + str(-0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p",
                 "target_distance:=" + str(dist)])

    def fork_down(self):
        duration = 3  # 3 seconds
        command = "ros2 service call /byd/send_command example_interfaces/srv/Command \"{command: 'down'}\""

        retcode = subprocess.call(command, shell=True)

    def fork_up(self):
        duration = 3  # 3 seconds
        command = "ros2 service call /byd/send_command example_interfaces/srv/Command \"{command: 'up'}\""

        retcode = subprocess.call(command, shell=True)


    def drive_to_point(self, destination_pose, speed, align=False):
        time.sleep(0.2)
        waypoints = []
        current_formatted_pose = get_current_pose()
        tf_pose = get_transformed_pose(destination_pose)

        waypoints = []
        waypoints.append(get_current_pose())
        waypoints.append(self.source_dock_pose)
        if speed == 'fast':
            self.v2_control(waypoints, 'fast')
        elif speed == 'slow':
            self.v2_control(waypoints, 'slow') #return status for error handling

        if align:
            subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(destination_pose[0]),
                            str(destination_pose[1]), str(destination_pose[2]), str(destination_pose[3])])






def signal_handler(sig, frame):
    print('Ctrl-C pressed, stopping the robot and shutting down')
    # Create a node
    node = rclpy.create_node('stop_robot_node')

    # Publisher for cmd_vel to stop the robot
    cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    stop_msg = Twist()
    stop_msg.linear.x = 0.0
    stop_msg.angular.z = 0.0

    # Publisher for the /state topic
    state_publisher = node.create_publisher(String, '/state', 10)
    state_msg = String()
    state_msg.data = 'auto'
    i = 0
    while i <= 100:
        cmd_vel_publisher.publish(stop_msg)
        state_publisher.publish(state_msg)
        i += 1
    navigator.cancelTask()

    kill_child_processes(os.getpid())

    # Allow some time for the messages to be sent
    time.sleep(1)

    # Cleanup and exit
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
        return
    for child in parent.children(recursive=True):
        child.terminate()
    gone, still_alive = psutil.wait_procs(parent.children(recursive=True), timeout=5, callback=None)
    for p in still_alive:
        p.kill()


def main(args=None):
    signal.signal(signal.SIGINT, signal_handler)

    rclpy.init(args=args)

    # Check if source and destination are provided as command-line arguments
    if len(sys.argv) < 3:
        print("Usage: ros2 run <your_package_name> load_transporter_node <source> <destination> [state_id]")
        rclpy.shutdown()
        return

    source = sys.argv[1]
    destination = sys.argv[2]
    state_id = int(sys.argv[3]) if len(sys.argv) > 3 else None
    global navigator  # Make navigator global to access it in the signal handler
    navigator = BasicNavigator()

    # Create an instance of LoadTransporter and set source and destination
    transporter = LoadTransporter(source, destination, navigator)

    transporter.pub_fb("Task Started")

    # Define the sequence of operations mapped by state IDs
    operations = {
        1: lambda: transporter.go_to_easy_dock_for_pickup_using_flow(),
        2: lambda: transporter.precise_control_to_pickup_dock(),
        3: lambda: transporter.go_for_pallet_pickup(),
        4: lambda: transporter.drive_back_to_flow_after_pickup(),
        5: lambda: transporter.go_to_drop_off_easy_dock_using_flow(),
        6: lambda: transporter.precise_control_to_drop_off_dock(),
        7: lambda: transporter.go_for_pallet_drop(),
        8: lambda: transporter.drive_back_to_flow_after_drop()
    }

    # Start from a specific point in the sequence if state_id is provided
    error_found = False
    error_state = None
    start_point = state_id if state_id else 1
    for state, operation in operations.items():
        if state >= start_point:
            stat = operation()  # Execute the operation
            if stat == False:
                if state == 3:
                    error_state = 3
                    start_point = 11
                elif state == 8:
                    error_state = 8
                    start_point = 13
                    des = transporter.destination_name
                    subprocess.run(
                        ["ros2 run load_transporter_node load_transporter_node_v6 " + str(des) + " " + 'buffer' + " 5",
                         "arguments"], shell=True)
                    break
                elif state == 9:
                    error_state = 9
                    start_point = 13
                    des = transporter.destination_name
                    subprocess.run(
                        ["ros2 run load_transporter_node load_transporter_node_v6 " + str(des) + " " + 'buffer' + " 5",
                         "arguments"], shell=True)
                    break
            # Add any necessary error handling or status checking here
    # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    #         s.connect(('192.168.68.57', 65434))
    #         s.sendall(b"Request Info")
    #         data = s.recv(1024).decode()
    #         print(f"Received: {data}")

    des = transporter.destination_name
    if des != "buffer":

        try:
            # with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            #         s.connect(('192.168.68.57', 65434))
            if error_found == False:
                if error_state == 3:
                    transporter.send_data(b"Pallet_Not_Present")
                    transporter.send_data(b"Task ended")
                elif error_state in [8, 9]:
                    transporter.send_data(b"task_incomplete_pallet_not_dropped")
                    transporter.send_data(b"Task ended")
                else:
                    transporter.send_data(b"Task ended")
            else:
                transporter.send_data(b"Task ended")
                # data = s.recv(1024).decode()
            print('Transporter sequence completed:', transporter.sequence_complete)
        except Exception as e:
            print(e)

    transporter.sequence_complete = True
    print(transporter.next_task)
    if transporter.next_task == "Parking":
        transporter.task_park()

    transporter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





def get_current_pose():
    node = CurrentLocationNode()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.current_pose is not None:
            break
    current_pose = node.current_pose

    current_formatted_pose = [current_pose.position.x, current_pose.position.y, current_pose.orientation.z,
                              current_pose.orientation.w]
    # print('fetched current pose:',current_formatted_pose)
    node.destroy_node()
    return current_formatted_pose


def get_transformed_pose(location):
    tf2_broadcaster = TF2Broadcaster(location[0], location[1], location[2], location[3])
    timeout_counter = 0
    print("location #####", location)
    while tf2_broadcaster.get_pose() is None and timeout_counter < 100:
        rclpy.spin_once(tf2_broadcaster)
        timeout_counter += 1

    tf_pose = tf2_broadcaster.get_pose()
    print(tf_pose)
    tf2_broadcaster.destroy_node()
    return tf_pose


def find_nearest_key(lookup_table, position, initial_tolerance=0.01, max_tolerance=0.1, tolerance_increment=0.01):
    tolerance = initial_tolerance

    while tolerance <= max_tolerance:
        print('current tolerance:', tolerance)
        potential_keys = []

        # Gather all keys within the current tolerance
        for key in lookup_table.keys():
            position_difference = np.sqrt((key[0] - position[0]) ** 2 + (key[1] - position[1]) ** 2)
            vel = lookup_table[key][1]

            # Check if the key satisfies the additional condition on velocity
            if position_difference <= tolerance and ((position[0] > 0 and vel > 0) or (position[0] <= 0 and vel <= 0)):
                potential_keys.append(key)

        # If potential keys are found, select the one with the smallest 'dist' value
        if potential_keys:
            nearest_key = min(potential_keys, key=lambda k: lookup_table[k][2])
            return nearest_key

        # Increase tolerance for next iteration
        tolerance += tolerance_increment

    return None


def get_precise_control_key(robot_state, tf_pose):
    print('going to ->', tf_pose)
    if robot_state == 'up':
        with open(ws_path + 'src/task_allocator/byd_task_allocator/lookup_table_1.33.pkl', 'rb') as f:
            loaded_lookup_table = pickle.load(f)
    elif robot_state == 'down':
        with open(ws_path + 'src/task_allocator/byd_task_allocator/lookup_table_1.36.pkl', 'rb') as f:
            loaded_lookup_table = pickle.load(f)

    position = tf_pose[:2]

    nearest_key = find_nearest_key(loaded_lookup_table, position)

    if nearest_key is None:
        print("No solution found within given tolerances.")
        return 0.0, 0.0, 0.0
    else:
        # print("Nearest key:", nearest_key)
        steering_angle = np.rad2deg(loaded_lookup_table[nearest_key][0])
        vel = loaded_lookup_table[nearest_key][1]
        dist = loaded_lookup_table[nearest_key][2]

    return steering_angle, vel, dist


def get_current_odot_state():
    node = CurrentOdotStateNode()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.sensor_data is not None:
            break
    front_sensor_byte = node.front_sensor_byte
    back_sensor_byte = node.back_sensor_byte
    # print('fetched current pose:',current_formatted_pose)
    node.destroy_node()
    return front_sensor_byte, back_sensor_byte


def kill_process_tree(pid, including_parent=True):
    parent = psutil.Process(pid)
    children = parent.children(recursive=True)
    for child in children:
        child.kill()
    psutil.wait_procs(children)
    if including_parent:
        parent.kill()
        parent.wait()



def clamp(value, minimum, maximum):
    return max(minimum, min(value, maximum))


def run_command_with_retry(command, timeout=45, max_retries=5, delay_between_retries=5):
    """
    Run a command with a timeout and retries. Terminates the process if it hangs.

    :param command: Command to run (list of strings)
    :param timeout: Timeout in seconds for the command
    :param max_retries: Maximum number of retries
    :param delay_between_retries: Delay in seconds between retries
    :return: subprocess.CompletedProcess object
    # Example usage
    command = ["ros2", "run", "your_package", "your_node"]
    try:
        result = run_command_with_retry(command)
        print(result.stdout)
    except Exception as e:
        print(str(e))
    """
    for attempt in range(max_retries):
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            output, error = process.communicate(timeout=timeout)
            return subprocess.CompletedProcess(process.args, process.returncode, output, error)
        except subprocess.TimeoutExpired:
            print(
                f"Command timed out (attempt {attempt + 1}/{max_retries}). Retrying in {delay_between_retries} seconds...")
            process.kill()
            process.wait()  # Ensure the process has terminated before retrying
            time.sleep(delay_between_retries)

    # If all retries fail, raise an exception
    raise Exception(f"Command '{' '.join(command)}' failed after {max_retries} retries.")


def get_pds_results():
    result_pds = subprocess.run(["ros2", "run", "pallet_detection", "pallet_detection"], capture_output=True, text=True)
    output_string = result_pds.stdout

    # Extract "Distance to midpoint"
    distance_to_midpoint_match = re.search("Distance to midpoint: (\d+\.\d+)", output_string)
    distance_to_midpoint = float(distance_to_midpoint_match.group(1)) if distance_to_midpoint_match else None

    # Extract "tensor()"
    tensor_match = re.search("tensor\((\d+\.\d+)\)", output_string)
    tensor_value = float(tensor_match.group(1)) if tensor_match else None

    # Extract "fetched_m_offset"
    fetched_m_offset_match = re.search("fetched_m_offset (-?\d+\.\d+)", output_string)
    fetched_m_offset = float(fetched_m_offset_match.group(1)) if fetched_m_offset_match else None
    print(distance_to_midpoint, tensor_value, fetched_m_offset)

    return distance_to_midpoint, tensor_value, fetched_m_offset


def get_pds_tag_results():
    result_pds = subprocess.run(["ros2", "run", "pallet_detection", "pallet_detection_tag"], capture_output=True,
                                text=True)
    output_string = result_pds.stdout

    # Extract "fetched_m_offset"
    fetched_m_offset_match = re.search("fetched_m_offset (-?\d+\.\d+)", output_string)
    fetched_m_offset = float(fetched_m_offset_match.group(1)) if fetched_m_offset_match else None
    fetched_angular_offset_match = re.search("fetched_angular_offset (-?\d+\.\d+)", output_string)
    fetched_angular_offset = float(fetched_angular_offset_match.group(1)) if fetched_angular_offset_match else None
    print(fetched_m_offset, fetched_angular_offset)

    return fetched_m_offset, fetched_angular_offset


class CurrentLocationNode(Node):
    def __init__(self):
        super().__init__('current_location_node')
        qos_settings = QoSProfile(depth=10)
        qos_settings.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self.pose_sub = self.create_subscription(PoseStamped, '/current_pose', self.current_pose_callback, qos_settings)
        self.current_pose = None
        self.success = False

    def current_pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose


class CurrentOdotStateNode(Node):
    def __init__(self):
        super().__init__('current_odot_state_node')
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.subscription = self.create_subscription(String, '/byd/can_odot_data', self.odot_callback, qos)
        self.sensor_data = None
        self.back_sensor_byte = None
        self.front_sensor_byte = None

    def odot_callback(self, msg):
        # self.get_logger().info(f'Received message: "{msg.data}"')
        sensor_data = msg.data.split(' ')[1]
        self.back_sensor_byte = sensor_data[0]  # back sensors
        self.front_sensor_byte = sensor_data[-1]  # forktip sensors
        self.sensor_data = msg.data.split(' ')


class TF2Broadcaster(Node):

    def __init__(self, x, y, z, w):
        super().__init__('tf2_broadcaster')
        qos_settings = QoSProfile(depth=10)
        qos_settings.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Setup a timer to broadcast the transform every 100ms
        # self.timer = self.create_timer(0.1, self.broadcast_tf)
        self.subscription = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.broadcast_tf,
            qos_settings)

        # Set the translation and rotation for the point
        self.x = x
        self.y = y
        self.z = z
        self.w = w

        # Initialize pose as None
        self.pose = None

    def get_pose(self):
        return self.pose

    def broadcast_tf(self, msg):

        t_map_pose = TransformStamped()
        t_map_pose.header.stamp = self.get_clock().now().to_msg()
        t_map_pose.header.frame_id = 'map'
        t_map_pose.child_frame_id = 'pose'

        t_map_pose.transform.translation.x = self.x
        t_map_pose.transform.translation.y = self.y
        t_map_pose.transform.translation.z = 0.0

        t_map_pose.transform.rotation.x = 0.0
        t_map_pose.transform.rotation.y = 0.0
        t_map_pose.transform.rotation.z = self.z
        t_map_pose.transform.rotation.w = self.w
        self.broadcaster.sendTransform(t_map_pose)

        # Broadcast transform from pose to odom

        t_pose_odom = TransformStamped()
        t_pose_odom.header.stamp = self.get_clock().now().to_msg()
        t_pose_odom.header.frame_id = 'map'
        t_pose_odom.child_frame_id = 'base_link_2'

        t_pose_odom.transform.translation.x = msg.pose.position.x
        t_pose_odom.transform.translation.y = msg.pose.position.y
        t_pose_odom.transform.translation.z = 0.0

        t_pose_odom.transform.rotation.x = 0.0
        t_pose_odom.transform.rotation.y = 0.0
        t_pose_odom.transform.rotation.z = msg.pose.orientation.z
        t_pose_odom.transform.rotation.w = msg.pose.orientation.w

        self.broadcaster.sendTransform(t_pose_odom)
        try:
            transform = self.tf_buffer.lookup_transform('base_link_2', 'pose', rclpy.time.Time())
            self.pose = (transform.transform.translation.x, transform.transform.translation.y,
                         transform.transform.rotation.z, transform.transform.rotation.w)
            print(transform.transform.translation.x, transform.transform.translation.y, transform.transform.rotation.z,
                  transform.transform.rotation.w)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transform available: ' + str(e))
