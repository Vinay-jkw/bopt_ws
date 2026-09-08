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
# from helper_classes import TF2Broadcaster, CurrentLocationNode, CurrentOdotStateNode
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

global cancel_status



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
        global cancel_status
        self.pallet_picked = False
        self.current_pose = None
        self.success = False
        self.pallet_manager_lift = None

        self.pallet_manager_drop = None

        self.current_state_of_task = 0

        self.thread2 = Thread(target=self.field_back, )
        self.next_task_source = None

        # Set source related locations
        self.source_easy_destination_pose = self.easy_station_locations[self.source_name]
        self.rev_source_easy_destination_pose = self.rev_easy_station_location[self.source_name]
        self.source_destination_pose = self.dock_station_locations[self.source_name]
        self.source_station_pose = self.station_locations[self.source_name]

        self.pallet_manager_lift = PalletManager(action='lift', easy_destination_pose=self.source_easy_destination_pose,
                                                 dock_location=self.source_destination_pose,
                                                 pallet_location=self.source_station_pose,
                                                 publisher_fn=self.pub_fb)
        self.pallet_manager_lift_rev = PalletManager(action='lift',
                                                     easy_destination_pose=self.rev_source_easy_destination_pose,
                                                     dock_location=self.source_destination_pose,
                                                     pallet_location=self.source_station_pose,
                                                     publisher_fn=self.pub_fb)

        # Set destination related locations
        self.rev_destination_easy_destination_pose = self.rev_easy_station_location[destination]
        self.destination_easy_destination_pose = self.easy_station_locations[destination]
        self.destination_destination_pose = self.dock_station_locations[destination]
        self.destination_station_pose = self.station_locations[destination]

        self.pallet_manager_drop = PalletManager(action='drop',
                                                 easy_destination_pose=self.destination_easy_destination_pose,
                                                 dock_location=self.destination_destination_pose,
                                                 pallet_location=self.destination_station_pose,
                                                 publisher_fn=self.pub_fb)
        self.pallet_manager_drop_rev = PalletManager(action='drop',
                                                     easy_destination_pose=self.rev_destination_easy_destination_pose,
                                                     dock_location=self.destination_destination_pose,
                                                     pallet_location=self.destination_station_pose,
                                                     publisher_fn=self.pub_fb)

        

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

    def get_flow_state(self, flow_matrix, flow_matrix_rev):
        current_pose = get_current_pose()

        min_diff = float('inf')
        closest_pose = None

        for ref_pose in flow_matrix.values():
            diff = self.compute_positional_difference(current_pose, ref_pose)
            if diff < min_diff:
                min_diff = diff
                closest_pose = ref_pose

        min_diff = float('inf')
        closest_pose_rev = None

        for ref_pose in flow_matrix_rev.values():
            diff = self.compute_positional_difference(current_pose, ref_pose)
            if diff < min_diff:
                min_diff = diff
                closest_pose_rev = ref_pose

        angular_diff_closest = self.compute_angular_difference(current_pose, closest_pose)
        angular_diff_closest_rev = self.compute_angular_difference(current_pose, closest_pose_rev)
        # print(angular_diff_closest, angular_diff_closest_rev, 'ADC')
        if angular_diff_closest + angular_diff_closest_rev < 360:
            if angular_diff_closest < angular_diff_closest_rev:
                return 1
            else:
                return 2
        else:
            if angular_diff_closest < angular_diff_closest_rev:
                return 2
            else:
                return 1

    def is_source_value_greater_than_destination(self, source, destination, flow_matrix, flow_matrix_rev):

        flow_state = self.get_flow_state(flow_matrix, flow_matrix_rev)  # get flow_state
        if flow_state == 1:
            station_sequence = {'buffer': 0, 'p8': 1, 'p7': 2, 'p6': 3, 'p5': 4, 'p4': 5, 'p3': 6, 'p2': 7, 'p1': 8,
                                'pick_1': 9, 's1': 10, 's2': 11, 's3': 12, 's4': 13, 's5': 14}
        elif flow_state == 2:
            station_sequence = {'s5': 14, 's4': 13, 's3': 12, 's2': 11, 's1': 10, 'pick_1': 9, 'p1': 8, 'p2': 7,
                                'p3': 6, 'p4': 5, 'p5': 4, 'p6': 3, 'p7': 2, 'p8': 1, 'buffer': 0}

        source_value = station_sequence.get(source)
        destination_value = station_sequence.get(destination)
        print(source_value, destination_value)
        if source_value > destination_value:
            return True
        else:
            return False

    def go_to_easy_dock_for_pickup_using_flow(self):
        self.current_state_of_task = 1
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"Task started")
        self.send_data(b"1")

        self.pub_fb("Localizing")
        current_formatted_pose = get_current_pose()

        point1 = current_formatted_pose[:2]
        point2 = self.rev_source_easy_destination_pose[:2]

        path_coords_with_quaternions = self.find_path_and_angles_between_points(self.G_loaded, point1, point2)


        # if self.destination_name in ['p1', 'p2', 'p3']:
        # if len(path_coords_with_quaternions) != 0:
        #     path_coords_with_quaternions.pop()
            # path_coords_with_quaternions.pop()
        path_coords_with_quaternions.pop()



        self.pub_fb("Navigating through waypoints")


        # result_through_poses = self.new_navigate_through_flow_poses(self.navigator, path_coords_with_quaternions,
        #                                                             self.rev_source_easy_destination_pose)
        path_coords_with_quaternions.append(self.rev_source_easy_destination_pose)
        self.v2_control(path_coords_with_quaternions, 'fast')


        # command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
        #            str(self.rev_source_easy_destination_pose[0]),
        #            str(self.rev_source_easy_destination_pose[1]),
        #            str(self.rev_source_easy_destination_pose[2]),
        #            str(self.rev_source_easy_destination_pose[3])]

        run_command_with_retry(command)
        self.pub_fb('at station dock')

        return True

    def v2_control(self, waypoints_to_navigate, type):
        '''

        :param waypoints_to_navigate:
        :param type: slow, fast, deep_pickup, deep_stack
        :return:
        '''
        SCALE = 1
        waypoints = []
        waypoints.append(get_current_pose())
        waypoints.extend(waypoints_to_navigate)

        if type == 'fast':
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
            with open('/home/byd3/python3.10_env/waypoints.json', 'w') as file:
                json.dump(data, file, indent=4)

            command = ['/home/byd3/python3.10_env/myenv/bin/python', '/home/byd3/python3.10_env/deep_pallet_pickup_plan.py']
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_fast",
                                     "--path_file",
                                     "/home/byd3/bopt_ws/src/load_transporter_node/load_transporter_node/constructed_rs_path.pkl"
                                     ], capture_output=True, text=True)
        elif type == 'slow':
            rs_path_waypoints = []

            for waypoint in waypoints:
                if waypoint == waypoints[-1]:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                              self.quaternion_to_angle(waypoint[2], waypoint[3]) + 3.14, 1.2 * SCALE,
                                              1.0 * SCALE])
                else:
                    rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                              self.quaternion_to_angle(waypoint[2], waypoint[3]) + 3.14, 1.2 * SCALE,
                                              0.0])

            data = {
                "rs_path_waypoints": rs_path_waypoints
            }

            # Write the data to a JSON file
            with open('/home/byd3/python3.10_env/waypoints.json', 'w') as file:
                json.dump(data, file, indent=4)

            command = ['/home/byd3/python3.10_env/myenv/bin/python',
                       '/home/byd3/python3.10_env/deep_pallet_pickup_plan.py']
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            print(f"Script output:\n{result.stdout}")

            result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_slow_deep_stack",
                                     "--path_file",
                                     "/home/byd3/bopt_ws/src/load_transporter_node/load_transporter_node/constructed_rs_path.pkl"
                                     ], capture_output=True, text=True)


    def precise_control_to_dock(self):
        self.current_state_of_task = 2
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"2")

        self.thread1 = Thread(target=self.field_change, )  # to change later
        self.thread1.start()
        current_formatted_pose = get_current_pose()
        tf_pose = get_transformed_pose(self.source_destination_pose)



        steering_angle, vel, dist = get_precise_control_key('down', tf_pose)
        print('tst val:', steering_angle, vel, dist)

        self.pub_fb("Navigating to Dock Pose")

        command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node",
                   "my_node", "--ros-args", "-p", "initial_velocity:=" +
                   str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p",
                   "target_distance:=" + str(dist)]
        run_command_with_retry(command)

        self.v2_control(self.source_destination_pose, 'slow')

        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                        str(self.source_destination_pose[0]), str(self.source_destination_pose[1]),
                        str(self.source_destination_pose[2]), str(self.source_destination_pose[3])])

        if self.source_name == 'p6':
            drive_to_point_and_adjust([8.658084, 28.186788, 0.999, 0.025], 'down')
        elif self.source_name == 'p7':
            drive_to_point_and_adjust([7.178084, 28.376788, 0.999, 0.025], 'down')
        elif self.source_name == 'p8':
            drive_to_point_and_adjust([5.778084, 28.566788, 0.999, 0.025], 'down')
        elif self.source_name == 'p2':
            drive_to_point_and_adjust([8.68, 25.59006, 0.99, 0.0], 'down')
        elif self.source_name == 'p3':
            drive_to_point_and_adjust([7.219149, 25.5912, 0.99, 0.01], 'down')
        elif self.source_name == 'p4':
            drive_to_point_and_adjust([5.759149, 25.6512, 0.99, 0.01], 'down')


        waypoints = []

        if self.source_name == 'pick1':
            waypoints.append(get_current_pose())
            # waypoints.append(self.dock_station_locations['pick2'])
            waypoints.append(self.dock_station_locations['pick3'])
        elif self.source_name == 'pick4':
            waypoints.append(get_current_pose())
            # waypoints.append(self.dock_station_locations['pick2'])
            waypoints.append(self.dock_station_locations['pick6'])

        rs_path_waypoints = []
        SCALE = 1
        for waypoint in waypoints:
            if waypoint == waypoints[-1]:
                rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                          3.14 + self.quaternion_to_angle_rad(waypoint[2], waypoint[3]), 1.8 * SCALE,
                                          0.0 * SCALE])
            else:
                rs_path_waypoints.append([waypoint[0] * SCALE, waypoint[1] * SCALE,
                                          3.14 + self.quaternion_to_angle_rad(waypoint[2], waypoint[3]), 1.8 * SCALE,
                                          0.0])

        data = {
            "rs_path_waypoints": rs_path_waypoints
        }

        # Write the data to a JSON file
        with open('/home/byd3/python3.10_env/waypoints.json', 'w') as file:
            json.dump(data, file, indent=4)

        command = ['/home/byd3/python3.10_env/myenv/bin/python', '/home/byd3/python3.10_env/deep_pallet_pickup_plan.py']
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        print(f"Script output:\n{result.stdout}")

        result = subprocess.run(["ros2", "run", "nmpc_controller", "nmpc_controller_v2_slow_deep_stack",
                                 "--path_file",
                                 "/home/byd3/bopt_ws/src/load_transporter_node/load_transporter_node/constructed_rs_path.pkl"
                                 ], capture_output=True, text=True)

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

        sta = self.pallet_manager_lift.run_drive_for_pallet()
        print(sta)
        if sta == 'False':
            self.pallet_picked = False
            # if self.source_name in ['p1', 'p2'] :=='p6' or self.source_name=='p7' or self.source_name=='p8' or self.source_name=='p2' or self.source_name=='p3' or self.source_name=='p4':
            self.pallet_manager_lift.run_drive_back()
            return False
        self.pallet_manager_lift.fork_up()

        # switching field on
        self.pallet_picked = True
        return True

    def drive_back_to_dock(self):
        # Drive back to dock
        self.current_state_of_task = 4
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=str(self.current_state_of_task), )
        self.current_state_thread.start()
        self.send_data(b"4")

        self.pallet_manager_lift.run_drive_back()

        return True

    def reorient_to_flow(self):
        # Reorient to flow (easy dock)
        self.current_state_of_task = 5
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"5")

        drive_to_point_and_adjust(self.source_easy_destination_pose, 'up')

        # self.pallet_manager_lift.reorient_to_flow()

        # if self.source_name == "pick_1" or self.source_name== 'p5' or self.source_name=='p6' or self.source_name=='p7' or self.source_name=='p8' or self.source_name=='p1' or self.source_name=='p2' or self.source_name=='p3' or self.source_name=='p4':
        #     self.pallet_manager_lift_rev.reorient_to_flow()
        # else:
        #     flow_break=self.is_source_value_greater_than_destination(self.source_name,self.destination_name,self.flow_matrix,self.rev_flow_matrix)
        #     flow_state = self.get_flow_state(self.flow_matrix, self.rev_flow_matrix)
        #     if flow_break:
        #         self.pallet_manager_lift_rev.reorient_to_flow()
        #     else:
        #         self.pallet_manager_lift.reorient_to_flow()

        return True

    def pose_correct_on_flow_after_lift(self):
        self.current_state_of_task = 6
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"6")


        self.thread2 = Thread(target=self.field_back, )
        self.thread2.start()

        return True

    def go_to_drop_off_easy_dock_using_flow(self):
        self.current_state_of_task = 7
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"7")

        # Go to easy dock of drop using flow

        self.pub_fb("Localizing")
        current_formatted_pose = get_current_pose()

        point1 = current_formatted_pose[:2]

        final_point = []
        if self.destination_name == 'buffer':
            final_point = self.destination_easy_destination_pose
        else:
            final_point = self.rev_destination_easy_destination_pose

        point2 = final_point[:2]

        path_coords_with_quaternions = self.find_path_and_angles_between_points(self.G_loaded, point1, point2)
        if self.destination_name == 'buffer':
            path_coords_with_quaternions.pop()
            path_coords_with_quaternions.pop()
            path_coords_with_quaternions.pop()
        if self.destination_name == 's5':
            path_coords_with_quaternions.pop()
            path_coords_with_quaternions.pop()

        self.pub_fb("Navigating through waypoints")

        self.v2_control(path_coords_with_quaternions, 'fast')

        try:
            if len(path_coords_with_quaternions) > 2:  # check
                result_through_poses = self.new_navigate_through_flow_poses(self.navigator,
                                                                            path_coords_with_quaternions, final_point)

        except Exception as e:
            # Handle any other exceptions
            self.pub_fb("can't find path")
            print("Error, can't find path:", str(e))

        command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(final_point[0]),
                   str(final_point[1]), str(final_point[2]),
                   str(final_point[3])]

        run_command_with_retry(command)

        robot_state = 'at station dock'



        return True

    def precise_control_to_drop_off_dock(self):
        # Precise control to drop-off dock
        self.current_state_of_task = 8
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"8")

        self.thread1 = Thread(target=self.field_change, )  # to change later
        self.thread1.start()
        time.sleep(1)

        self.v2_control(self.destination_destination_pose, 'slow')


        # current_formatted_pose = get_current_pose()
        # tf_pose = get_transformed_pose(self.destination_destination_pose)
        #
        # steering_angle, vel, dist = get_precise_control_key('up', tf_pose)
        #
        # # if cancel_status==False:
        # self.pub_fb("Navigating to Dock Pose")
        #
        # # command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node_with_forktip_check", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)]
        # result = subprocess.run(
        #     ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
        #      "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p",
        #      "target_distance:=" + str(dist)], capture_output=True, text=True)


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
        self.current_state_of_task = 9
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"9")

        res = self.check_pallet_present()
        fun_res = True
        if res == 'True':
            fun_res = False
            self.send_data(b"Pallet_Already_Present")

            return False
        else:
            result = self.pallet_manager_drop.run_drop_pallet()
            if result == False:
                self.send_data(b"Pallet_Already_Present")

                return False

            self.pallet_manager_drop.fork_down()

        return fun_res

    def drive_back_from_drop_off(self):
        # Drive back from drop off
        self.current_state_of_task = 10
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"10")

        self.pallet_manager_drop.run_drive_back()

        return True

    def task_park(self):

        robot_state = 'localizing and parking'
        # #robot locates itself
        # current_formatted_pose = get_current_pose()
        # source_pose = current_formatted_pose

        # easy_destination_pose = self.easy_station_locations["Parking"]
        # easy_destination_pose_rev = self.rev_easy_station_location["Parking"]

        # destination_pose = self.dock_station_locations["Parking"]
        # distance_between_goals = self.euclidean_distance(easy_destination_pose, destination_pose)
        # station_pose = self.station_locations["Parking"]
        # # flow_state = self.get_flow_state(self.flow_matrix, self.rev_flow_matrix) #get flow_state
        # # flow_break=self.is_source_value_greater_than_destination(self.source_name,self.destination_name,self.flow_matrix,self.rev_flow_matrix)
        # destination_name="Parking"
        # current_formatted_pose = get_current_pose()

        # #  cancel_status = False

        # self.pub_fb("Localizing")
        # current_formatted_pose = get_current_pose()

        # point1 = current_formatted_pose[:2]
        # point2 = easy_destination_pose[:2]

        # path_coords_with_quaternions = self.find_path_and_angles_between_points(self.G_loaded, point1, point2)

        # self.pub_fb("Navigating through waypoints")

        # # print("flow_statee  ---====",flow_state)
        # # try:
        # #     if len(path_with_coords) > 2: #check
        # result_through_poses = self.new_navigate_through_flow_poses(self.navigator, path_coords_with_quaternions, easy_destination_pose)

        # result_to_pose = self.navigate_to_pose(self.navigator, )
        # result_to_pose = self.navigate_to_pose(self.navigator, )
        result_to_poses = self.navigate_through_poses(self.navigator,
                                                      [[11.81, 21.28, -0.71, 0.69], [7.026, 16.08, -0.71, 0.69]])

        # source_pose = current_formatted_pose
        # _,dis_1 = self.find_nearest_point_from_current_location_to_flow(current_formatted_pose,self.flow_matrix)
        # _,dis_2 = self.find_nearest_point_from_current_location_to_flow(current_formatted_pose,self.pick_flow)
        # print("    PICK_FLOW  ",self.pick_flow)
        # print(dis_1,"  ",dis_2)
        # flow_no = 1
        # if dis_1 < dis_2 and (self.source_name == 'pick_1' or self.source_name=='p1' or self.source_name=='p2' or self.source_name=='p3' or self.source_name=='p4' or self.source_name=='p7' or self.source_name=='p8' or self.source_name== 'p5' or self.source_name=='p6'):
        #     flow_matrix = self.flow_matrix
        #     rev_flow_matrix = self.rev_flow_matrix
        #     flow_no = 1
        #     print("----------------NORMAL_FLOW++++++++++++")
        # elif dis_2 < dis_1 and (self.source_name == 'pick_1' or self.source_name=='p1' or self.source_name=='p2' or self.source_name=='p3' or self.source_name=='p4' or self.source_name=='p7' or self.source_name=='p8' or self.source_name== 'p5' or self.source_name=='p6'):
        #     flow_matrix = self.pick_flow
        #     rev_flow_matrix = self.rev_pick_flow
        #     flow_no=2
        #     print("----------------PARKING_FLOW++++++++++++")
        # else:
        #     flow_matrix = self.flow_matrix
        #     rev_flow_matrix = self.rev_flow_matrix
        #     flow_no = 0

        # if flow_state == 1 and flow_break:
        #     path_with_names, path_with_coords = self.find_path_from_current_location(current_formatted_pose, destination_name,
        #                                                                             self.flow_matrix, self.easy_station_locations)
        # elif flow_state == 2 and flow_break:
        #     path_with_names, path_with_coords = self.find_path_from_current_location(current_formatted_pose, destination_name,
        #                                                                             self.rev_flow_matrix, self.rev_easy_station_location)
        # if flow_state == 2:
        #     path_with_names, path_with_coords = self.find_path_from_current_location(current_formatted_pose, destination_name,
        #                                                                             self.flow_matrix_park, self.rev_easy_station_location)
        # else:
        #     path_with_names, path_with_coords = self.find_path_from_current_location(current_formatted_pose, destination_name,
        #                                                                             self.flow_matrix_park, self.easy_station_locations)

        # if flow_state == 1:
        #     result_through_poses = self.new_navigate_through_flow_poses(self.navigator, path_with_coords, self.destination_easy_destination_pose)
        # else:
        #     result_through_poses = self.new_navigate_through_flow_poses(self.navigator, path_with_coords, self.rev_destination_easy_destination_pose)

        # if flow_state == 1:
        #     # print('nav2',flow_matrix[1])
        #     result_to_pose = self.navigate_to_pose(navigator, [10.46,	2.24,	-0.00611011,	0.999981])
        # else:
        #     # print('nav2',flow_matrix_rev[-2])
        #     result_to_pose = self.navigate_to_pose(navigator, [10.46,	2.24,	-0.999996,	0.00289796])
        # drive_to_point([8.529264, 3.81,	0.702044, 0.712134], 'down')
        # drive_to_point([8.529264, 5.596786,	0.702044, 0.712134], 'down')
        # drive_to_point([8.53668,	6.996573,	0.702044,	0.712134], 'down')

    def reorient_to_flow_after_drop_off(self):
        # Reorient to flow after drop off
        self.current_state_of_task = 11
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"11")

        sou = None
        des = None
        try:
            data_rec = self.send_data(b"Request Info")

            if data_rec != "Parking":
                sou, des = data_rec.split(" ")
                des = self.destination_name
                self.next_task_source = sou
            elif data_rec == "Parking":
                self.next_task = "Parking"
        except Exception as e:
            print(e)



        return True

    def pose_correct_on_flow_after_drop(self):
        self.current_state_of_task = 12
        self.current_state_thread = Thread(target=self.bopt_current_state_publisher,
                                           args=(str(self.current_state_of_task),))
        self.current_state_thread.start()
        self.send_data(b"12")


        drive_to_point_and_adjust(self.destination_easy_destination_pose, 'down')
        

        self.thread2 = Thread(target=self.field_back, )
        self.thread2.start()

        self.sequence_complete = True
        return True

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

    def set_source_and_destination(self, source, destination):
        self.source = source
        self.destination = destination
        pub_str = (f"Source set to {source}, destination set to {destination}")
        self.feedback_thread = Thread(target=self.feedback_publisher, args=(pub_str,))  # to change later
        self.feedback_thread.start()

    def euclidean_distance(self, pose1, pose2):

        dx = pose1[0] - pose2[0]
        dy = pose1[1] - pose2[1]

        distance = math.sqrt(dx ** 2 + dy ** 2)
        return distance

    def find_nearest_point(self, station_key, flow_matrix, station_locations):
        '''
        finds nearest point from flow_matrix to our desired location
        '''
        station_coords = station_locations[station_key]

        nearest_point = None
        nearest_distance = None

        for key, value in flow_matrix.items():
            distance = math.sqrt((value[0] - station_coords[0]) ** 2 + (value[1] - station_coords[1]) ** 2)

            # If it's the first point or if it's nearer than the current nearest, update nearest point and distance
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_point = value
                nearest_point_key = key

        return nearest_point_key

    def find_nearest_point_from_current_location_to_flow(self, current_location, flow_matrix):
        '''
        finds nearest point from flow_matrix to our current location
        '''
        station_coords = current_location

        nearest_point = None
        nearest_distance = None
        # print(flow_matrix)
        for key, value in flow_matrix.items():
            distance = math.sqrt((value[0] - station_coords[0]) ** 2 + (value[1] - station_coords[1]) ** 2)

            # If it's the first point or if it's nearer than the current nearest, update nearest point and distance
            if nearest_distance is None or distance < nearest_distance:
                nearest_distance = distance
                nearest_point = value
                nearest_point_key = key

        return nearest_point_key, nearest_distance

    def find_path_from_current_location(self, current_location, station2, flow_matrix, station_locations):
        '''
        Finds path between two stations using the flow matrix as intermediate points
        '''
        path_with_names = []
        path_with_coords = []

        nearest_point_to_station1, _ = self.find_nearest_point_from_current_location_to_flow(current_location,
                                                                                             flow_matrix)
        nearest_point_to_station2 = self.find_nearest_point(station2, flow_matrix, station_locations)
        path_with_names.append(current_location)

        path_with_names = self.forward_count(list(flow_matrix.keys()), nearest_point_to_station1,
                                             nearest_point_to_station2)

        path_with_names.append(station2)
        for pose in path_with_names:
            if isinstance(pose, str):
                path_with_coords.append(station_locations[pose])
            elif isinstance(pose, int):
                path_with_coords.append(flow_matrix[pose])

        return path_with_names, path_with_coords

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

    def navigate_to_pose(self, navigator, pose):
        # time.sleep(0.2)
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.orientation.z = pose[2]
        goal_pose.pose.orientation.w = pose[3]

        navigator.goToPose(goal_pose,
                           behavior_tree="/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/byd_navigate_to_pose_w_replanning_and_recovery.xml")
        # navigator.goToPose(goal_pose, behavior_tree = "/bopt_ws/src/navigation2/nav2_bt_navigator/behavior_trees/byd_navigate_to_pose_w_replanning_and_recovery.xml")

        i = 0
        print('here')

        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                pass
        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')

        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
            return 'Goal was canceled!'
        elif result == TaskResult.FAILED:
            print('Goal failed!')
            return 'Goal failed!'
        else:
            print('Goal has an invalid return status!')
        print(result)

        return result

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
        print("pick_flow---", pick_flow)
        for rows in cur.execute("SELECT * FROM easy_station_loc"):
            easy_station_locations[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
        for rows in cur.execute("SELECT * FROM easy_station_loc_rev"):
            easy_station_location_rev[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]

        for rows in cur.execute("SELECT * FROM doc_station_loc"):
            doc_station_locations[rows[0]] = [float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])]
        print(station_locations, easy_station_locations, flow_matrix, doc_station_locations)

        return station_locations, easy_station_locations, flow_matrix, doc_station_locations, rev_flow_matrix, easy_station_location_rev, flow_matrix_park, pick_flow, rev_pick_flow

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
            # print(msg.data)
            i += 1


    def field_change(self):
        i = 0
        msg = String()
        msg.data = 'pickdrop'
        while i <= 100:
            self.publisher_.publish(msg)
            i += 1

    def dist_checker(self, goal_pose, tolerance):
        current_pose = get_current_pose()

        x_pose = current_pose[0]
        y_pose = current_pose[1]
        # w_pose=pose.pose.pose.position.w
        # print(x_pose,y_pose,cords[0],cords[1])

        dist = (((x_pose - goal_pose.pose.position.x) ** 2) + ((y_pose - goal_pose.pose.position.y) ** 2)) ** 0.5

        # print("goal_tolerance_distance",dist)
        if dist > tolerance:
            return True
        else:
            return False



    def new_navigate_through_flow_poses(self, navigator, path_with_coords, final_pose):
        # print(action)
        goal_poses = []

        if len(path_with_coords[1:-1]) != 0:
            goal_poses = []
            for coord in path_with_coords[1:] + [final_pose]:  # + [easy_destination_pose]:#+ [path_with_coords[-1]]:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = coord[0]
                goal_pose.pose.position.y = coord[1]
                goal_pose.pose.orientation.z = coord[2]
                goal_pose.pose.orientation.w = coord[3]
                goal_poses.append(goal_pose)
        else:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = final_pose[0]
            goal_pose.pose.position.y = final_pose[1]
            goal_pose.pose.orientation.z = final_pose[2]
            goal_pose.pose.orientation.w = final_pose[3]
            goal_poses.append(goal_pose)

        # goal_poses = goal_poses[::2]
        gr = 0
        goal_attempts = {}

        for index, goal_pose in enumerate(goal_poses):
            pose_key = str(goal_pose)
            if pose_key not in goal_attempts:
                goal_attempts[pose_key] = 0

            start_time = time.time()
            timeout_duration = 30  # seconds
            navigator.goToPose(goal_pose,
                               behavior_tree="/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml")

            while index < len(goal_poses) - 1 and self.dist_checker(goal_pose, tolerance=5.5):
                if time.time() - start_time > timeout_duration:
                    if goal_attempts[pose_key] < 6:
                        # Retry the same goal once
                        goal_attempts[pose_key] += 1
                        print(f"Retrying goal due to timeout: {goal_pose}")
                        navigator.goToPose(goal_pose,
                                           behavior_tree="/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml")
                        start_time = time.time()  # Reset timer for retry
                    else:
                        # Move to next goal
                        print(f"Moving to next goal after timeout: {goal_pose}")
                        break
                time.sleep(0.1)  # Sleep to prevent busy waiting

            gr += 1  # Increment the count of reached goals
            continue

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'last pose'
                )

        if gr == len(goal_poses):
            print('Goal succeeded!')
            return TaskResult.SUCCEEDED
        else:
            print('Goal failed!')
            return TaskResult.FAILED

    def navigate_through_poses(self, navigator, poses):

        goal_poses = []
        for pose in poses:  # + [easy_destination_pose]:#+ [path_with_coords[-1]]:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = navigator.get_clock().now().to_msg()
            goal_pose.pose.position.x = pose[0]
            goal_pose.pose.position.y = pose[1]
            goal_pose.pose.orientation.z = pose[2]
            goal_pose.pose.orientation.w = pose[3]
            goal_poses.append(goal_pose)

        # goal_poses = goal_poses[::2]
        gr = 0
        goal_attempts = {}

        for index, goal_pose in enumerate(goal_poses):
            pose_key = str(goal_pose)
            if pose_key not in goal_attempts:
                goal_attempts[pose_key] = 0

            start_time = time.time()
            timeout_duration = 12  # seconds
            navigator.goToPose(goal_pose,
                               behavior_tree="/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml")

            while index < len(goal_poses) - 1 and self.dist_checker(goal_pose, tolerance=2.0):
                if time.time() - start_time > timeout_duration:
                    if goal_attempts[pose_key] < 6:
                        # Retry the same goal once
                        goal_attempts[pose_key] += 1
                        print(f"Retrying goal due to timeout: {goal_pose}")
                        navigator.goToPose(goal_pose,
                                           behavior_tree="/nav2_ws/src/navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml")
                        start_time = time.time()  # Reset timer for retry
                    else:
                        # Move to next goal
                        print(f"Moving to next goal after timeout: {goal_pose}")
                        break
                time.sleep(0.1)  # Sleep to prevent busy waiting

            gr += 1  # Increment the count of reached goals
            continue

        i = 0
        while not navigator.isTaskComplete():
            i = i + 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                print(
                    'last pose'
                )

        if gr == len(goal_poses):
            print('Goal succeeded!')
            return TaskResult.SUCCEEDED
        else:
            print('Goal failed!')
            return TaskResult.FAILED[3]


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
        2: lambda: transporter.precise_control_to_dock(),
        3: lambda: transporter.go_for_pallet_pickup(),
        4: lambda: transporter.drive_back_to_dock(),
        5: lambda: transporter.reorient_to_flow(),
        6: lambda: transporter.pose_correct_on_flow_after_lift(),
        7: lambda: transporter.go_to_drop_off_easy_dock_using_flow(),
        8: lambda: transporter.precise_control_to_drop_off_dock(),
        9: lambda: transporter.go_for_pallet_drop(),
        10: lambda: transporter.drive_back_from_drop_off(),
        11: lambda: transporter.reorient_to_flow_after_drop_off(),
        12: lambda: transporter.pose_correct_on_flow_after_drop()
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
    # transporter.pub_fb("Task Completed")
    # time.sleep(0.5)
    # transporter.pub_fb("Task Completed")
    # time.sleep(0.5)
    # transporter.pub_fb("Task Completed")
    # time.sleep(0.5)
    # time.sleep(3)
    transporter.sequence_complete = True
    print(transporter.next_task)
    if transporter.next_task == "Parking":
        transporter.task_park()

    transporter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


def drive_to_point(destination_pose, fork_status):  # fork_status= 'up' or 'down'
    time.sleep(0.2)
    current_formatted_pose = get_current_pose()
    tf_pose = get_transformed_pose(destination_pose)

    steering_angle, vel, dist = get_precise_control_key(fork_status, tf_pose)

    if cancel_status == False:
        subprocess.run(
            ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
             "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p",
             "target_distance:=" + str(dist)])


def drive_to_point_for_stack(destination_pose, fork_status):  # fork_status= 'up' or 'down'
    time.sleep(0.2)
    current_formatted_pose = get_current_pose()
    tf_pose = get_transformed_pose(destination_pose)

    steering_angle, vel, dist = get_precise_control_key(fork_status, tf_pose)

    if cancel_status == False:
        subprocess.run(
            ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node_with_pallet_check", "my_node",
             "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p",
             "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)])
        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(destination_pose[0]),
                        str(destination_pose[1]), str(destination_pose[2]), str(destination_pose[3])])


def drive_to_point_and_adjust(destination_pose, fork_status):
    time.sleep(0.2)
    current_formatted_pose = get_current_pose()
    tf_pose = get_transformed_pose(destination_pose)

    steering_angle, vel, dist = get_precise_control_key(fork_status, tf_pose)

    if cancel_status == False:
        subprocess.run(
            ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
             "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p",
             "target_distance:=" + str(dist)])
        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(destination_pose[0]),
                        str(destination_pose[1]), str(destination_pose[2]), str(destination_pose[3])])


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


class PalletManager(Node):

    def __init__(self, action, easy_destination_pose, dock_location, pallet_location, publisher_fn):
        self.action = action
        self.easy_destination_pose = easy_destination_pose
        self.operation_complete_flag = False
        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        super().__init__('pallet_manager')
        self.pallet_location = pallet_location
        self.pallet_located = False
        self.pallet_loaded = False
        self.pallet_not_found = False
        self.task_status = 'Task not Completed'
        self.start_time = time.time()
        self.countdown = 20
        self.dock_location = dock_location
        # self.st = safety_thread
        self.state_publisher_fn = publisher_fn

        global cancel_status
        cancel_status = False
        # self.main_execution()

    def main_execution(self):
        if self.action == 'lift':
            self.handle_lift()
        elif self.action == 'drop':
            self.handle_drop()
        else:
            self.get_logger().error(f"Unknown action: {self.action}")

    def main_execution(self):
        # self.get_logger().info(str(self.drive_ahead_flag))
        # print(msg.data.split(' ')[1], self.task_status)
        global task_complete
        result = None
        if self.action == 'lift' and cancel_status == False:
            state = "Driving for Pallet"
            self.state_publisher_fn(state)
            # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
            # thread1.start()
            result = self.run_drive_for_pallet(self.dock_location,
                                               self.pallet_location)  # edit for dist travelled as well
            self.get_logger().info("result drive_for_pallet:" + str(result))
            if result == 'True' and cancel_status == False:  # return proper result
                state = "Lifting Forks"
                self.state_publisher_fn(state)
                # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                # thread1.start()
                self.fork_up()
                if cancel_status == False:
                    state = "Driving Back"
                    self.state_publisher_fn(state)
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                    self.run_drive_back(self.dock_location, self.pallet_location)

                if cancel_status == False:
                    state = "Reorient To Flow"
                    self.state_publisher_fn(state)
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                    self.reorient_to_flow(self.easy_destination_pose, 'up')
                    self.st.start()
                self.state_publisher_fn('Pallet loaded')
                task_complete = True
            else:
                # state="Pallet not found"
                # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                # thread1.start()
                self.task_status = 'Pallet not found'
                self.state_publisher_fn(state)
                self.run_no_pallet_found(self.dock_location)
                self.reorient_to_flow(self.easy_destination_pose, 'down')
                task_complete = True

        if self.operation_complete_flag == False and self.action == 'drop':
            # is_pallet_already_present = self.check_if_pallet_already_present()
            # is_pallet_already_present = False

            # if is_pallet_already_present == False:
            #     if cancel_status==False:
            #         self.run_drop_pallet(self.dock_location, self.pallet_location)
            #     if cancel_status==False:
            #         self.fork_down()
            #     if cancel_status==False:
            #         self.run_drive_back(self.dock_location, self.pallet_location)
            #     if cancel_status==False:
            #         self.reorient_to_flow(self.easy_destination_pose)
            #     if cancel_status==False:
            #         self.task_status = 'Pallet Dropped'
            is_pallet_already_present = False

            if is_pallet_already_present == False:
                if cancel_status == False:
                    state = "Dropping Pallet"
                    self.state_publisher_fn(state)
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                    self.run_drop_pallet(self.dock_location, self.pallet_location)
                if cancel_status == False:
                    self.fork_down()
                    state = "Fork Down"
                    self.state_publisher_fn(state)
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                if cancel_status == False:
                    self.run_drive_back(self.dock_location, self.pallet_location)

                    state = "Drive Back"
                    self.state_publisher_fn(state)
                    # thread1= Thread(
                    # target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                if cancel_status == False:
                    self.reorient_to_flow(self.easy_destination_pose, 'up')
                    self.st.start()
                    state = "Reorient to Flow"
                    self.state_publisher_fn(state)
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                if cancel_status == False:
                    self.task_status = 'Pallet Dropped'
                    self.state_publisher_fn('Pallet Dropped')
                else:
                    self.task_status = 'Manually Cancelled'
                    self.state_publisher_fn('Manually Cancelled')
                self.operation_complete_flag = True
                task_complete = True
                self.destroy_node()
            else:
                task_complete = True
                self.task_status = 'Pallet Already Present'

    def run_no_pallet_found(self):
        current_pose = get_current_pose()
        dist = np.sqrt((self.dock_location[0] - current_pose[0]) ** 2 + (self.dock_location[1] - current_pose[1]) ** 2)

        if cancel_status == False:
            subprocess.run(
                ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
                 "initial_velocity:=" + str(-0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p",
                 "target_distance:=" + str(dist)])

    def run_drop_pallet(self):  # check if pallet is present already and update status
        command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                   str(self.pallet_location[0]), str(self.pallet_location[1]), str(self.pallet_location[2]),
                   str(self.pallet_location[3])]
        run_command_with_retry(command)

        tf_pose = get_transformed_pose(self.pallet_location)
        steering_angle, vel, dist = get_precise_control_key('up', tf_pose)
        print("unclamped", steering_angle, dist)
        steering_angle = clamp(steering_angle, -2.0, 2.0)
        dist = clamp(dist, 1.3, 1.45)
        print("clamped", steering_angle, dist)
        if cancel_status == False:
            result = subprocess.run(
                ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node_with_forktip_check_for_drop",
                 "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p",
                 "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)],
                capture_output=True, text=True)
            if result.returncode == 0:
                self.run_drive_back()
                return False

        if cancel_status == False:
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                       str(self.pallet_location[0]),
                       str(self.pallet_location[1]), str(self.pallet_location[2]), str(self.pallet_location[3])]
            run_command_with_retry(command)

        current_pose = get_current_pose()
        print('pallet dropped at ->', current_pose)

        return True

    def reorient_to_flow(self):
        print("easy_destination_pose---->", self.easy_destination_pose)
        tf_pose = get_transformed_pose(self.easy_destination_pose)
        print(self.action)
        if self.action == 'lift':
            robot_state = 'down'
        else:
            robot_state = 'up'
        steering_angle, vel, dist = get_precise_control_key(robot_state, tf_pose)

        print('reorienting to flow using:', steering_angle, vel, dist)
        vel = -abs(vel)
        command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
                   "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p",
                   "target_distance:=" + str(dist)]
        run_command_with_retry(command)

    def pose_correct_on_flow(self):
        command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                   str(self.easy_destination_pose[0]), str(self.easy_destination_pose[1]),
                   str(self.easy_destination_pose[2]), str(self.easy_destination_pose[3])]
        print(self.easy_destination_pose)
        run_command_with_retry(command)

    def run_drive_for_pallet(self):
        result = None
        if cancel_status == False:
            subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",
                            str(self.pallet_location[0]), str(self.pallet_location[1]),
                            str(self.pallet_location[2]), str(self.pallet_location[3])])
        fetched_m_offset = None
        try:
            fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
            if fetched_angular_offset == None:
                return 'False'
            elif abs(fetched_m_offset) > 0.05:
                check_counter = 0
                while check_counter < 3:
                    if cancel_status == False:
                        subprocess.run(["ros2", "run", "byd_bopt_pallet_picker", "correct_heading_v2",
                                        str(fetched_m_offset)])
                    if cancel_status == False:
                        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp",
                                        "pose_correction_node", str(self.pallet_location[0]),
                                        str(self.pallet_location[1]), str(self.pallet_location[2]),
                                        str(self.pallet_location[3])])
                    fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
                    check_counter += 1
                    if abs(fetched_m_offset) < 0.05:
                        if abs(fetched_angular_offset) > 0.07:
                            command = ["ros2", "run", "byd_pose_correction_node_cpp",
                                       "pose_correction_node_rad", str(-float(fetched_angular_offset))]  # Add Retry
                            run_command_with_retry(command)
                        break
        except Exception as e:
            # Handle any other exceptions
            print("Error, camera can't find pallet:", str(e))
            return 'False'
        # fetched_m_offset, fetched_angular_offset = get_pds_tag_results()

        if fetched_m_offset == None:
            return 'False'
        # fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
        # print('check',fetched_m_offset, fetched_angular_offset)
        command = ["ros2", "run", "byd_bopt_pallet_picker", "drive_for_pallet"]
        try:
            result = run_command_with_retry(command)
            stdout = result.stdout.strip()
            print("result stdout:", stdout)

            if stdout == 'True':
                return 'True'
            else:
                return 'False'
        except Exception as e:
            print(str(e))
            return 'False'

    def run_drive_back(self):  # current_loc
        current_pose = get_current_pose()
        dist = np.sqrt((self.dock_location[0] - current_pose[0]) ** 2 + (self.dock_location[1] - current_pose[1]) ** 2)
        # dist = np.sqrt((dock_location[0] - pallet_location[0]) ** 2 + (dock_location[1]- pallet_location[1]) ** 2)
        # dist+=0.1
        if cancel_status == False:
            subprocess.run(
                ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p",
                 "initial_velocity:=" + str(-0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p",
                 "target_distance:=" + str(dist)])

    def fork_down(self):
        duration = 3  # 3 seconds
        command = "ros2 service call /byd/send_command example_interfaces/srv/Command \"{command: 'down'}\""

        # subprocess.call() is blocking by default
        retcode = subprocess.call(command, shell=True)

    def fork_up(self):
        duration = 3  # 3 seconds
        command = "ros2 service call /byd/send_command example_interfaces/srv/Command \"{command: 'up'}\""

        # subprocess.call() is blocking by default
        retcode = subprocess.call(command, shell=True)

        bus = can.interface.Bus(channel="can0", bustype='socketcan')
        message_id = 0x209
        try:
            message = bus.recv()
            if message.arbitration_id == message_id:
                print("REcived_fork_can_message", message)
                data = message.data
                print("data_fork_can", data)
        except Exception as e:
            print(e)


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
