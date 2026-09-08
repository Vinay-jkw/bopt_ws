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
from geometry_msgs.msg import PoseStamped,Twist
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
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy,QoSHistoryPolicy
from collections import deque
import psutil


ws_path = os.getenv('WS_PATH')
databasepath = '/home/lenovo/bopt_ws/src/task_allocator/Vehicles.db'#os.getenv('DATABASE_PATH_VEH')
databasepathuser=os.getenv('DATABASE_PATH_USERS')
bt_path=os.getenv('BT_PATH')

global cancel_status


class SafetyPublisher(Node):
    def __init__(self):
        super().__init__('Task_allocator')
        self.publisher_ = self.create_publisher(String, '/byd/safety', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.task_publisher = self.create_publisher(String, '/byd/current_task', 10)
        self.state_publisher = self.create_publisher(String, '/byd/status', 10)
        print("publishers_created")

start_ack=False
finish_ack=False
    

class MySubscriber(Node):
    def __init__(self):
        super().__init__('my_subscriber_node')
        self.subscription = self.create_subscription(
            String,
            '/load_transporter_task_status',
            self.topic_callback,
            10
        )
        self.subscription  # prevent unused variable warning

    def topic_callback(self,msg):
        print("------------",msg.data,"-----------------")
        if msg.data=="Task_started_recived":
            global start_ack
            start_ack=True

        if msg.data=="Task_finished_recived":
            global finish_ack
            finish_ack=True

def run_subscriber():
    # rclpy.init()
    my_subscriber = MySubscriber()
    while rclpy.ok():
        rclpy.spin_once(my_subscriber)
    my_subscriber.destroy_node()
    rclpy.shutdown()



class LoadTransporter(Node):
    def __init__(self):
        super().__init__('load_transporter')
        
        
        self.ltn_feedback_publisher = self.create_publisher(String, '/load_transporter_feedback', 10)
        self.ltn_task_publisher = self.create_publisher(String, '/load_transporter_current_task', 10)
        self.ltn_ack_subscriber = self.create_subscription(String, '/load_transporter_ack_receiver', 10)
        self.subscription = self.create_subscription(
            String,
            '/load_transporter_task_status',
            self.topic_callback,
            10
        )
        self.subscription

        self.source = None
        self.destination = None
        self.databasepath = databasepath

        self.station_locations, self.easy_station_locations, self.flow_matrix, self.dock_station_locations = self.get_locations_and_waypoints()

        self.navigator = BasicNavigator()
        publisher = SafetyPublisher()
        self.publisher_ = publisher.publisher_ #safety pub
        self.cmd_vel_publ = publisher.cmd_vel_pub
        self.t_publisher = publisher.task_publisher
        self.state_pub = publisher.state_publisher
        self.sequence_complete = False
        self.bt_path = bt_path
        global cancel_status 
        # subscriber_thread = Thread(target=run_subscriber)
        # subscriber_thread.start()
        self.current_pose = None
        self.success = False
        self.start_ack=False
        self.finish_ack=False


    def topic_callback(self,msg):
        print("------------",msg.data,"-----------------")
        if msg.data=="Task_started_recived":
            self.start_ack=True

        if msg.data=="Task_finished_recived":
            self.finish_ack=True    

    def feedback_publisher(self, feedback):
        i=0
        msg = String()
        msg.data = feedback
        print('Publishing: "%s"' % msg.data)
        while i<=100:
            self.ltn_feedback_publisher.publish(msg)
            i+=1
    
    def task_publisher(self, feedback):
        i=0
        msg = String()
        msg.data = feedback
        print('Publishing: "%s"' % msg.data)
        while i<=100:
            self.ltn_task_publisher.publish(msg)
            i+=1


    def set_source_and_destination(self, source, destination):
        self.source = source
        self.destination = destination
        pub_str = (f"Source set to {source}, destination set to {destination}")
        self.feedback_thread = Thread(target=self.feedback_publisher, args=(pub_str,)) #to change later
        self.feedback_thread.start()



    def euclidean_distance(self,pose1, pose2):

        dx = pose1[0] - pose2[0]
        dy = pose1[1] - pose2[1]

        distance = math.sqrt(dx**2 + dy**2)
        return distance
    
    def find_nearest_point(self,station_key, flow_matrix, station_locations):
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

    def find_nearest_point_from_current_location_to_flow(self,current_location, flow_matrix):
        '''
        finds nearest point from flow_matrix to our current location
        '''
        station_coords = current_location

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

    def find_path_from_current_location(self,current_location, station2, flow_matrix, station_locations):
        '''
        Finds path between two stations using the flow matrix as intermediate points
        '''
        path_with_names = []
        path_with_coords = []

        nearest_point_to_station1 = self.find_nearest_point_from_current_location_to_flow(current_location, flow_matrix)
        nearest_point_to_station2 = self.find_nearest_point(station2, flow_matrix, station_locations)
        path_with_names.append(current_location)

        if nearest_point_to_station2 >= nearest_point_to_station1:
            station_counter = nearest_point_to_station1
            path_with_names.append(nearest_point_to_station1)
            while station_counter != nearest_point_to_station2:
                station_counter += 1
                path_with_names.append(station_counter)
        else:
            station_counter = nearest_point_to_station1
            path_with_names.append(nearest_point_to_station1)
            while station_counter != (nearest_point_to_station2 + len(flow_matrix)):
                station_counter += 1
                if station_counter >= len(flow_matrix):
                    path_with_names.append(station_counter - len(flow_matrix))
                else:
                    path_with_names.append(station_counter)

        path_with_names.append(station2)
        for pose in path_with_names:
            if isinstance(pose, str):
                path_with_coords.append(station_locations[pose])
            elif isinstance(pose, int):
                path_with_coords.append(flow_matrix[pose])

        return path_with_names, path_with_coords
    
    def navigate_to_pose(self, navigator, pose):        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = pose[0]
        goal_pose.pose.position.y = pose[1]
        goal_pose.pose.orientation.z = pose[2]
        goal_pose.pose.orientation.w = pose[3]

        navigator.goToPose(goal_pose, behavior_tree = ws_path + "src/navigation2/nav2_bt_navigator/behavior_trees/byd_navigate_to_pose_w_replanning_and_recovery.xml")
        # navigator.goToPose(goal_pose, behavior_tree = "/bopt_ws/src/navigation2/nav2_bt_navigator/behavior_trees/byd_navigate_to_pose_w_replanning_and_recovery.xml")

        i = 0
            
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
        station_locations={}
        flow_matrix={}
        easy_station_locations={}
        doc_station_locations={}
     
        for rows in cur.execute("SELECT * FROM location"):
            station_locations[rows[0]]=[float(rows[1]),float(rows[2]),float(rows[3]),float(rows[4])]
        i=0
        for rows in cur.execute("SELECT * FROM waypoints"):
            flow_matrix[i]=[float(rows[1]),float(rows[2]),float(rows[3]),float(rows[4])]
            i+=1
        for rows in cur.execute("SELECT * FROM easy_station_loc"):
            easy_station_locations[rows[0]]=[float(rows[1]),float(rows[2]),float(rows[3]),float(rows[4])]
        for rows in cur.execute("SELECT * FROM doc_station_loc"):
            doc_station_locations[rows[0]]=[float(rows[1]),float(rows[2]),float(rows[3]),float(rows[4])]
        print(station_locations, easy_station_locations, flow_matrix, doc_station_locations)
        
        return station_locations, easy_station_locations, flow_matrix, doc_station_locations
    
    def pub_fb(self,fb_string):
        pub_str = fb_string
        self.feedback_thread = Thread(target=self.feedback_publisher, args=(pub_str,)) #to change later 
        self.feedback_thread.start()
    
    # def sub_fb(self):


    def go_for_pickup(self, destination):
        self.pub_fb("Task Started")
        self.pub_fb("Task Started")
        # while ack != Received:
        #     self.pub_fb("Task Started")
        #     ack = self.sub_ta()
        global start_ack
        while start_ack != True:
            self.pub_fb("Task Started")
        time.sleep(1)

        cancel_status = False

        self.pub_fb("Localizing")
        current_formatted_pose = get_current_pose()
        source_pose = current_formatted_pose
        easy_destination_pose = self.easy_station_locations[destination]
        destination_pose = self.dock_station_locations[destination]
        distance_between_goals = self.euclidean_distance(easy_destination_pose, destination_pose)
        station_pose = self.station_locations[destination]  # Placeholder for actual function
        flow_matrix = self.flow_matrix

        
        # 3. Navigate through waypoints
        self.pub_fb("Navigating through waypoints")
        try:
            #find path to station 
            path_with_names, path_with_coords = self.find_path_from_current_location(source_pose, destination, flow_matrix, self.easy_station_locations)
            # path_with_coords = path_with_coords + [easy_destination_pose]
            print('path found:', path_with_names ,path_with_coords)
            result_through_poses=None
            if len(path_with_coords) > 2:#check
                result_through_poses = self.new_navigate_through_flow_poses(self.navigator, path_with_coords, easy_destination_pose)
                # self.follow_path_through_flow(navigator, path_with_coords, easy_destination_pose)
        except Exception as e:
            # Handle any other exceptions
            self.pub_fb("Error, can't find path:")   

        
        if cancel_status==False: #front field off
            self.pub_fb('navigating to dock')
            result_to_pose = self.navigate_to_pose(self.navigator, easy_destination_pose)
        else:
            self.thread1= Thread(target=self.field_back,) #switching safety on
            self.thread1.start()
            print("ctask", cancel_status)
            self.pub_fb("Goal Was Cancelled")
            return "Goal Was Cancelled"
        
        if result_to_pose == TaskResult.SUCCEEDED:
            self.pub_fb('at station dock')
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",str(easy_destination_pose[0]), str(easy_destination_pose[1]), str(easy_destination_pose[2]), str(easy_destination_pose[3])]
            run_command_with_retry(command)
            # self.state=robot_state
            # thread1= Thread(target=self.state_publisher,args=(self.state,)) #switching safety on
            # thread1.start()

            self.thread1= Thread(target=self.field_change,) #to change later
            self.thread1.start()
            time.sleep(1)
            current_formatted_pose = get_current_pose()
            tf_pose = get_transformed_pose(destination_pose)

            steering_angle, vel, dist = get_precise_control_key('down', tf_pose)

            # if cancel_status==False:
            self.pub_fb("Navigating to Dock Pose")
            
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)]
            run_command_with_retry(command)
            time.sleep(1)
            # subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(destination_pose[0]), str(destination_pose[1]), str(destination_pose[2]), str(destination_pose[3])]) 
            # time.sleep(1)
            global task_complete
            self.thread2= Thread(target=self.field_back,)
            # if cancel_status==False:              
            pallet_manager = PalletManager(action = 'lift', distance_between_goals = distance_between_goals, easy_destination_pose = easy_destination_pose,dock_location = destination_pose, pallet_location = station_pose, vel=vel, dist=dist, steering=steering_angle, destination = destination, safety_thread=self.thread2, publisher_fn=self.pub_fb)

            pallet_manager.main_execution()
            task_complete = False
            # if pallet_manager.task_status == 'Pallet not found':
            #     #reorient to flow

            #switching field on
            self.thread2= Thread(target=self.field_back,)
            self.thread2.start()
            # if stations = t1, t2 navigate to nearest point in flow
            if destination == "T1" or destination == "T2" and cancel_status == False:
                print('going to flow')
                #Find current Location
                current_formatted_pose = get_current_pose()
                robot_state = 'Going to Flow'
                self.state=robot_state
                
                nearest_flow_point_to_station = self.find_nearest_point_from_current_location_to_flow(current_formatted_pose, flow_matrix)
                result_to_pose = self.navigate_to_pose(self.navigator, flow_matrix[nearest_flow_point_to_station-1])
                result_to_pose = self.navigate_to_pose(self.navigator, flow_matrix[nearest_flow_point_to_station+1])
            if pallet_manager.task_status!=None:
                return pallet_manager.task_status
            else:
                self.pub_fb("Goal Was Manually Cancelled")
                return "Goal Was Manually Cancelled"

            # else:
            #     task_complete = False
            #     self.thread2= Thread(target=self.field_back,)
            #     self.thread2.start()
        else:
            return result_through_poses

    def go_for_drop(self, destination):
        self.pub_fb("Localizing")
        current_formatted_pose = get_current_pose()
        source_pose = current_formatted_pose
        easy_destination_pose = self.easy_station_locations[destination]
        destination_pose = self.dock_station_locations[destination]
        distance_between_goals = self.euclidean_distance(easy_destination_pose, destination_pose)
        station_pose = self.station_locations[destination]  # Placeholder for actual function
        flow_matrix = self.flow_matrix


        # 3. Navigate through waypoints
        self.pub_fb("Navigating through waypoints")
        try:
            #find path to station 
            path_with_names, path_with_coords = self.find_path_from_current_location(source_pose, destination, flow_matrix, self.easy_station_locations)
            # path_with_coords = path_with_coords + [easy_destination_pose]
            print(path_with_coords)
            result_through_poses=None
            if len(path_with_coords) > 2:#check
                robot_state = 'navigating through flow'
                self.state=robot_state
               
                result_through_poses = self.new_navigate_through_flow_poses(self.navigator, path_with_coords, easy_destination_pose)
                # self.follow_path_through_flow(navigator, path_with_coords, easy_destination_pose)
        except Exception as e:
            # Handle any other exceptions
            self.pub_fb("can't find path")
            print("Error, can't find path:", str(e))   

        #Go to station dock
        # if cancel_status==False: #front field off
        robot_state = 'navigating to dock'
        self.state=robot_state
        
        result_to_pose = self.navigate_to_pose(self.navigator, easy_destination_pose)
        # else:
            # self.thread1= Thread(target=self.field_back,) #switching safety on
            # self.thread1.start()
            # print("ctask", cancel_status)
            # self.pub_fb("Goal Was Cancelled")
            # return "Goal Was Cancelled"
        
        if result_to_pose == TaskResult.SUCCEEDED:
            robot_state = 'at station dock'
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",str(easy_destination_pose[0]), str(easy_destination_pose[1]), str(easy_destination_pose[2]), str(easy_destination_pose[3])]
            run_command_with_retry(command)
            print('correcting pose')

            self.state=robot_state
            
            self.thread1= Thread(target=self.field_change,) #to change later
            self.thread1.start()
            time.sleep(1)
            
            current_formatted_pose = get_current_pose()
            tf_pose = get_transformed_pose(destination_pose)


 
            steering_angle, vel, dist = get_precise_control_key('up', tf_pose)


            # if cancel_status==False:
            robot_state="Navigating to Dock Pose"
            self.state=robot_state
            
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)]
            run_command_with_retry(command)
            time.sleep(1)
            # subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(destination_pose[0]), str(destination_pose[1]), str(destination_pose[2]), str(destination_pose[3])]) 
            # time.sleep(1)
            global task_complete
            self.thread2= Thread(target=self.field_back,)
            # if cancel_status==False:              
            pallet_manager = PalletManager(action = 'drop', distance_between_goals = distance_between_goals, easy_destination_pose = easy_destination_pose,dock_location = destination_pose, pallet_location = station_pose, vel=vel, dist=dist, steering=steering_angle, destination = destination, safety_thread=self.thread2, publisher_fn=self.pub_fb)

            pallet_manager.main_execution()
            task_complete = False
            # if pallet_manager.task_status == 'Pallet not found':
            #     #reorient to flow

            #switching field on
            self.thread2= Thread(target=self.field_back,)
            self.thread2.start()
            # if stations = t1, t2 navigate to nearest point in flow
            if destination == "T1" or destination == "T2" and cancel_status == False:
                print('going to flow')
                #Find current Location
                current_formatted_pose = get_current_pose()
                robot_state = 'Going to Flow'
                self.state=robot_state
                
                nearest_flow_point_to_station = self.find_nearest_point_from_current_location_to_flow(current_formatted_pose, flow_matrix)
                result_to_pose = self.navigate_to_pose(self.navigator, flow_matrix[nearest_flow_point_to_station-1])
                result_to_pose = self.navigate_to_pose(self.navigator, flow_matrix[nearest_flow_point_to_station+1])
            if pallet_manager.task_status!=None:
                return pallet_manager.task_status
            else:
                self.pub_fb("Goal Was Manually Cancelled")
                return "Goal Was Manually Cancelled"

            # else:
            #     task_complete = False
            #     self.thread2= Thread(target=self.field_back,)
            #     self.thread2.start()
        else:
            return result_through_poses


    def field_back(self):
        i=0
        msg = String()
        msg.data = 'lock'
        while i<=100:
            self.publisher_.publish(msg)
            # print(msg.data)
            i+=1
        
    


    def start_transport_sequence(self):
        task_str = "Pickup at:" + self.source + " and Drop at:" + self.destination
        self.task_thread = Thread(target=self.task_publisher, args=(task_str,)) #to change later
        self.task_thread.start()

        go_for_pickup_status = self.go_for_pickup(self.source)
        print(go_for_pickup_status, ":go_for_pickup_status")

        go_for_drop_status = self.go_for_drop(self.destination)
        print(go_for_drop_status, ":go_for_drop_status")
        if go_for_drop_status == "Pallet Dropped":
            self.pub_fb("Task Completed")
            self.pub_fb("Task Completed")
            self.pub_fb("Task Completed")
            global finish_ack
            while finish_ack != True:
                self.pub_fb("Task Completed")
            time.sleep(3)
            self.sequence_complete = True
        else:
            self.pub_fb("Task Not Completed")
            time.sleep(3)
            self.sequence_complete = True


    def field_change(self):
        i=0
        msg = String()
        msg.data = 'pickdrop'
        while i<=100:
            self.publisher_.publish(msg)
            i+=1
            

    def dist_checker(self,goal_pose, tolerance):
        current_pose = get_current_pose()
        
        x_pose=current_pose[0]
        y_pose=current_pose[1]
        # w_pose=pose.pose.pose.position.w
        # print(x_pose,y_pose,cords[0],cords[1])

        dist= (((x_pose - goal_pose.pose.position.x)**2)+((y_pose - goal_pose.pose.position.y)**2))**0.5
        # print(dist)
        if dist>tolerance:
            return True
        else:
            return False     
    

    

    def new_navigate_through_flow_poses(self, navigator, path_with_coords, easy_destination_pose):
        # print(action)
        goal_poses = []

        if len(path_with_coords[1:-2]) != 0:
            goal_poses = []
            for coord in path_with_coords[1:-2]:# + [easy_destination_pose]:#+ [path_with_coords[-1]]:
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = navigator.get_clock().now().to_msg()
                goal_pose.pose.position.x = coord[0]
                goal_pose.pose.position.y = coord[1]
                goal_pose.pose.orientation.z = coord[2]
                goal_pose.pose.orientation.w = coord[3]
                goal_poses.append(goal_pose)


        gr = 0
        for goal_pose in goal_poses:
            # if cancel_status==False:
            navigator.goToPose(goal_pose, behavior_tree=ws_path + "src/navigation2/nav2_bt_navigator/behavior_trees/byd_navigate_to_pose_w_replanning_and_recovery.xml")
            # navigator.goToPose(goal_pose, behavior_tree="/bopt_ws/src/navigation2/nav2_bt_navigator/behavior_trees/byd_navigate_to_pose_w_replanning_and_recovery.xml")

            # if goal_pose != goal_poses[-1]:
            while self.dist_checker(goal_pose, tolerance=2.9):
                # print("Not reached")
                pass
            gr+=1
            continue
            #     print(str(goal_pose) + " Reached")
            # else:
            #     gr+=1
            #     print('at final goal')

        if gr == len(goal_poses):
            print('Goal succeeded!')
            return TaskResult.SUCCEEDED
        else:
            print('Goal failed!')
            return TaskResult.FAILED



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
    i=0
    while i<=100:
        cmd_vel_publisher.publish(stop_msg)
        state_publisher.publish(state_msg)
        i+=1

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
        print("Usage: ros2 run <your_package_name> load_transporter_node <source> <destination>")
        rclpy.shutdown()
        return

    source = sys.argv[1]
    destination = sys.argv[2]

    # Create an instance of LoadTransporter and set source and destination
    transporter = LoadTransporter()
    transporter.set_source_and_destination(source, destination)

    # Start the transport sequence
    transporter.start_transport_sequence()

    print('transporter status:', transporter.sequence_complete)
    
    # if transporter.sequence_complete
    # Keep running until the sequence is complete or ROS is shut down
    # while rclpy.ok():
    #     rclpy.spin_once(transporter)
    #     if transporter.sequence_complete:
    #         break

    transporter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


def drive_to_point(destination_pose, fork_status):#fork_status= 'up' or 'down'
    time.sleep(1)
    current_formatted_pose = get_current_pose()
    tf_pose = get_transformed_pose(destination_pose)

    steering_angle, vel, dist = get_precise_control_key(fork_status, tf_pose)

    if cancel_status==False:
        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)])
        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",str(destination_pose[0]), str(destination_pose[1]), str(destination_pose[2]), str(destination_pose[3])]) 



def run_thr(node_to_run):
    while rclpy.ok():
        rclpy.spin_once(node_to_run,timeout_sec=1.0)
        if node_to_run.current_pose is not None:
            break
    node_to_run.destroy_node()


def get_current_pose():
    node = CurrentLocationNode()
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.current_pose is not None:
            break
    print("----current_pose_executed------------------")    
    node.destroy_node()
    
    current_pose = node.current_pose

    current_formatted_pose = [current_pose.position.x, current_pose.position.y, current_pose.orientation.z, current_pose.orientation.w]
    # print('fetched current pose:',current_formatted_pose)
    # node.destroy_node()
    return current_formatted_pose

def get_transformed_pose(location):
    tf2_broadcaster = TF2Broadcaster(location[0], location[1], location[2], location[3])
    timeout_counter = 0
    while tf2_broadcaster.get_pose() is None and timeout_counter < 100:
        rclpy.spin_once(tf2_broadcaster)
        timeout_counter += 1

    tf_pose = tf2_broadcaster.get_pose()
    tf2_broadcaster.destroy_node()
    return tf_pose

def find_nearest_key(lookup_table, position, initial_tolerance=0.01, max_tolerance=0.1, tolerance_increment=0.01):
    tolerance = initial_tolerance

    while tolerance <= max_tolerance:
        print('current tolerance:',tolerance)
        potential_keys = []
        
        # Gather all keys within the current tolerance
        for key in lookup_table.keys():
            position_difference = np.sqrt((key[0] - position[0]) ** 2 + (key[1] - position[1]) ** 2)
            vel = lookup_table[key][1]

            # Check if the key satisfies the additional condition on velocity
            if position_difference <= tolerance and ((position[0] > 0 and vel > 0) or (position[0] <= 0 and vel <=0)):
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
        return 0.0,0.0,0.0
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

    def __init__(self, action, distance_between_goals, easy_destination_pose, dock_location, pallet_location, vel, dist, steering, destination,safety_thread,publisher_fn):
        self.action = action
        self.destination = destination
        self.distance_between_goals = distance_between_goals
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
        self.vel = -vel
        self.dist = dist
        self.steering_angle = steering
        self.dock_location = dock_location
        self.st = safety_thread
        self.state_publisher_fn = publisher_fn

        global cancel_status
        cancel_status = False
        # self.main_execution()

    def main_execution(self):
        # self.get_logger().info(str(self.drive_ahead_flag))
        # print(msg.data.split(' ')[1], self.task_status)
        global task_complete
        result=None
        if self.action == 'lift' and cancel_status == False:
            state="Driving for Pallet"
            self.state_publisher_fn(state)            
            # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
            # thread1.start()
            result = self.run_drive_for_pallet(self.dock_location, self.pallet_location) #edit for dist travelled as well
            self.get_logger().info("result drive_for_pallet:" + str(result))
            if result=='True' and cancel_status==False: #return proper result
                state="Lifting Forks"
                self.state_publisher_fn(state)           
                # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                # thread1.start()
                self.fork_up()
                if cancel_status==False:
                    state="Driving Back"
                    self.state_publisher_fn(state) 
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                    self.run_drive_back(self.dock_location, self.pallet_location)
                    
                if cancel_status==False:
                    state="Reorient To Flow"
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
                if cancel_status==False:
                    state="Dropping Pallet"
                    self.state_publisher_fn(state) 
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                    self.run_drop_pallet(self.dock_location, self.pallet_location) 
                if cancel_status==False:
                    self.fork_down()
                    state="Fork Down"
                    self.state_publisher_fn(state) 
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                if cancel_status==False:
                    self.run_drive_back(self.dock_location, self.pallet_location)
                    
                    state="Drive Back"
                    self.state_publisher_fn(state) 
                    # thread1= Thread(
                    # target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                if cancel_status==False:
                    self.reorient_to_flow(self.easy_destination_pose, 'up')
                    self.st.start() 
                    state="Reorient to Flow"
                    self.state_publisher_fn(state) 
                    # thread1= Thread(target=self.publisher_fn,args=(state,)) #switching safety on
                    # thread1.start()
                if cancel_status==False:
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



    def check_if_pallet_already_present(self):
        #use odot to see if pallet already present
        front_sensor_byte, back_sensor_byte = get_current_odot_state()
        if front_sensor_byte != '3':
            return True
        else:
            return False
        
    def run_no_pallet_found(self, dock_location):
        current_pose = get_current_pose()
        dist = np.sqrt((dock_location[0] - current_pose[0]) ** 2 + (dock_location[1]- current_pose[1]) ** 2)
        # dist+=0.1
        
        if cancel_status==False:
            subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(-0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p", "target_distance:=" + str(dist)])

    def run_drop_pallet(self, dock_location, pallet_location): #check if pallet is present already and update status
        command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(pallet_location[0]), str(pallet_location[1]), str(pallet_location[2]), str(pallet_location[3])]
        run_command_with_retry(command)
        # if self.destination == 'T5':
        #     subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p", "target_distance:=" + str(1.2)])
        # else:
        tf_pose = get_transformed_pose(pallet_location)
        steering_angle, vel, dist = get_precise_control_key('up', tf_pose)
        if cancel_status==False:
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)]
            run_command_with_retry(command) 
        if cancel_status==False:
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(pallet_location[0]), str(pallet_location[1]), str(pallet_location[2]), str(pallet_location[3])]
            run_command_with_retry(command)
        current_pose = get_current_pose()
        print('pallet dropped at ->', current_pose)


    
    def reorient_to_flow(self, easy_destination_pose, action):
        time.sleep(1)
        current_formatted_pose = get_current_pose()
        tf_pose = get_transformed_pose(easy_destination_pose)

        steering_angle, vel, dist = get_precise_control_key(action, tf_pose)

        print('reorienting to flow using:',steering_angle, vel, dist)
        vel = -abs(vel)
        if cancel_status==False:
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(vel), "-p", "initial_steering_angle:=" + str(steering_angle), "-p", "target_distance:=" + str(dist)]
            run_command_with_retry(command)
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node",str(easy_destination_pose[0]), str(easy_destination_pose[1]), str(easy_destination_pose[2]), str(easy_destination_pose[3])]
            run_command_with_retry(command)

    def run_drive_for_pallet(self, dock_location, pallet_location):
        result=None
        if cancel_status==False:
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(pallet_location[0]), str(pallet_location[1]), str(pallet_location[2]), str(pallet_location[3])]
            run_command_with_retry(command)

        # try: 
        #     distance_to_midpoint, tensor_value, fetched_m_offset = get_pds_results()
        #     if abs(fetched_m_offset) > 0.04:
        #         check_counter = 0
        #         while check_counter < 3:
        #             if cancel_status==False:
        #                 subprocess.run(["ros2", "run", "byd_bopt_pallet_picker", "correct_heading", str(fetched_m_offset)])
        #             if cancel_status==False:
        #                 subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(pallet_location[0]), str(pallet_location[1]), str(pallet_location[2]), str(pallet_location[3])]) 
        #             distance_to_midpoint, tensor_value, fetched_m_offset = get_pds_results()
        #             check_counter += 1
        #             if abs(fetched_m_offset) < 0.04:
        #                 break
        #     else:
        #         pass 
        # except Exception as e:
        #     # Handle any other exceptions
        #     print("Error, camera can't find pallet:", str(e))   
        try: 
            fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
            if abs(fetched_m_offset) > 0.05:
                check_counter = 0
                while check_counter < 6:
                    if cancel_status==False:
                        subprocess.run(["ros2", "run", "byd_bopt_pallet_picker", "correct_heading", str(fetched_m_offset)])
                    if cancel_status==False:
                        subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node", str(pallet_location[0]), str(pallet_location[1]), str(pallet_location[2]), str(pallet_location[3])]) 
                    fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
                    check_counter += 1
                    if abs(fetched_m_offset) < 0.05:
                        break
            else:
                pass 
        except Exception as e:
            # Handle any other exceptions
            print("Error, camera can't find pallet:", str(e))   
        fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
        if abs(fetched_angular_offset) > 0.07:
            command = ["ros2", "run", "byd_pose_correction_node_cpp", "pose_correction_node_rad", str(-float(fetched_angular_offset))]
            run_command_with_retry(command)

        # fetched_m_offset, fetched_angular_offset = get_pds_tag_results()
        # print('check',fetched_m_offset, fetched_angular_offset)
        if cancel_status==False:
            # result = 
            command = ["ros2", "run", "byd_bopt_pallet_picker", "drive_for_pallet"]
            result = run_command_with_retry(command)
            stdout = result.stdout.strip()
            print("result stdout:", stdout)
            # self.get_logger().info("result stdout:" + str(result.stdout))
        return 'True'
        # if str(result.stdout).strip() == 'True': #if pallet found proceed as intended if not return to flow
        #     return 'True'
        # # elif result.stdout == 'False':
        # else:
        #     return 'False'
        
    
    def run_drive_back(self, dock_location, pallet_location): #current_loc
        current_pose = get_current_pose()
        dist = np.sqrt((dock_location[0] - current_pose[0]) ** 2 + (dock_location[1]- current_pose[1]) ** 2)
        # dist = np.sqrt((dock_location[0] - pallet_location[0]) ** 2 + (dock_location[1]- pallet_location[1]) ** 2)
        # dist+=0.1
        if cancel_status==False:
            subprocess.run(["ros2", "run", "byd_pose_correction_node_cpp", "precise_control_node", "my_node", "--ros-args", "-p", "initial_velocity:=" + str(-0.2), "-p", "initial_steering_angle:=" + str(0.0), "-p", "target_distance:=" + str(dist)])

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

    result_pds = subprocess.run(["ros2", "run", "pallet_detection", "pallet_detection_tag"], capture_output=True, text=True)
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

import subprocess
import time

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
            print(f"Command timed out (attempt {attempt + 1}/{max_retries}). Retrying in {delay_between_retries} seconds...")
            process.kill()
            process.wait()  # Ensure the process has terminated before retrying
            time.sleep(delay_between_retries)
    
    # If all retries fail, raise an exception
    raise Exception(f"Command '{' '.join(command)}' failed after {max_retries} retries.")





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
        self.back_sensor_byte = sensor_data[0] #back sensors
        self.front_sensor_byte = sensor_data[-1] #forktip sensors
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
    
    def broadcast_tf(self,msg):

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
            print(transform.transform.translation.x,transform.transform.translation.y,transform.transform.rotation.z,transform.transform.rotation.w)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().info('No transform available: ' + str(e))

