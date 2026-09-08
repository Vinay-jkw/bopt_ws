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
from threading import Thread
from rclpy.clock import ROSClock
from tf2_ros import TransformListener, Buffer
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from rclpy.qos import QoSProfile, QoSReliabilityPolicy,QoSHistoryPolicy



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

