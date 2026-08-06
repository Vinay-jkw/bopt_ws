#!/usr/bin/env python3
"""
BOPT Robot Controller Node
==========================
High-level bridge between standard ROS2 interfaces and the robot's
ros2_control-based independent drive/steering/lift controllers.

Subscriptions:
  /cmd_vel        (geometry_msgs/Twist)   – linear.x [m/s] + angular.z [rad/s]
  /lift_cmd       (std_msgs/Float64)      – desired lift height [0.0 … 0.095 m]

Publications:
  /drive_velocity_controller/commands     (std_msgs/Float64MultiArray)
  /steering_position_controller/commands  (std_msgs/Float64MultiArray)
  /lift_controller/commands               (std_msgs/Float64MultiArray)
  /robot_status                           (std_msgs/String)

Parameters:
  wheelbase        (float, default 0.07483) – Distance from rear axle to drive wheel (m)
  wheel_radius     (float, default 0.115)   – Radius of drive wheel (m)
"""

import json
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, String


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('max_linear_vel',  1.0)
        self.declare_parameter('max_angular_vel', 1.5)
        self.declare_parameter('lift_min',        0.0)
        self.declare_parameter('lift_max',        0.095)
        self.declare_parameter('lift_speed',      0.02)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        
        # Tricycle kinematics parameters
        self.declare_parameter('wheelbase',    0.07483)
        self.declare_parameter('wheel_radius', 0.115)

        self._max_lin    = self.get_parameter('max_linear_vel').value
        self._max_ang    = self.get_parameter('max_angular_vel').value
        self._lift_min   = self.get_parameter('lift_min').value
        self._lift_max   = self.get_parameter('lift_max').value
        self._lift_speed = self.get_parameter('lift_speed').value
        self._timeout    = self.get_parameter('cmd_vel_timeout').value
        
        self._wheelbase    = self.get_parameter('wheelbase').value
        self._wheel_radius = self.get_parameter('wheel_radius').value

        # ── Internal state ───────────────────────────────────────────────
        self._last_cmd_time    = self.get_clock().now()
        self._joint_states     = {}
        self._current_lift_pos = 0.0
        self._lift_setpoint    = 0.0
        self._lift_target      = 0.0
        
        # Store last desired steering angle so it doesn't snap back to 0 when we stop
        self._last_steering_angle = 0.0

        # ── Publishers ───────────────────────────────────────────────────
        self._drive_pub = self.create_publisher(
            Float64MultiArray, '/drive_velocity_controller/commands', 10)
            
        self._steer_pub = self.create_publisher(
            Float64MultiArray, '/steering_position_controller/commands', 10)

        self._lift_pub = self.create_publisher(
            Float64MultiArray, '/lift_controller/commands', 10)

        self._status_pub = self.create_publisher(
            String, '/robot_status', 10)

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_cb, 10)

        self.create_subscription(
            Float64, '/lift_cmd', self._lift_cmd_cb, 10)

        self.create_subscription(
            JointState, '/joint_states', self._joint_states_cb, 10)

        # ── Timers ────────────────────────────────────────────────────────
        self._lift_dt    = 0.02
        self._lift_timer = self.create_timer(self._lift_dt, self._lift_ramp_cb)
        self._watchdog_timer = self.create_timer(0.1, self._watchdog_cb)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info('BOPT Custom Tricycle Kinematics Controller started.')

    def _cmd_vel_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()

        # Clamp inputs
        v = self._clamp(msg.linear.x,  -self._max_lin, self._max_lin)
        w = self._clamp(msg.angular.z, -self._max_ang, self._max_ang)

        # --- Tricycle Kinematics ---
        # If we have no linear velocity, but want to spin in place, 
        # a standard tricycle can't do it unless we turn the wheel to 90 degrees.
        # But for normal driving:
        
        if abs(v) > 1e-4 or abs(w) > 1e-4:
            # BOPT is a REAR-drive-wheel vehicle (forklift style).
            # To turn the front of the vehicle left (+w), the rear wheel 
            # must swing to the right (negative steering angle).
            # So we invert the angular velocity for the kinematics calculation.
            w_kin = -w
            
            if abs(v) < 1e-4:
                # Spinning in place (roughly)
                steer_angle = math.copysign(math.pi / 2, w_kin)
                wheel_speed = -w_kin * self._wheelbase
            else:
                steer_angle = math.atan((w_kin * self._wheelbase) / v)
                wheel_speed = v / math.cos(steer_angle)
            
            self._last_steering_angle = steer_angle
        else:
            # Stopped - keep the last steering angle, just zero the wheel speed
            steer_angle = self._last_steering_angle
            wheel_speed = 0.0

        # Convert wheel linear speed to rotational speed (rad/s)
        wheel_rot_speed = wheel_speed / self._wheel_radius

        # Publish commands
        steer_msg = Float64MultiArray()
        steer_msg.data = [steer_angle]
        self._steer_pub.publish(steer_msg)
        
        drive_msg = Float64MultiArray()
        drive_msg.data = [wheel_rot_speed]
        self._drive_pub.publish(drive_msg)

    def _lift_cmd_cb(self, msg: Float64):
        self._lift_target = self._clamp(msg.data, self._lift_min, self._lift_max)

    def _lift_ramp_cb(self):
        max_step = self._lift_speed * self._lift_dt
        error = self._lift_target - self._lift_setpoint
        if abs(error) < 1e-5:
            return

        step = self._clamp(error, -max_step, max_step)
        self._lift_setpoint = self._clamp(
            self._lift_setpoint + step, self._lift_min, self._lift_max)

        out = Float64MultiArray()
        out.data = [self._lift_setpoint]
        self._lift_pub.publish(out)

    def _joint_states_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self._joint_states[name] = pos
        self._current_lift_pos = self._joint_states.get('front_lift_joint', 0.0)

    def _watchdog_cb(self):
        elapsed = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if elapsed > self._timeout:
            # Stop drive wheel, maintain steering
            steer_msg = Float64MultiArray()
            steer_msg.data = [self._last_steering_angle]
            self._steer_pub.publish(steer_msg)
            
            drive_msg = Float64MultiArray()
            drive_msg.data = [0.0]
            self._drive_pub.publish(drive_msg)

    def _publish_status(self):
        status = {
            'lift_target_m':   round(self._lift_target, 4),
            'steering_rad':    round(self._joint_states.get('drive_wheel_Ass_joint', 0.0), 4),
            'drive_wheel_vel': round(self._joint_states.get('drive_wheel_joint', 0.0), 4),
        }
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)

    @staticmethod
    def _clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == '__main__':
    main()
