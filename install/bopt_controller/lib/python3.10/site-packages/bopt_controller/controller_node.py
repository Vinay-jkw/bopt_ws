#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class BOPTController(Node):

    def __init__(self):
        super().__init__('bopt_controller')

        # --- Robot Parameters ---
        self.wheel_radius = 0.115
        self.wheelbase = 1.542
        self.max_wheel_velocity = 5.0
        self.max_steering_angle = 1.5708 
        self.control_dt = 0.05

        # --- Lift Parameters ---
        self.lift_min = 0.0
        self.lift_max = 0.095

        # --- Safety & State ---
        self.cmd_vel_timeout = 0.5
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_lift_position = 0.0

        # --- Subscribers ---
        self.create_subscription(
            AckermannDriveStamped,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/lift_cmd',
            self.lift_cmd_callback,
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # --- Publishers ---
        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            '/traction_joint_controller/commands',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_joint_controller/commands',
            10
        )

        self.lift_pub = self.create_publisher(
            JointTrajectory,
            '/lift_joint_controller/joint_trajectory',
            10
        )

        self.control_timer = self.create_timer(
            self.control_dt,
            self.control_loop
        )

        self.publish_traction(0.0)
        self.publish_steering(0.0)

        self.get_logger().info(
            'BOPT Controller started - ROBOT STOPPED'
        )

    def cmd_vel_callback(self, msg):

        self.last_cmd_vel_time = self.get_clock().now()

        # Commanded velocity at the front load wheel axis
        v = msg.drive.speed
        omega = msg.drive.steering_angle

        # Kinematics at the rear drive wheel
        v_x = v
        v_y = -omega * self.wheelbase

        if v_x == 0.0 and v_y == 0.0:
            target_steering = 0.0
            v_drive_linear = 0.0
        else:
            target_steering = math.atan2(v_y, v_x)
            v_drive_linear = math.hypot(v_x, v_y)

        # Prevent 180-degree steering flips when reversing
        if target_steering > math.pi / 2:
            target_steering -= math.pi
            v_drive_linear = -v_drive_linear
        elif target_steering < -math.pi / 2:
            target_steering += math.pi
            v_drive_linear = -v_drive_linear

        # Convert linear drive speed to wheel rotational velocity (rad/s)
        wheel_velocity = v_drive_linear / self.wheel_radius
        
        # If your robot drives backward when commanded forward, uncomment the next line:
        # wheel_velocity = -wheel_velocity

        wheel_velocity = self.clamp(
            wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        target_steering = self.clamp(
            target_steering,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        self.publish_traction(wheel_velocity)
        self.publish_steering(target_steering)

        self.get_logger().info(
            f'DRIVE | v_front={v:.2f} m/s | omega={omega:.2f} rad/s | '
            f'wheel_vel={wheel_velocity:.2f} rad/s | steer={target_steering:.2f} rad'
        )

    def lift_cmd_callback(self, msg):

        lift_position = self.clamp(
            msg.data,
            self.lift_min,
            self.lift_max
        )

        distance = abs(
            lift_position - self.current_lift_position
        )

        duration_s = max(
            1.0,
            distance / 0.025 + 0.5
        )

        self.publish_lift(
            lift_position,
            duration_s
        )

        self.get_logger().info(
            f'LIFT | target_pos={lift_position:.4f} m | '
            f'duration={duration_s:.2f} s'
        )

    def joint_state_callback(self, msg):

        if 'front_lift_joint' in msg.name:
            index = msg.name.index('front_lift_joint')

            if index < len(msg.position):
                self.current_lift_position = msg.position[index]

    def control_loop(self):
        # Only checks for cmd_vel timeout now since mimic logic is gone
        elapsed = (
            self.get_clock().now() -
            self.last_cmd_vel_time
        ).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout:
            self.publish_traction(0.0)
            self.publish_steering(0.0)

    def publish_traction(self, velocity):

        self.traction_pub.publish(
            Float64MultiArray(
                data=[velocity]
            )
        )

    def publish_steering(self, angle):

        self.steering_pub.publish(
            Float64MultiArray(
                data=[angle]
            )
        )

    def publish_lift(self, position, duration_s=2.0):

        msg = JointTrajectory()
        msg.joint_names = [
            'front_lift_joint'
        ]

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]

        point.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int(
                (duration_s % 1.0) * 1e9
            )
        )

        msg.points = [point]

        self.lift_pub.publish(msg)

    @staticmethod
    def clamp(value, minimum, maximum):

        return max(
            minimum,
            min(maximum, value)
        )


def main(args=None):

    rclpy.init(args=args)

    node = BOPTController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        try:
            node.publish_traction(0.0)
            node.publish_steering(0.0)
        except Exception:
            pass

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()