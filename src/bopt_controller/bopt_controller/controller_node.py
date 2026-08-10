#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class BOPTController(Node):

    def __init__(self):
        super().__init__('bopt_controller')

        # --- Robot Parameters ---
        self.wheel_radius = 0.115
        self.max_wheel_velocity = 3.0
        self.max_steering_angle = 0.6

        # --- Mimic Parameters ---
        self.lift_min = 0.0
        self.lift_max = 0.095
        self.mimic_min_angle = 0.0
        self.mimic_max_angle = 0.78

        self.mimic_slew_rate = 0.35
        self.control_dt = 0.05

        self.commanded_mimic_angle = None
        self.joint_states_received = False

        # --- Safety & State ---
        self.cmd_vel_timeout = 0.5
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_lift_position = 0.0
        self.current_shaft_angle = 0.0

        # --- Subscribers ---
        self.create_subscription(
            AckermannDriveStamped,
            '/cmd_ackermann',
            self.cmd_ackermann_callback,
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

        self.mimic_pub = self.create_publisher(
            Float64MultiArray,
            '/mimic_joint_controller/commands',
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

    def cmd_ackermann_callback(self, msg):

        self.last_cmd_vel_time = self.get_clock().now()

        speed = msg.drive.speed
        steering_angle = msg.drive.steering_angle

        wheel_velocity = -(speed / self.wheel_radius)

        wheel_velocity = self.clamp(
            wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        self.publish_traction(wheel_velocity)
        self.publish_steering(steering_angle)

        self.get_logger().info(
            f'DRIVE | speed={speed:.3f} m/s | '
            f'wheel={wheel_velocity:.3f} rad/s | '
            f'steering={steering_angle:.3f} rad'
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

        if 'front_wheel_shaft_left_joint' in msg.name:
            index = msg.name.index(
                'front_wheel_shaft_left_joint'
            )

            if index < len(msg.position):
                self.current_shaft_angle = msg.position[index]

        if not self.joint_states_received:
            self.commanded_mimic_angle = self.current_shaft_angle
            self.joint_states_received = True

            self.get_logger().info(
                f'MIMIC | Seeded from actual shaft position: '
                f'{self.commanded_mimic_angle:.4f} rad'
            )

    def control_loop(self):

        elapsed = (
            self.get_clock().now() -
            self.last_cmd_vel_time
        ).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout:
            self.publish_traction(0.0)
            self.publish_steering(0.0)

        if not self.joint_states_received:
            return

        target_mimic_angle = self.calculate_mimic_angle(
            self.current_lift_position
        )

        max_step = (
            self.mimic_slew_rate *
            self.control_dt
        )

        error = (
            target_mimic_angle -
            self.commanded_mimic_angle
        )

        if abs(error) <= max_step:
            self.commanded_mimic_angle = target_mimic_angle
        else:
            self.commanded_mimic_angle += math.copysign(
                max_step,
                error
            )

        self.commanded_mimic_angle = self.clamp(
            self.commanded_mimic_angle,
            self.mimic_min_angle,
            self.mimic_max_angle
        )

        self.publish_mimic(
            self.commanded_mimic_angle
        )

    def calculate_mimic_angle(self, lift_position):

        lift_position = self.clamp(
            lift_position,
            self.lift_min,
            self.lift_max
        )

        lift_ratio = (
            (lift_position - self.lift_min) /
            (self.lift_max - self.lift_min)
        )

        angle = (
            self.mimic_min_angle +
            lift_ratio *
            (self.mimic_max_angle - self.mimic_min_angle)
        )

        return self.clamp(
            angle,
            self.mimic_min_angle,
            self.mimic_max_angle
        )

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

    def publish_mimic(self, angle):

        self.mimic_pub.publish(
            Float64MultiArray(
                data=[angle, angle]
            )
        )

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
        node.publish_traction(0.0)
        node.publish_steering(0.0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()