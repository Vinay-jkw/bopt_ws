#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi

    while angle < -math.pi:
        angle += 2.0 * math.pi

    return angle


def angle_error(target, current):
    return wrap_to_pi(target - current)


class SquareTest(Node):

    def __init__(self):

        super().__init__('square_test')

        # ====================================================
        # Parameters
        # ====================================================

        self.declare_parameter('side_length', 1.0)

        self.declare_parameter(
            'linear_speed',
            0.15
        )

        self.declare_parameter(
            'angular_speed',
            0.30
        )

        self.declare_parameter(
            'position_tolerance',
            0.02
        )

        self.declare_parameter(
            'yaw_tolerance_deg',
            2.0
        )

        self.declare_parameter(
            'settle_time',
            1.0
        )

        self.side_length = float(
            self.get_parameter(
                'side_length'
            ).value
        )

        self.linear_speed = float(
            self.get_parameter(
                'linear_speed'
            ).value
        )

        self.angular_speed = float(
            self.get_parameter(
                'angular_speed'
            ).value
        )

        self.position_tolerance = float(
            self.get_parameter(
                'position_tolerance'
            ).value
        )

        self.yaw_tolerance = math.radians(
            float(
                self.get_parameter(
                    'yaw_tolerance_deg'
                ).value
            )
        )

        self.settle_time = float(
            self.get_parameter(
                'settle_time'
            ).value
        )

        # ====================================================
        # Publishers
        # ====================================================

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.phase_pub = self.create_publisher(
            String,
            '/localization_test/phase',
            10
        )

        # ====================================================
        # Ground truth subscriber
        # ====================================================

        self.create_subscription(
            PoseStamped,
            '/ground_truth/pose',
            self.gt_callback,
            20
        )

        # ====================================================
        # Ground truth state
        # ====================================================

        self.gt_x = None
        self.gt_y = None
        self.gt_yaw = None

        # ====================================================
        # Test state
        # ====================================================

        self.state = 'WAITING'

        self.side_index = 0

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.target_x = None
        self.target_y = None
        self.target_yaw = None

        self.settle_start = None

        self.last_phase = None

        # ====================================================
        # Timer
        # ====================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'BOPT LOCALIZATION SQUARE TEST'
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            f'Side length       : {self.side_length:.2f} m'
        )

        self.get_logger().info(
            f'Linear speed      : {self.linear_speed:.2f} m/s'
        )

        self.get_logger().info(
            f'Angular speed     : {self.angular_speed:.2f} rad/s'
        )

        self.get_logger().info(
            f'Position tolerance: '
            f'{self.position_tolerance:.3f} m'
        )

        self.get_logger().info(
            f'Yaw tolerance     : '
            f'{math.degrees(self.yaw_tolerance):.2f} deg'
        )

        self.get_logger().info(
            'Waiting for Gazebo ground truth...'
        )

    # ========================================================
    # Ground truth
    # ========================================================

    def gt_callback(self, msg):

        self.gt_x = msg.pose.position.x
        self.gt_y = msg.pose.position.y

        self.gt_yaw = quaternion_to_yaw(
            msg.pose.orientation
        )

    # ========================================================
    # Publish phase
    # ========================================================

    def publish_phase(self, phase):

        if phase == self.last_phase:
            return

        msg = String()
        msg.data = phase

        self.phase_pub.publish(msg)

        self.last_phase = phase

        self.get_logger().info(
            f'PHASE -> {phase}'
        )

    # ========================================================
    # Publish zero command
    # ========================================================

    def stop_robot(self):

        cmd = Twist()

        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ========================================================
    # Start test
    # ========================================================

    def start_test(self):

        self.start_x = self.gt_x
        self.start_y = self.gt_y
        self.start_yaw = self.gt_yaw

        self.side_index = 0

        self.state = 'DRIVE'

        self.calculate_drive_target()

        self.publish_phase(
            'START'
        )

        self.get_logger().info(
            '=========================================='
        )

        self.get_logger().info(
            'SQUARE TEST START'
        )

        self.get_logger().info(
            f'Start: '
            f'x={self.start_x:.4f}, '
            f'y={self.start_y:.4f}, '
            f'yaw={math.degrees(self.start_yaw):.2f} deg'
        )

        self.get_logger().info(
            '=========================================='
        )

    # ========================================================
    # Calculate target for current 1 m side
    # ========================================================

    def calculate_drive_target(self):

        # Current heading is the direction of travel.
        heading = self.gt_yaw

        self.side_start_x = self.gt_x
        self.side_start_y = self.gt_y
        self.side_start_yaw = heading

        self.target_x = (
            self.gt_x +
            self.side_length *
            math.cos(heading)
        )

        self.target_y = (
            self.gt_y +
            self.side_length *
            math.sin(heading)
        )

        self.target_yaw = heading
        self.state = 'DRIVE'

        self.publish_phase(
            f'SIDE_{self.side_index + 1}_DRIVE'
        )

        self.get_logger().info(
            f'Side {self.side_index + 1}: '
            f'start=({self.side_start_x:.3f}, {self.side_start_y:.3f}) -> '
            f'target=({self.target_x:.3f}, {self.target_y:.3f})'
        )

    # ========================================================
    # Calculate next 90 degree target
    # ========================================================

    def calculate_rotation_target(self):

        self.target_yaw = wrap_to_pi(
            self.gt_yaw +
            math.pi / 2.0
        )

        self.state = 'ROTATE'

        self.publish_phase(
            f'TURN_{self.side_index + 1}'
        )

        self.get_logger().info(
            f'Turn {self.side_index + 1} target: '
            f'{math.degrees(self.target_yaw):.2f} deg'
        )

    # ========================================================
    # Drive controller with smooth deceleration & progress tracking
    # ========================================================

    def drive_control(self):

        # Progress along the intended side vector
        dx = self.gt_x - self.side_start_x
        dy = self.gt_y - self.side_start_y

        dist_traveled = (
            dx * math.cos(self.side_start_yaw) +
            dy * math.sin(self.side_start_yaw)
        )

        remaining = self.side_length - dist_traveled

        if remaining <= self.position_tolerance or dist_traveled >= self.side_length:

            self.stop_robot()

            self.settle_start = time.monotonic()

            self.state = 'SETTLE_DRIVE'

            self.publish_phase(
                f'SIDE_{self.side_index + 1}_END'
            )

            self.get_logger().info(
                f'Side {self.side_index + 1} complete | '
                f'traveled={dist_traveled:.4f} m (error={abs(remaining):.4f} m)'
            )

            return

        # Smooth deceleration ramp in the last 25 cm
        slowdown_dist = 0.25
        min_speed = 0.03

        if remaining < slowdown_dist:
            speed = min_speed + (self.linear_speed - min_speed) * max(0.0, remaining / slowdown_dist)
        else:
            speed = self.linear_speed

        cmd = Twist()
        cmd.linear.x = float(speed)
        cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

    # ========================================================
    # Rotation controller with smooth angular deceleration
    # ========================================================

    def rotate_control(self):

        error = angle_error(
            self.target_yaw,
            self.gt_yaw
        )

        if abs(error) <= self.yaw_tolerance:

            self.stop_robot()

            self.settle_start = time.monotonic()

            self.state = 'SETTLE_ROTATE'

            self.publish_phase(
                f'TURN_{self.side_index + 1}_END'
            )

            self.get_logger().info(
                f'Turn {self.side_index + 1} complete | '
                f'error={math.degrees(error):.3f} deg'
            )

            return

        # Smooth angular deceleration in the last 25 degrees
        slowdown_angle = math.radians(25.0)
        min_angular = 0.04

        if abs(error) < slowdown_angle:
            turn_rate = min_angular + (self.angular_speed - min_angular) * (abs(error) / slowdown_angle)
        else:
            turn_rate = self.angular_speed

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = float(math.copysign(turn_rate, error))

        self.cmd_pub.publish(cmd)

    # ========================================================
    # Main control loop
    # ========================================================

    def control_loop(self):

        # ----------------------------------------------------
        # Wait for GT
        # ----------------------------------------------------

        if (
            self.gt_x is None or
            self.gt_y is None or
            self.gt_yaw is None
        ):

            return

        # ----------------------------------------------------
        # Waiting
        # ----------------------------------------------------

        if self.state == 'WAITING':

            self.start_test()

            return

        # ----------------------------------------------------
        # Drive
        # ----------------------------------------------------

        if self.state == 'DRIVE':

            self.drive_control()

            return

        # ----------------------------------------------------
        # Settle after drive
        # ----------------------------------------------------

        if self.state == 'SETTLE_DRIVE':

            self.stop_robot()

            if (
                time.monotonic() -
                self.settle_start
                >= self.settle_time
            ):

                self.calculate_rotation_target()

            return

        # ----------------------------------------------------
        # Rotate
        # ----------------------------------------------------

        if self.state == 'ROTATE':

            self.rotate_control()

            return

        # ----------------------------------------------------
        # Settle after rotation
        # ----------------------------------------------------

        if self.state == 'SETTLE_ROTATE':

            self.stop_robot()

            if (
                time.monotonic() -
                self.settle_start
                >= self.settle_time
            ):

                self.side_index += 1

                # After fourth rotation, test is complete.
                if self.side_index >= 4:

                    self.state = 'FINISHED'

                    self.publish_phase(
                        'FINAL'
                    )

                    self.stop_robot()

                    self.get_logger().info(
                        '=========================================='
                    )

                    self.get_logger().info(
                        'SQUARE TEST COMPLETE'
                    )

                    self.get_logger().info(
                        'Robot completed 4 sides + 4 turns.'
                    )

                    self.get_logger().info(
                        '=========================================='
                    )

                else:

                    self.calculate_drive_target()

            return

        # ----------------------------------------------------
        # Finished
        # ----------------------------------------------------

        if self.state == 'FINISHED':

            self.stop_robot()

            return


# ============================================================
# Main
# ============================================================

def main(args=None):

    rclpy.init(args=args)

    node = SquareTest()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        try:
            node.get_logger().info(
                'Square test interrupted.'
            )
        except Exception:
            print('Square test interrupted.')

    finally:

        try:
            node.stop_robot()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()

