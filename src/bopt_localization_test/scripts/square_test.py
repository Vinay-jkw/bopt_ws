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

        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.20)
        self.declare_parameter('segment_duration', 4.0)
        self.declare_parameter('settle_time', 1.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.segment_duration = float(self.get_parameter('segment_duration').value)
        self.settle_time = float(self.get_parameter('settle_time').value)

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
        # State variables
        # ====================================================

        self.gt_x = None
        self.gt_y = None
        self.gt_yaw = None

        self.state = 'WAITING'

        self.start_x = None
        self.start_y = None
        self.start_yaw = None

        self.segment_start_time = None
        self.settle_start = None

        self.last_phase = None

        # Waypoint recording for error reporting
        self.seg1_end_pose = None
        self.seg2_end_pose = None

        # ====================================================
        # Timer
        # ====================================================

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info('==========================================')
        self.get_logger().info('BOPT LOCALIZATION REVERSE CURVED TEST')
        self.get_logger().info('==========================================')
        self.get_logger().info(f'Linear speed    : {self.linear_speed:.2f} m/s')
        self.get_logger().info(f'Angular speed   : {self.angular_speed:.2f} rad/s')
        self.get_logger().info(f'Segment Duration: {self.segment_duration:.2f} s')
        self.get_logger().info(f'Settle Time     : {self.settle_time:.2f} s')
        self.get_logger().info('Waiting for Gazebo ground truth...')

    # ========================================================
    # Ground truth callback
    # ========================================================

    def gt_callback(self, msg):

        self.gt_x = msg.pose.position.x
        self.gt_y = msg.pose.position.y
        self.gt_yaw = quaternion_to_yaw(msg.pose.orientation)

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

        self.get_logger().info(f'PHASE -> {phase}')

    # ========================================================
    # Stop robot
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

        self.state = 'REVERSE_LEFT'
        self.segment_start_time = time.monotonic()

        self.publish_phase('START')
        self.publish_phase('REVERSE_LEFT_DRIVE')

        self.get_logger().info('==========================================')
        self.get_logger().info('REVERSE CURVED MANEUVER TEST START')
        self.get_logger().info('Pattern: REVERSE LEFT CURVE -> REVERSE RIGHT CURVE')
        self.get_logger().info(
            f'Start Pose: x={self.start_x:.4f} m, y={self.start_y:.4f} m, yaw={math.degrees(self.start_yaw):.2f}°'
        )
        self.get_logger().info('==========================================')

    # ========================================================
    # Motion Controllers
    # ========================================================

    def drive_segment(self, linear_vel, angular_vel, next_settle_state, end_phase_name):

        elapsed = time.monotonic() - self.segment_start_time

        if elapsed >= self.segment_duration:

            self.stop_robot()
            self.settle_start = time.monotonic()
            self.state = next_settle_state

            self.publish_phase(end_phase_name)

            current_pose = (self.gt_x, self.gt_y, self.gt_yaw)

            if next_settle_state == 'SETTLE_SEG1':
                self.seg1_end_pose = current_pose
            elif next_settle_state == 'SETTLE_SEG2':
                self.seg2_end_pose = current_pose

            self.get_logger().info(f'Segment completed ({end_phase_name}) | Pose=({self.gt_x:.3f}, {self.gt_y:.3f}, {math.degrees(self.gt_yaw):.1f}°)')
            return

        cmd = Twist()
        cmd.linear.x = float(linear_vel)
        cmd.angular.z = float(angular_vel)

        self.cmd_pub.publish(cmd)

    # ========================================================
    # Finish Test & Report Errors
    # ========================================================

    def finish_test(self):

        self.state = 'FINISHED'
        self.publish_phase('FINAL')
        self.stop_robot()

        final_x = self.gt_x
        final_y = self.gt_y
        final_yaw = self.gt_yaw

        dx_final = final_x - self.start_x
        dy_final = final_y - self.start_y

        net_displacement = math.sqrt(dx_final ** 2 + dy_final ** 2)

        longitudinal_dist = (
            dx_final * math.cos(self.start_yaw) +
            dy_final * math.sin(self.start_yaw)
        )

        lateral_offset = (
            -dx_final * math.sin(self.start_yaw) +
            dy_final * math.cos(self.start_yaw)
        )

        heading_change = wrap_to_pi(final_yaw - self.start_yaw)

        self.get_logger().info('==========================================')
        self.get_logger().info('REVERSE CURVED MANEUVER TEST COMPLETE')
        self.get_logger().info('==========================================')
        self.get_logger().info(
            f'Initial Start Pose      : x={self.start_x:.4f} m, y={self.start_y:.4f} m, yaw={math.degrees(self.start_yaw):.2f}°'
        )
        if self.seg1_end_pose:
            self.get_logger().info(
                f'REV Left End Pose       : x={self.seg1_end_pose[0]:.4f} m, y={self.seg1_end_pose[1]:.4f} m, yaw={math.degrees(self.seg1_end_pose[2]):.2f}°'
            )
        self.get_logger().info(
            f'Final Ground Truth Pose : x={final_x:.4f} m, y={final_y:.4f} m, yaw={math.degrees(final_yaw):.2f}°'
        )
        self.get_logger().info('------------------------------------------')
        self.get_logger().info(
            f'Total Net Displacement  : {net_displacement:.4f} m'
        )
        self.get_logger().info(
            f'Longitudinal Distance   : {longitudinal_dist:+.4f} m'
        )
        self.get_logger().info(
            f'Lateral Offset          : {lateral_offset:+.4f} m'
        )
        self.get_logger().info(
            f'Net Heading Change      : {math.degrees(heading_change):+.2f}° ({heading_change:+.4f} rad)'
        )
        self.get_logger().info('==========================================')

    # ========================================================
    # Control Loop State Machine
    # ========================================================

    def control_loop(self):

        if self.gt_x is None or self.gt_y is None or self.gt_yaw is None:
            return

        if self.state == 'WAITING':
            self.start_test()
            return

        # ----------------------------------------------------
        # Segment 1: Reverse Left Curve
        # ----------------------------------------------------
        if self.state == 'REVERSE_LEFT':
            self.drive_segment(
                linear_vel=-self.linear_speed,
                angular_vel=+self.angular_speed,
                next_settle_state='SETTLE_SEG1',
                end_phase_name='REVERSE_LEFT_END'
            )
            return

        if self.state == 'SETTLE_SEG1':
            self.stop_robot()
            if time.monotonic() - self.settle_start >= self.settle_time:
                self.state = 'REVERSE_RIGHT'
                self.segment_start_time = time.monotonic()
                self.publish_phase('REVERSE_RIGHT_DRIVE')
            return

        # ----------------------------------------------------
        # Segment 2: Reverse Right Curve
        # ----------------------------------------------------
        if self.state == 'REVERSE_RIGHT':
            self.drive_segment(
                linear_vel=-self.linear_speed,
                angular_vel=-self.angular_speed,
                next_settle_state='SETTLE_SEG2',
                end_phase_name='REVERSE_RIGHT_END'
            )
            return

        if self.state == 'SETTLE_SEG2':
            self.stop_robot()
            if time.monotonic() - self.settle_start >= self.settle_time:
                self.finish_test()
            return

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
            node.get_logger().info('Localization test interrupted.')
        except Exception:
            print('Localization test interrupted.')
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






