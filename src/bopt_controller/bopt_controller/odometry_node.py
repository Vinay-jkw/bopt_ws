import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster


class BoptOdometry(Node):

    def __init__(self):

        super().__init__('bopt_odometry')

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.drive_wheel_radius = self.declare_parameter(
            'drive_wheel_radius',
            0.115
        ).value

        # Distance from base_footprint
        # (front load-wheel axle midpoint)
        # to the steerable drive wheel
        self.wheelbase = self.declare_parameter(
            'wheelbase',
            1.542
        ).value

        self.traction_joint = self.declare_parameter(
            'traction_joint',
            'drive_wheel_joint'
        ).value

        self.steering_joint = self.declare_parameter(
            'steering_joint',
            'drive_wheel_Ass_joint'
        ).value

        self.control_period = self.declare_parameter(
            'control_period',
            0.02
        ).value

        # =====================================================
        # ODOMETRY STATE
        # =====================================================

        # base_footprint pose in odom frame
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        # Actual joint feedback
        self.actual_drive_position = 0.0
        self.actual_drive_velocity = 0.0

        self.actual_steering_position = 0.0
        self.actual_steering_velocity = 0.0

        # Previous drive wheel position
        self.previous_drive_position = 0.0

        # We need one valid JointState before starting
        self.received_joint_state = False
        self.odometry_initialized = False

        # =====================================================
        # JOINT STATE SUBSCRIBER
        # =====================================================

        self.joint_state_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            50
        )

        # =====================================================
        # ODOMETRY PUBLISHER
        # =====================================================

        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # =====================================================
        # TF BROADCASTER
        # =====================================================

        self.tf_broadcaster = TransformBroadcaster(self)

        # =====================================================
        # TIMER
        # =====================================================

        self.control_timer = self.create_timer(
            self.control_period,
            self.update_odometry
        )

        # =====================================================
        # TIME
        # =====================================================

        self.last_odometry_time = self.get_clock().now()

        # =====================================================
        # LOGGING
        # =====================================================

        self.get_logger().info(
            'BOPT Odometry node started'
        )

        self.get_logger().info(
            f'Drive wheel radius: '
            f'{self.drive_wheel_radius:.4f} m'
        )

        self.get_logger().info(
            f'Wheelbase: '
            f'{self.wheelbase:.4f} m'
        )

        self.get_logger().info(
            f'Traction joint: '
            f'{self.traction_joint}'
        )

        self.get_logger().info(
            f'Steering joint: '
            f'{self.steering_joint}'
        )

        self.get_logger().info(
            'Odometry reference: base_footprint '
            '(front load-wheel axle midpoint)'
        )

    # =========================================================
    # JOINT STATE CALLBACK
    # =========================================================

    def joint_state_callback(self, msg):

        for i, name in enumerate(msg.name):

            # -------------------------------------------------
            # DRIVE WHEEL
            # -------------------------------------------------

            if name == self.traction_joint:

                if i < len(msg.position):
                    self.actual_drive_position = msg.position[i]

                if i < len(msg.velocity):
                    self.actual_drive_velocity = msg.velocity[i]

            # -------------------------------------------------
            # STEERING
            # -------------------------------------------------

            elif name == self.steering_joint:

                if i < len(msg.position):
                    self.actual_steering_position = msg.position[i]

                if i < len(msg.velocity):
                    self.actual_steering_velocity = msg.velocity[i]

        self.received_joint_state = True

    # =========================================================
    # UPDATE ODOMETRY
    # =========================================================

    def update_odometry(self):

        # Don't calculate anything before joint feedback exists
        if not self.received_joint_state:
            return

        current_time = self.get_clock().now()

        dt = (
            current_time -
            self.last_odometry_time
        ).nanoseconds * 1e-9

        if dt <= 0.0:
            return

        self.last_odometry_time = current_time

        # =====================================================
        # INITIALIZATION
        # =====================================================

        if not self.odometry_initialized:

            self.previous_drive_position = (
                self.actual_drive_position
            )

            self.odometry_initialized = True

            self.publish_odometry(
                linear_velocity=0.0,
                angular_velocity=0.0,
                stamp=current_time
            )

            return

        # =====================================================
        # DRIVE WHEEL ROTATION
        # =====================================================

        delta_drive = (
            self.actual_drive_position -
            self.previous_drive_position
        )

        self.previous_drive_position = (
            self.actual_drive_position
        )

        # =====================================================
        # WHEEL ROTATION -> DISTANCE
        # =====================================================

        drive_distance = (
            self.drive_wheel_radius *
            delta_drive
        )

        # =====================================================
        # ACTUAL STEERING ANGLE
        # =====================================================

        steering_angle = (
            self.actual_steering_position
        )

        # =====================================================
        # BOPT KINEMATICS
        #
        # base_footprint is the front load-wheel axle
        # midpoint.
        #
        # The drive wheel is wheelbase meters behind it.
        #
        # Drive-wheel velocity:
        #
        #     Vw_x = V
        #     Vw_y = -omega * wheelbase
        #
        # Therefore:
        #
        #     V = Vw * cos(delta)
        #
        #     omega =
        #       -Vw * sin(delta) / wheelbase
        #
        # =====================================================

        drive_velocity = (
            drive_distance / dt
        )

        linear_velocity = (
            drive_velocity *
            math.cos(steering_angle)
        )

        angular_velocity = (
            -drive_velocity *
            math.sin(steering_angle)
            / self.wheelbase
        )

        # =====================================================
        # INTEGRATE BASE_FOOTPRINT POSE
        # =====================================================

        delta_yaw = (
            angular_velocity * dt
        )

        # Midpoint integration gives better accuracy during
        # turning than using the old yaw for the entire step.

        yaw_mid = (
            self.odom_yaw +
            0.5 * delta_yaw
        )

        distance_base = (
            linear_velocity * dt
        )

        self.odom_x += (
            distance_base *
            math.cos(yaw_mid)
        )

        self.odom_y += (
            distance_base *
            math.sin(yaw_mid)
        )

        self.odom_yaw += delta_yaw

        # =====================================================
        # NORMALIZE YAW
        # =====================================================

        self.odom_yaw = math.atan2(
            math.sin(self.odom_yaw),
            math.cos(self.odom_yaw)
        )

        # =====================================================
        # PUBLISH
        # =====================================================

        self.publish_odometry(
            linear_velocity=linear_velocity,
            angular_velocity=angular_velocity,
            stamp=current_time
        )

    # =========================================================
    # PUBLISH ODOMETRY
    # =========================================================

    def publish_odometry(
        self,
        linear_velocity,
        angular_velocity,
        stamp
    ):

        # =====================================================
        # QUATERNION
        # =====================================================

        qz = math.sin(
            self.odom_yaw / 2.0
        )

        qw = math.cos(
            self.odom_yaw / 2.0
        )

        # =====================================================
        # ODOMETRY MESSAGE
        # =====================================================

        odom = Odometry()

        odom.header.stamp = stamp.to_msg()

        odom.header.frame_id = 'odom'

        odom.child_frame_id = 'base_footprint'

        # Position

        odom.pose.pose.position.x = (
            self.odom_x
        )

        odom.pose.pose.position.y = (
            self.odom_y
        )

        odom.pose.pose.position.z = 0.0

        # Orientation

        odom.pose.pose.orientation.x = 0.0

        odom.pose.pose.orientation.y = 0.0

        odom.pose.pose.orientation.z = qz

        odom.pose.pose.orientation.w = qw

        # Velocity of base_footprint

        odom.twist.twist.linear.x = (
            linear_velocity
        )

        odom.twist.twist.linear.y = 0.0

        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0

        odom.twist.twist.angular.y = 0.0

        odom.twist.twist.angular.z = (
            angular_velocity
        )

        self.odom_publisher.publish(odom)

        # =====================================================
        # TF: odom -> base_footprint
        # =====================================================

        transform = TransformStamped()

        transform.header.stamp = stamp.to_msg()

        transform.header.frame_id = 'odom'

        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = (
            self.odom_x
        )

        transform.transform.translation.y = (
            self.odom_y
        )

        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0

        transform.transform.rotation.y = 0.0

        transform.transform.rotation.z = qz

        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(
            transform
        )


def main(args=None):

    rclpy.init(args=args)

    node = BoptOdometry()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()