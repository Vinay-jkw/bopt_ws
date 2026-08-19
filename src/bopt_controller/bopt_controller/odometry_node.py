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
        # BOPT ROBOT PARAMETERS
        # =====================================================

        self.drive_wheel_radius = self.declare_parameter(
            'drive_wheel_radius',
            0.115
        ).value

        self.front_wheel_radius = self.declare_parameter(
            'front_wheel_radius',
            0.0425
        ).value

        self.wheelbase = self.declare_parameter(
            'wheelbase',
            1.542
        ).value

        self.front_wheel_center_x = self.declare_parameter(
            'front_wheel_center_x',
            1.539
        ).value

        self.front_wheel_center_y = self.declare_parameter(
            'front_wheel_center_y',
            -0.0035
        ).value

        # =====================================================
        # JOINT NAMES
        # =====================================================

        self.traction_joint = self.declare_parameter(
            'traction_joint',
            'drive_wheel_joint'
        ).value

        self.steering_joint = self.declare_parameter(
            'steering_joint',
            'drive_wheel_Ass_joint'
        ).value

        # =====================================================
        # ODOMETRY STATE
        # =====================================================

        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_yaw = 0.0

        self.actual_drive_position = 0.0
        self.actual_drive_velocity = 0.0

        self.actual_steering_position = 0.0
        self.actual_steering_velocity = 0.0

        self.previous_drive_position = 0.0

        self.odometry_initialized = True

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

        self.control_period = self.declare_parameter(
            'control_period',
            0.02
        ).value

        timer_period = self.control_period

        self.control_timer = self.create_timer(
            timer_period,
            self.update_odometry
        )

        # =====================================================
        # START TIME
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
            f'Front wheel center X: '
            f'{self.front_wheel_center_x:.4f} m'
        )

        self.get_logger().info(
            f'Front wheel center Y: '
            f'{self.front_wheel_center_y:.4f} m'
        )

        self.get_logger().info(
            f'Traction joint: '
            f'{self.traction_joint}'
        )

        self.get_logger().info(
            f'Steering joint: '
            f'{self.steering_joint}'
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

                    self.actual_drive_position = (
                        msg.position[i]
                    )

                if i < len(msg.velocity):

                    self.actual_drive_velocity = (
                        msg.velocity[i]
                    )

            # -------------------------------------------------
            # STEERING
            # -------------------------------------------------

            elif name == self.steering_joint:

                if i < len(msg.position):

                    self.actual_steering_position = (
                        msg.position[i]
                    )

                if i < len(msg.velocity):

                    self.actual_steering_velocity = (
                        msg.velocity[i]
                    )

    # =========================================================
    # UPDATE ODOMETRY
    # =========================================================

    def update_odometry(self):

        current_time = self.get_clock().now()

        dt = (
            current_time -
            self.last_odometry_time
        ).nanoseconds * 1e-9

        if dt <= 0.0:

            return

        self.last_odometry_time = current_time

        # -----------------------------------------------------
        # INITIALIZATION
        # -----------------------------------------------------

        if not self.odometry_initialized:

            self.previous_drive_position = (
                self.actual_drive_position
            )

            self.odometry_initialized = True

            return

        # -----------------------------------------------------
        # DRIVE WHEEL ROTATION
        # -----------------------------------------------------

        delta_drive = (
            self.actual_drive_position -
            self.previous_drive_position
        )

        self.previous_drive_position = (
            self.actual_drive_position
        )

        # -----------------------------------------------------
        # WHEEL ROTATION -> DISTANCE
        # -----------------------------------------------------

        drive_distance = (
            self.drive_wheel_radius *
            delta_drive
        )

        # -----------------------------------------------------
        # CURRENT STEERING ANGLE
        # -----------------------------------------------------

        steering_angle = (
            self.actual_steering_position
        )

        # -----------------------------------------------------
        # DISTANCE OF ODOMETRY REFERENCE POINT
        # -----------------------------------------------------

        center_distance = (
            drive_distance *
            math.cos(steering_angle)
        )

        # -----------------------------------------------------
        # CHANGE IN YAW
        # -----------------------------------------------------

        delta_yaw = (
            -drive_distance *
            math.sin(steering_angle)
            / self.wheelbase
        )

        # -----------------------------------------------------
        # FRONT WHEEL CENTER POSITION
        # -----------------------------------------------------

        center_x = (
            self.odom_x
            + math.cos(self.odom_yaw)
            * self.front_wheel_center_x
            - math.sin(self.odom_yaw)
            * self.front_wheel_center_y
        )

        center_y = (
            self.odom_y
            + math.sin(self.odom_yaw)
            * self.front_wheel_center_x
            + math.cos(self.odom_yaw)
            * self.front_wheel_center_y
        )

        # -----------------------------------------------------
        # MIDPOINT YAW
        # -----------------------------------------------------

        yaw_mid = (
            self.odom_yaw
            + 0.5 * delta_yaw
        )

        # -----------------------------------------------------
        # NEW FRONT WHEEL CENTER POSITION
        # -----------------------------------------------------

        new_center_x = (
            center_x
            + center_distance
            * math.cos(yaw_mid)
        )

        new_center_y = (
            center_y
            + center_distance
            * math.sin(yaw_mid)
        )

        # -----------------------------------------------------
        # NEW YAW
        # -----------------------------------------------------

        new_yaw = (
            self.odom_yaw +
            delta_yaw
        )

        # -----------------------------------------------------
        # TRANSFORM BACK TO BASE_FOOTPRINT
        # -----------------------------------------------------

        self.odom_x = (
            new_center_x
            - math.cos(new_yaw)
            * self.front_wheel_center_x
            + math.sin(new_yaw)
            * self.front_wheel_center_y
        )

        self.odom_y = (
            new_center_y
            - math.sin(new_yaw)
            * self.front_wheel_center_x
            - math.cos(new_yaw)
            * self.front_wheel_center_y
        )

        self.odom_yaw = new_yaw

        # -----------------------------------------------------
        # NORMALIZE YAW
        # -----------------------------------------------------

        while self.odom_yaw > math.pi:

            self.odom_yaw -= 2.0 * math.pi

        while self.odom_yaw < -math.pi:

            self.odom_yaw += 2.0 * math.pi

        # -----------------------------------------------------
        # PUBLISH
        # -----------------------------------------------------

        self.publish_odometry()

    # =========================================================
    # PUBLISH ODOMETRY
    # =========================================================

    def publish_odometry(self):

        # -----------------------------------------------------
        # QUATERNION FROM YAW
        # -----------------------------------------------------

        qz = math.sin(
            self.odom_yaw / 2.0
        )

        qw = math.cos(
            self.odom_yaw / 2.0
        )

        # -----------------------------------------------------
        # ODOMETRY MESSAGE
        # -----------------------------------------------------

        odom = Odometry()

        odom.header.stamp = (
            self.get_clock().now().to_msg()
        )

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

        # -----------------------------------------------------
        # VELOCITY
        # -----------------------------------------------------

        drive_velocity = (
            self.drive_wheel_radius *
            self.actual_drive_velocity
        )

        steering_angle = (
            self.actual_steering_position
        )

        center_velocity = (
            drive_velocity *
            math.cos(steering_angle)
        )

        angular_velocity = (
            -drive_velocity *
            math.sin(steering_angle)
            / self.wheelbase
        )

        odom.twist.twist.linear.x = (
            center_velocity
            + angular_velocity
            * self.front_wheel_center_y
        )

        odom.twist.twist.linear.y = (
            -angular_velocity
            * self.front_wheel_center_x
        )

        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0

        odom.twist.twist.angular.y = 0.0

        odom.twist.twist.angular.z = (
            angular_velocity
        )

        self.odom_publisher.publish(odom)

        # -----------------------------------------------------
        # TF
        # -----------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = (
            self.get_clock().now().to_msg()
        )

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