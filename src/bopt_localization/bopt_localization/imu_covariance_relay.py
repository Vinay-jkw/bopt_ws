#!/usr/bin/env python3
"""
Republishes /imu with non-zero covariance matrices, since the
Gazebo IMU sensor plugin is publishing all-zero covariance despite
the configured noise model. robot_localization's EKF treats zero
covariance as invalid/unsupported or over-trusts raw noise.

Values below match the <noise> stddev set in gazebo.xacro:
  angular_velocity stddev:     2e-4  rad/s  -> variance = 4e-8
  linear_acceleration stddev:  1.7e-2 m/s^2 -> variance = 2.89e-4
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

ANGULAR_VELOCITY_VAR = 4e-8
LINEAR_ACCEL_VAR = 2.89e-4
ORIENTATION_VAR = 1e-3


def diag_covariance(variance):
    return [
        variance, 0.0, 0.0,
        0.0, variance, 0.0,
        0.0, 0.0, variance,
    ]


class ImuCovarianceRelay(Node):

    def __init__(self):
        super().__init__('imu_covariance_relay')

        self.declare_parameter('input_topic', '/imu')
        self.declare_parameter('output_topic', '/imu/corrected')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.publisher = self.create_publisher(Imu, output_topic, 10)

        self.subscription = self.create_subscription(
            Imu,
            input_topic,
            self.imu_callback,
            10
        )

        self.get_logger().info(
            f'Relaying {input_topic} -> {output_topic} with non-zero covariance'
        )

    def imu_callback(self, msg):
        msg.orientation_covariance = diag_covariance(ORIENTATION_VAR)
        msg.angular_velocity_covariance = diag_covariance(ANGULAR_VELOCITY_VAR)
        msg.linear_acceleration_covariance = diag_covariance(LINEAR_ACCEL_VAR)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImuCovarianceRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
