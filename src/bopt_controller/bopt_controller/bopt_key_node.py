#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from bopt_interfaces.msg import BoptCommandStamped


class BoptKeyNode(Node):

    def __init__(self):
        super().__init__('bopt_key_node')

        # =====================================================
        # SAME VEHICLE PARAMETERS AS ORIGINAL BOPT CONTROLLER
        # =====================================================

        self.declare_parameter('wheelbase', 1.542)
        self.declare_parameter('max_steering_angle', math.pi / 2.0)

        self.wheelbase = self.get_parameter(
            'wheelbase'
        ).value

        self.max_steering_angle = self.get_parameter(
            'max_steering_angle'
        ).value

        # =====================================================
        # INPUT
        # =====================================================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # =====================================================
        # OUTPUT
        # =====================================================

        self.command_pub = self.create_publisher(
            BoptCommandStamped,
            'bopt/key_cmd',
            10
        )

        # Current lift command is kept unchanged until
        # hydraulic controller is connected.
        self.lift_height = 0.0

        self.get_logger().info(
            'BOPT KEY NODE started'
        )

        self.get_logger().info(
            'Input : /cmd_vel'
        )

        self.get_logger().info(
            'Output: /bopt/key_cmd'
        )

    # =========================================================
    # CMD_VEL → BOPT COMMAND
    # =========================================================

    def cmd_vel_callback(self, msg):

        v = msg.linear.x
        yaw_rate = msg.angular.z

        # -----------------------------------------------------
        # Stop command
        # -----------------------------------------------------

        if abs(v) < 1e-6 and abs(yaw_rate) < 1e-6:

            traction_velocity = 0.0
            steering_angle = 0.0

        else:

            # -------------------------------------------------
            # EXACT ORIGINAL BOPT KINEMATICS
            # -------------------------------------------------

            steering_angle = math.atan2(
                -self.wheelbase * yaw_rate,
                v
            )

            traction_velocity = math.copysign(
                math.hypot(
                    v,
                    self.wheelbase * yaw_rate
                ),
                v
            )

            # -------------------------------------------------
            # ORIGINAL REVERSE STEERING NORMALIZATION
            # -------------------------------------------------

            if abs(steering_angle) > math.pi / 2.0:

                steering_angle -= math.copysign(
                    math.pi,
                    steering_angle
                )

            # -------------------------------------------------
            # ORIGINAL STEERING LIMIT
            # -------------------------------------------------

            steering_angle = max(
                -self.max_steering_angle,
                min(
                    self.max_steering_angle,
                    steering_angle
                )
            )

        # =====================================================
        # CREATE BOPT COMMAND
        # =====================================================

        command = BoptCommandStamped()

        command.header.stamp = self.get_clock().now().to_msg()

        command.traction_velocity = traction_velocity
        command.steering_angle = steering_angle
        command.lift_height = self.lift_height

        self.command_pub.publish(command)

        self.get_logger().debug(
            f'KEY | '
            f'cmd_vel v={v:.3f}, '
            f'w={yaw_rate:.3f} → '
            f'velocity={traction_velocity:.3f}, '
            f'steering={steering_angle:.3f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = BoptKeyNode()

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