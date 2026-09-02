#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64

from bopt_interfaces.msg import BoptCommand
from bopt_interfaces.msg import BoptCommandStamped


class BoptTwistRelay(Node):

    def __init__(self):
        super().__init__('bopt_twist_relay')

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.declare_parameter('control_mode', 'manual')

        self.control_mode = self.get_parameter(
            'control_mode'
        ).value

        # =====================================================
        # SUBSCRIBERS
        # =====================================================

        # -----------------------------------------------------
        # BOPT KEY
        #
        # Already converted:
        # cmd_vel -> traction velocity + steering angle
        # -----------------------------------------------------

        self.key_sub = self.create_subscription(
            BoptCommandStamped,
            '/bopt/key_cmd',
            self.key_callback,
            10
        )

        # -----------------------------------------------------
        # NMPC
        #
        # NMPC already provides:
        # traction velocity + steering angle
        # -----------------------------------------------------

        self.nmpc_sub = self.create_subscription(
            BoptCommand,
            '/bopt/nmpc_cmd',
            self.nmpc_callback,
            10
        )

        # -----------------------------------------------------
        # HYDRAULIC
        # -----------------------------------------------------

        self.lift_sub = self.create_subscription(
            BoptCommand,
            '/bopt/hydraulic_cmd',
            self.lift_callback,
            10
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.relay_pub = self.create_publisher(
            BoptCommandStamped,
            '/bopt/relay_cmd',
            10
        )

        # =====================================================
        # COMMAND STATE
        # =====================================================

        self.key_speed = 0.0
        self.key_steering = 0.0

        self.nmpc_speed = 0.0
        self.nmpc_steering = 0.0
        self.nmpc_lift = 0.0

        self.lift_height = 0.0

        # =====================================================
        # RELAY LOOP
        # =====================================================

        self.timer = self.create_timer(
            0.02,       # 50 Hz
            self.relay_callback
        )

        self.get_logger().info(
            'BOPT TWIST RELAY started'
        )

        self.get_logger().info(
            f'Control mode: {self.control_mode}'
        )

    # =========================================================
    # KEY CALLBACK
    # =========================================================

    def key_callback(self, msg):

        self.key_speed = msg.traction_velocity
        self.key_steering = msg.steering_angle

    # =========================================================
    # NMPC CALLBACK
    # =========================================================

    def nmpc_callback(self, msg):

        self.nmpc_speed = msg.traction_velocity
        self.nmpc_steering = msg.steering_angle
        self.nmpc_lift = msg.lift_height

    # =========================================================
    # HYDRAULIC CALLBACK
    # =========================================================

    def lift_callback(self, msg):

        self.lift_height = msg.lift_height

    # =========================================================
    # RELAY
    # =========================================================

    def relay_callback(self):

        command = BoptCommandStamped()

        # =====================================================
        # MANUAL MODE
        # =====================================================

        if self.control_mode == 'manual':

            command.traction_velocity = (
                self.key_speed
            )

            command.steering_angle = (
                self.key_steering
            )

            # Hydraulic remains independent
            command.lift_height = (
                self.lift_height
            )

        # =====================================================
        # NMPC MODE
        # =====================================================

        elif self.control_mode == 'nmpc':

            command.traction_velocity = (
                self.nmpc_speed
            )

            command.steering_angle = (
                self.nmpc_steering
            )

            command.lift_height = (
                self.nmpc_lift
            )

        # =====================================================
        # INVALID MODE
        # =====================================================

        else:

            self.get_logger().error(
                f'Unknown control mode: {self.control_mode}'
            )

            command.traction_velocity = 0.0
            command.steering_angle = 0.0
            command.lift_height = self.lift_height

        # =====================================================
        # HEADER
        # =====================================================

        command.header.stamp = (
            self.get_clock().now().to_msg()
        )

        command.header.frame_id = 'base_link'

        # =====================================================
        # PUBLISH
        # =====================================================

        self.relay_pub.publish(command)


def main(args=None):

    rclpy.init(args=args)

    node = BoptTwistRelay()

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