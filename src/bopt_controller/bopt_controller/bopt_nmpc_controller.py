#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String

from bopt_interfaces.msg import BoptCommand


class BoptNmpcController(Node):

    def __init__(self):
        super().__init__('bopt_nmpc_controller')

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.declare_parameter(
            'max_steering_angle',
            math.pi / 2.0
        )

        self.declare_parameter(
            'require_state_check',
            False
        )

        self.declare_parameter(
            'invert_steering',
            True
        )

        self.max_steering_angle = self.get_parameter(
            'max_steering_angle'
        ).value

        self.require_state_check = self.get_parameter(
            'require_state_check'
        ).value

        self.invert_steering = self.get_parameter(
            'invert_steering'
        ).value

        # =====================================================
        # STATE
        # =====================================================

        self.velocity = 0.0
        self.steering_angle = 0.0
        self.current_state = 'Custom'

        # =====================================================
        # SUBSCRIBERS
        # =====================================================

        self.velocity_sub = self.create_subscription(
            Float64,
            'velocity',
            self.velocity_callback,
            10
        )

        self.steering_sub = self.create_subscription(
            Float64,
            'steering_angle',
            self.steering_callback,
            10
        )

        self.state_sub = self.create_subscription(
            String,
            'state',
            self.state_callback,
            10
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.command_pub = self.create_publisher(
            BoptCommand,
            'bopt/nmpc_cmd',
            10
        )

        # =====================================================
        # PUBLISH LOOP
        # =====================================================

        self.timer = self.create_timer(
            0.02,       # 50 Hz
            self.publish_command
        )

        self.get_logger().info(
            'NMPC COMMAND BRIDGE started'
        )

        self.get_logger().info(
            'Input : /velocity [m/s]'
        )

        self.get_logger().info(
            'Input : /steering_angle [deg]'
        )

        self.get_logger().info(
            'Input : /state'
        )

        self.get_logger().info(
            'Output: /bopt/nmpc_cmd'
        )

    # =========================================================
    # VELOCITY
    # =========================================================

    def velocity_callback(self, msg):

        if math.isnan(msg.data) or math.isinf(msg.data):
            self.get_logger().warn(
                'Invalid NMPC velocity received'
            )
            return

        # NMPC already provides vehicle velocity in m/s.
        # DO NOT divide by wheel radius here.
        self.velocity = msg.data

    # =========================================================
    # STEERING
    # =========================================================

    def steering_callback(self, msg):

        if math.isnan(msg.data) or math.isinf(msg.data):
            self.get_logger().warn(
                'Invalid NMPC steering received'
            )
            return

        # NMPC publishes steering in degrees (positive = turn body left).
        # In BOPT rear-wheel steering kinematics, the physical joint
        # rotates in the inverted direction to pivot the vehicle body left.
        steering_deg = -msg.data if self.invert_steering else msg.data

        # BOPT internally expects radians.
        steering_rad = math.radians(steering_deg)

        # Apply BOPT steering limit.
        steering_rad = max(
            -self.max_steering_angle,
            min(
                self.max_steering_angle,
                steering_rad
            )
        )

        self.steering_angle = steering_rad

    # =========================================================
    # STATE
    # =========================================================

    def state_callback(self, msg):

        self.current_state = msg.data

    # =========================================================
    # COMMAND
    # =========================================================

    def publish_command(self):

        command = BoptCommand()

        # -----------------------------------------------------
        # NMPC STATE CHECK
        # 'Custom' means active/running by default.
        # If NMPC publishes anything else (e.g. Stop, Idle), stop.
        # -----------------------------------------------------

        if self.current_state == 'Custom':

            command.traction_velocity = self.velocity
            command.steering_angle = self.steering_angle
            command.lift_height = 0.0

        else:

            command.traction_velocity = 0.0
            command.steering_angle = 0.0
            command.lift_height = 0.0

        self.command_pub.publish(command)

    # =========================================================
    # SHUTDOWN
    # =========================================================

    def destroy_node(self):

        try:
            stop_command = BoptCommand()
            stop_command.traction_velocity = 0.0
            stop_command.steering_angle = 0.0
            stop_command.lift_height = 0.0
            self.command_pub.publish(stop_command)
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BoptNmpcController()

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
