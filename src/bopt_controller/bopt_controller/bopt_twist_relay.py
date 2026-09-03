#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from bopt_interfaces.msg import BoptCommand
from bopt_interfaces.msg import BoptCommandStamped


class BoptTwistRelay(Node):

    def __init__(self):
        super().__init__('bopt_twist_relay')

        # =====================================================
        # PARAMETERS
        # =====================================================

        self.declare_parameter('control_mode', 'auto')
        self.declare_parameter('manual_timeout', 0.5)

        self.control_mode = self.get_parameter(
            'control_mode'
        ).value.lower()

        self.manual_timeout = self.get_parameter(
            'manual_timeout'
        ).value

        # Dynamic parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # =====================================================
        # SUBSCRIBERS
        # =====================================================

        # -----------------------------------------------------
        # BOPT KEY (Manual Teleop - Priority 1)
        # -----------------------------------------------------

        self.key_sub = self.create_subscription(
            BoptCommandStamped,
            'bopt/key_cmd',
            self.key_callback,
            10
        )

        # -----------------------------------------------------
        # NMPC (Autonomous Path Tracking - Default in Auto)
        # -----------------------------------------------------

        self.nmpc_sub = self.create_subscription(
            BoptCommand,
            'bopt/nmpc_cmd',
            self.nmpc_callback,
            10
        )

        # -----------------------------------------------------
        # HYDRAULIC (Independent Lift)
        # -----------------------------------------------------

        self.lift_sub = self.create_subscription(
            BoptCommand,
            'bopt/hydraulic_cmd',
            self.lift_callback,
            10
        )

        # -----------------------------------------------------
        # MODE SWITCH TOPIC
        # -----------------------------------------------------

        self.mode_sub = self.create_subscription(
            String,
            'bopt/set_control_mode',
            self.mode_callback,
            10
        )

        # =====================================================
        # PUBLISHER
        # =====================================================

        self.relay_pub = self.create_publisher(
            BoptCommandStamped,
            'bopt/relay_cmd',
            10
        )

        # =====================================================
        # COMMAND & ACTIVITY STATE
        # =====================================================

        self.key_speed = 0.0
        self.key_steering = 0.0
        self.last_key_time = None

        self.nmpc_speed = 0.0
        self.nmpc_steering = 0.0

        self.lift_height = 0.0

        # =====================================================
        # RELAY LOOP
        # =====================================================

        self.timer = self.create_timer(
            0.02,       # 50 Hz
            self.relay_callback
        )

        self.get_logger().info(
            'BOPT TWIST RELAY (Priority Mux) started'
        )

        self.get_logger().info(
            f'Control mode: {self.control_mode} (auto / manual / nmpc)'
        )

    # =========================================================
    # DYNAMIC PARAMETER CALLBACK
    # =========================================================

    def parameters_callback(self, params):
        for param in params:
            if param.name == 'control_mode':
                mode = str(param.value).lower()
                if mode in ['auto', 'manual', 'nmpc']:
                    self.control_mode = mode
                    self.get_logger().info(
                        f'Control mode switched via parameter to: {self.control_mode}'
                    )
                else:
                    return SetParametersResult(
                        successful=False,
                        reason=f"Invalid control_mode '{mode}'. Use 'auto', 'manual', or 'nmpc'."
                    )
            elif param.name == 'manual_timeout':
                self.manual_timeout = float(param.value)

        return SetParametersResult(successful=True)

    # =========================================================
    # MODE TOPIC CALLBACK
    # =========================================================

    def mode_callback(self, msg):
        mode = msg.data.strip().lower()
        if mode in ['auto', 'manual', 'nmpc']:
            self.control_mode = mode
            self.get_logger().info(
                f'Control mode switched via topic to: {self.control_mode}'
            )
        else:
            self.get_logger().warn(
                f"Ignored invalid control mode '{msg.data}'. Valid options: 'auto', 'manual', 'nmpc'."
            )

    # =========================================================
    # KEY CALLBACK (Manual Teleop)
    # =========================================================

    def key_callback(self, msg):
        self.key_speed = msg.traction_velocity
        self.key_steering = msg.steering_angle
        # Record activity timestamp if key command is active
        if abs(msg.traction_velocity) > 1e-4 or abs(msg.steering_angle) > 1e-4:
            self.last_key_time = self.get_clock().now()

    # =========================================================
    # NMPC CALLBACK (Autonomous Path Tracking)
    # =========================================================

    def nmpc_callback(self, msg):
        self.nmpc_speed = msg.traction_velocity
        self.nmpc_steering = msg.steering_angle

    # =========================================================
    # HYDRAULIC CALLBACK
    # =========================================================

    def lift_callback(self, msg):
        self.lift_height = msg.lift_height

    # =========================================================
    # RELAY (Priority Multiplexing)
    # =========================================================

    def relay_callback(self):
        now = self.get_clock().now()

        # Check if manual teleop is actively being used
        manual_active = False
        if self.last_key_time is not None:
            elapsed_key = (now - self.last_key_time).nanoseconds / 1e9
            if elapsed_key < self.manual_timeout:
                manual_active = True

        # -----------------------------------------------------
        # Priority Routing Decision
        # -----------------------------------------------------
        if self.control_mode == 'auto':
            if manual_active:
                # Priority 1: Teleop key pressed -> override NMPC
                out_speed = self.key_speed
                out_steering = self.key_steering
            else:
                # Default: Follow NMPC
                out_speed = self.nmpc_speed
                out_steering = self.nmpc_steering

        elif self.control_mode == 'manual':
            if manual_active:
                out_speed = self.key_speed
                out_steering = self.key_steering
            else:
                out_speed = 0.0
                out_steering = 0.0

        elif self.control_mode == 'nmpc':
            out_speed = self.nmpc_speed
            out_steering = self.nmpc_steering

        else:
            self.get_logger().error(f'Unknown control mode: {self.control_mode}')
            out_speed = 0.0
            out_steering = 0.0

        # =====================================================
        # ASSEMBLE OUTPUT COMMAND
        # =====================================================
        command = BoptCommandStamped()
        command.header.stamp = now.to_msg()
        command.header.frame_id = 'base_link'

        command.traction_velocity = out_speed
        command.steering_angle = out_steering
        command.lift_height = self.lift_height

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()