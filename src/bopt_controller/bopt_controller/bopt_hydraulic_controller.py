#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

from bopt_interfaces.msg import BoptCommand
from bopt_interfaces.srv import SetLiftHeight


class BoptHydraulicController(Node):

    def __init__(self):
        super().__init__('bopt_hydraulic_controller')

        # --------------------------------------------------
        # Parameters
        # --------------------------------------------------

        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.095)

        self.lift_min = self.get_parameter(
            'lift_min'
        ).value

        self.lift_max = self.get_parameter(
            'lift_max'
        ).value

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------

        self.command_pub = self.create_publisher(
            BoptCommand,
            'bopt/hydraulic_cmd',
            10
        )

        # --------------------------------------------------
        # Service
        # --------------------------------------------------

        self.height_srv = self.create_service(
            SetLiftHeight,
            'set_lift_height',
            self.height_callback
        )

        self.lift_cmd_sub = self.create_subscription(
            Float64,
            'lift_cmd',
            self.lift_cmd_callback,
            10
        )

        # --------------------------------------------------
        # State
        # --------------------------------------------------

        self.current_height = 0.0

        self.get_logger().info(
            'BOPT HYDRAULIC CONTROLLER started'
        )

        self.get_logger().info(
            f'Lift range: {self.lift_min:.3f} '
            f'to {self.lift_max:.3f} m'
        )

    # ======================================================
    # SET LIFT HEIGHT
    # ======================================================
    def lift_cmd_callback(self, msg):

        target_height = msg.data

        # --------------------------------------------------
        # Validate
        # --------------------------------------------------

        if target_height < self.lift_min:
            target_height = self.lift_min

        if target_height > self.lift_max:
            target_height = self.lift_max

        # --------------------------------------------------
        # Update state
        # --------------------------------------------------

        self.current_height = target_height

        # --------------------------------------------------
        # Publish BOPT hydraulic command
        # --------------------------------------------------

        command = BoptCommand()

        command.traction_velocity = 0.0
        command.steering_angle = 0.0
        command.lift_height = self.current_height

        self.command_pub.publish(command)

        self.get_logger().info(
            f'Keyboard lift command: '
            f'{self.current_height:.3f} m'
        )

    def height_callback(self, request, response):

        target_height = request.height

        # --------------------------------------------------
        # Validate
        # --------------------------------------------------

        if target_height < self.lift_min:

            response.success = False
            response.message = (
                f'Height {target_height:.3f} m '
                f'is below minimum '
                f'{self.lift_min:.3f} m'
            )

            self.get_logger().warn(response.message)

            return response

        if target_height > self.lift_max:

            response.success = False
            response.message = (
                f'Height {target_height:.3f} m '
                f'is above maximum '
                f'{self.lift_max:.3f} m'
            )

            self.get_logger().warn(response.message)

            return response

        # --------------------------------------------------
        # Accept exact requested height
        # --------------------------------------------------

        self.current_height = target_height

        msg = BoptCommand()

        msg.traction_velocity = 0.0
        msg.steering_angle = 0.0
        msg.lift_height = self.current_height

        self.command_pub.publish(msg)

        response.success = True
        response.message = (
            f'Lift target set to '
            f'{self.current_height:.3f} m'
        )

        self.get_logger().info(
            response.message
        )

        return response


def main(args=None):

    rclpy.init(args=args)

    node = BoptHydraulicController()

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