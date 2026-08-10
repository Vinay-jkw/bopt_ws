#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64


class BOPTKeyboard(Node):

    def __init__(self):
        super().__init__('bopt_keyboard')

        self.speed = 0.0
        self.steering = 0.0
        self.lift = 0.0

        self.speed_step = 0.10
        self.steering_step = 0.30
        self.lift_step = 0.01

        self.max_speed = 3.0
        self.max_steering = 0.6
        self.max_lift = 0.095

        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            '/cmd_ackermann',
            10
        )

        self.lift_pub = self.create_publisher(
            Float64,
            '/lift_cmd',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        self.get_logger().info('BOPT Keyboard Controller Started')
        self.get_logger().info(
            'W/S: Drive | A/D: Steering | Q/E: Lift | '
            'Space: Stop | R: Reset'
        )

    def get_key(self):

        if select.select(
            [sys.stdin],
            [],
            [],
            0
        )[0]:
            return sys.stdin.read(1)

        return None

    def control_loop(self):

        key = self.get_key()

        lift_changed = False

        if key == 'w':
            self.speed += self.speed_step

        elif key == 's':
            self.speed -= self.speed_step

        elif key == 'a':
            self.steering += self.steering_step

        elif key == 'd':
            self.steering -= self.steering_step

        elif key == 'q':
            old_lift = self.lift
            self.lift += self.lift_step

            if self.lift > self.max_lift:
                self.lift = self.max_lift

            if self.lift != old_lift:
                lift_changed = True

        elif key == 'e':
            old_lift = self.lift
            self.lift -= self.lift_step

            if self.lift < 0.0:
                self.lift = 0.0

            if self.lift != old_lift:
                lift_changed = True

        elif key == ' ':
            self.speed = 0.0
            self.steering = 0.0

        elif key == 'r':
            self.speed = 0.0
            self.steering = 0.0

            if self.lift != 0.0:
                self.lift = 0.0
                lift_changed = True

        self.speed = max(
            -self.max_speed,
            min(self.max_speed, self.speed)
        )

        self.steering = max(
            -self.max_steering,
            min(self.max_steering, self.steering)
        )

        self.publish_drive()

        if lift_changed:
            self.publish_lift()

    def publish_drive(self):

        msg = AckermannDriveStamped()

        msg.drive.speed = self.speed
        msg.drive.steering_angle = self.steering

        self.ackermann_pub.publish(msg)

    def publish_lift(self):

        msg = Float64()
        msg.data = self.lift

        self.lift_pub.publish(msg)

        self.get_logger().info(
            f'LIFT TARGET: {self.lift:.3f} m'
        )

    def shutdown(self):

        self.speed = 0.0
        self.steering = 0.0

        msg = AckermannDriveStamped()

        msg.drive.speed = 0.0
        msg.drive.steering_angle = 0.0

        self.ackermann_pub.publish(msg)

        try:
            termios.tcsetattr(
                sys.stdin,
                termios.TCSADRAIN,
                self.settings
            )
        except Exception:
            pass


def main(args=None):

    rclpy.init(args=args)

    node = BOPTKeyboard()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()