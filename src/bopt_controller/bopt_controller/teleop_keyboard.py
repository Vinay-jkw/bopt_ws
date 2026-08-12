#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float64

# Instructions to print on startup
MSG = """
------------------------------------
BOPT 8-Way Keyboard Controller
------------------------------------
Moving around (Drive & Steer):
   U    I    O
   J    K    L
   M    ,    .

U/I/O : Forward-Left / Forward / Forward-Right
J/K/L : Steer Left / STOP / Steer Right
M/,/. : Back-Left / Backward / Back-Right

Lift Control:
Q : Raise Lift
E : Lower Lift

Space / K : E-Stop (Zero speed/steering)
R         : Reset all (Zero speed/steering/lift)
CTRL-C    : Quit
------------------------------------
"""

class BOPTKeyboard(Node):
    def __init__(self):
        super().__init__('bopt_keyboard')

        # --- State Variables ---
        self.speed = 0.0
        self.steering = 0.0
        self.lift = 0.0

        # --- Control Parameters ---
        self.speed_step = 0.10
        self.steering_step = 0.10
        self.lift_step = 0.01

        self.max_speed = 3.0
        self.max_steering = 0.6
        self.max_lift = 0.094

        # --- Direction Mappings (Speed_multiplier, Steering_multiplier) ---
        self.moveBindings = {
            'i': (1, 0),
            'o': (1, 1),
            'j': (0, -1),
            'l': (0, 1),
            'u': (1, -1),
            ',': (-1, 0),
            '.': (-1, 1),
            'm': (-1, -1),
        }

        self.liftBindings = {
            'q': 1,
            'e': -1,
        }

        # --- Publishers ---
        self.ackermann_pub = self.create_publisher(AckermannDriveStamped, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift_cmd', 10)

        # --- Terminal Setup ---
        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print(MSG)

        # Loop at 20 Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def get_key(self):
        # Non-blocking terminal read
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def control_loop(self):
        key = self.get_key()
        lift_changed = False

        if key:
            if key in self.moveBindings:
                self.speed += self.moveBindings[key][0] * self.speed_step
                self.steering += self.moveBindings[key][1] * self.steering_step
            elif key in self.liftBindings:
                self.lift += self.liftBindings[key] * self.lift_step
                lift_changed = True
            elif key == 'k' or key == ' ':
                self.speed = 0.0
                self.steering = 0.0
            elif key == 'r':
                self.speed = 0.0
                self.steering = 0.0
                if self.lift != 0.0:
                    self.lift = 0.0
                    lift_changed = True
            elif key == '\x03': # CTRL-C
                raise KeyboardInterrupt

        # Clamp values
        self.speed = self.clamp(self.speed, -self.max_speed, self.max_speed)
        self.steering = self.clamp(self.steering, -self.max_steering, self.max_steering)
        
        old_lift = self.lift
        self.lift = self.clamp(self.lift, 0.0, self.max_lift)
        if self.lift != old_lift:
            lift_changed = True

        self.publish_drive()

        if lift_changed:
            self.publish_lift()

        self.print_status()

    def publish_drive(self):
        msg = AckermannDriveStamped()
        msg.drive.speed = float(self.speed)
        msg.drive.steering_angle = float(self.steering)
        self.ackermann_pub.publish(msg)

    def publish_lift(self):
        msg = Float64()
        msg.data = float(self.lift)
        self.lift_pub.publish(msg)

    def print_status(self):
        status = f"\rSpeed: {self.speed: .2f} m/s | Steering: {self.steering: .2f} rad | Lift: {self.lift: .3f} m    "
        sys.stdout.write(status)
        sys.stdout.flush()

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def shutdown(self):
        self.speed = 0.0
        self.steering = 0.0
        self.publish_drive()
        print("\nStopping robot...")
        # Restore terminal settings
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
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
        print("\nExiting BOPT Keyboard...")
        try:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()