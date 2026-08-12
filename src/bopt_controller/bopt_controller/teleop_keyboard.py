#!/usr/bin/env python3

import sys
import select
import termios
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

MSG = """
------------------------------------
BOPT Twist Keyboard Controller
------------------------------------
Moving around:
   W    
 A S D  

W/S : Forward / Backward (Linear)
A/D : Turn Left / Right (Angular)
Q/E : Raise / Lower Lift

Space : E-Stop (Zero velocity)
R     : Reset all (Zero velocity/lift)
CTRL-C: Quit
------------------------------------
"""

class BOPTKeyboard(Node):
    def __init__(self):
        super().__init__('bopt_keyboard')

        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.lift = 0.0

        self.linear_step = 0.10
        self.angular_step = 0.20
        self.lift_step = 0.01

        self.max_linear = 1.0
        self.max_angular = 1.0
        self.max_lift = 0.095

        self.twist_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.lift_pub = self.create_publisher(Float64, '/lift_cmd', 10)

        self.settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

        print(MSG)
        self.timer = self.create_timer(0.05, self.control_loop)

    def get_key(self):
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def control_loop(self):
        key = self.get_key()
        lift_changed = False

        if key:
            if key == 'w':
                self.linear_vel += self.linear_step
            elif key == 's':
                self.linear_vel -= self.linear_step
            elif key == 'a':
                self.angular_vel += self.angular_step
            elif key == 'd':
                self.angular_vel -= self.angular_step
            elif key == 'q':
                self.lift += self.lift_step
                lift_changed = True
            elif key == 'e':
                self.lift -= self.lift_step
                lift_changed = True
            elif key == ' ':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            elif key == 'r':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
                if self.lift != 0.0:
                    self.lift = 0.0
                    lift_changed = True
            elif key == '\x03': 
                raise KeyboardInterrupt

        self.linear_vel = self.clamp(self.linear_vel, -self.max_linear, self.max_linear)
        self.angular_vel = self.clamp(self.angular_vel, -self.max_angular, self.max_angular)
        
        old_lift = self.lift
        self.lift = self.clamp(self.lift, 0.0, self.max_lift)
        if self.lift != old_lift:
            lift_changed = True

        self.publish_drive()

        if lift_changed:
            self.publish_lift()

        self.print_status()

    def publish_drive(self):
        msg = Twist()
        msg.linear.x = float(self.linear_vel)
        msg.angular.z = float(self.angular_vel)
        self.twist_pub.publish(msg)

    def publish_lift(self):
        msg = Float64()
        msg.data = float(self.lift)
        self.lift_pub.publish(msg)

    def print_status(self):
        status = f"\rLinear: {self.linear_vel: .2f} m/s | Angular: {self.angular_vel: .2f} rad/s | Lift: {self.lift: .3f} m    "
        sys.stdout.write(status)
        sys.stdout.flush()

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def shutdown(self):
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.publish_drive()
        print("\nStopping robot...")
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
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()