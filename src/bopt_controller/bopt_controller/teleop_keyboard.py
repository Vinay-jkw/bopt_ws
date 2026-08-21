
from ackermann_msgs import msg
import math
import select
import sys
import termios
import tty
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

MSG = """
-----------------------------------------------------------------------
                    BOPT Keyboard Teleoperation
-----------------------------------------------------------------------
Drive & Steer (Keypad or WASD / Arrows):
      U    I    O                W
      J    K    L       or     A S D     or    [Arrow Keys]
      M    ,    .

  I / W / Up       : Forward
  , / S / Down     : Backward
  J / A / Left     : Steer Left  (+ angle)
  L / D / Right    : Steer Right (- angle)
  U                : Forward-Left
  O                : Forward-Right
  M                : Back-Left
  .                : Back-Right

Lift Control:
  Q                : Raise Lift
  E                : Lower Lift

Emergency & Reset:
  Space / K        : Stop Drive (Zero speed & steering)
  R                : Reset All (Zero speed, steering & lift)
  CTRL-C           : Quit
-----------------------------------------------------------------------
"""


class BOPTKeyboard(Node):
    """Keyboard teleoperation node for BOPT pallet truck."""

    def __init__(self, original_terminal_settings=None):
        super().__init__('bopt_keyboard')

        # --- Parameters ---
        self.declare_parameter('speed_step', 0.10)
        self.declare_parameter('steering_step', 0.05)
        self.declare_parameter('lift_step', 0.01)
        self.declare_parameter('max_speed', 1.0)
        self.declare_parameter('max_steering', 1.57)
        self.declare_parameter('max_lift', 0.095)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('key_timeout', 0.15)

        self.target_speed = 0.0
        self.target_steering = 0.0

        self.declare_parameter('acceleration', 0.8)
        self.declare_parameter('deceleration', 1.2)

        self.acceleration = self.get_parameter('acceleration').value
        self.deceleration = self.get_parameter('deceleration').value

        self.speed_step = self.get_parameter('speed_step').value
        self.steering_step = self.get_parameter('steering_step').value
        self.lift_step = self.get_parameter('lift_step').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_steering = self.get_parameter('max_steering').value
        self.max_lift = self.get_parameter('max_lift').value
        self.control_rate = self.get_parameter('control_rate').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.key_timeout = self.get_parameter('key_timeout').value
        self.last_key_time = self.get_clock().now()

        # --- State Variables ---
        self.speed = 0.0
        self.steering = 0.0
        self.lift = 0.0

        # --- Direction Mappings: (speed_multiplier, steering_multiplier) ---
        # Note: In standard ROS (REP 103), Left is positive (+) and Right is negative (-).
        self.move_bindings = {
            # -----------------------
            # Forward movement keys
            # -----------------------
            'u': (1.0, 1.0),     # Forward-Left
            'i': (1.0, 0.0),     # Forward
            'o': (1.0, -1.0),    # Forward-Right
            # -----------------------
            # Backward movement keys
            # -----------------------
            '.': (-1.0, 1.0),   # Backward-Left
            ',': (-1.0, 0.0),    # Backward
            'm': (-1.0, -1.0),    # Backward-Right
            # -----------------------
            # Turning keys (zero speed)
            # -----------------------
            'j': (0.0, -1.0),    # Left
            'l': (0.0, 1.0),     # Right
            'k': (0.0, 0.0),     # Stop
        }

        self.lift_bindings = {
            'q': 1,
            'e': -1,
        }

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.lift_pub = self.create_publisher(
            Float64,
            '/lift_cmd',
            10
        )

        # --- Terminal Setup ---
        self.settings = original_terminal_settings
        if self.settings is None:
            try:
                self.settings = termios.tcgetattr(sys.stdin)
            except Exception:
                self.settings = None

        if self.settings is not None:
            try:
                tty.setcbreak(sys.stdin.fileno())
            except Exception:
                pass

        sys.stdout.write(MSG)
        sys.stdout.flush()

        # Control timer
        timer_period = 1.0 / self.control_rate
        self.timer = self.create_timer(timer_period, self.control_loop)

    def get_key(self):
        """Read the newest available keyboard command.

        Drain all currently buffered keyboard input so stale commands
        cannot remain queued and override a newer command.
        """

        latest_key = None

        while select.select([sys.stdin], [], [], 0.0)[0]:
            try:
                ch = sys.stdin.read(1)

                if ch == '\x03':
                    return '\x03'

                # Handle ANSI arrow keys
                if ch == '\x1b':
                    sequence = ''

                    # Read the rest of the escape sequence if available
                    if select.select([sys.stdin], [], [], 0.005)[0]:
                        ch2 = sys.stdin.read(1)

                        if ch2 == '[':
                            if select.select([sys.stdin], [], [], 0.005)[0]:
                                ch3 = sys.stdin.read(1)

                                if ch3 == 'A':
                                    latest_key = 'up'
                                elif ch3 == 'B':
                                    latest_key = 'down'
                                elif ch3 == 'C':
                                    latest_key = 'right'
                                elif ch3 == 'D':
                                    latest_key = 'left'

                    continue

                # Normal keyboard key
                latest_key = ch

            except Exception:
                break

        return latest_key

    def control_loop(self):
        """Periodic control callback to read keyboard and publish commands."""
        lift_changed=False
        key = self.get_key()

        if key:
            self.last_key_time = self.get_clock().now()

            if key == '\x03':
                raise KeyboardInterrupt

            lower_key = key.lower()

            if lower_key in self.move_bindings:
                speed_mult, yaw_mult = self.move_bindings[lower_key]

                self.target_speed = speed_mult * self.max_speed
                self.target_steering = yaw_mult * self.max_yaw_rate

            elif lower_key in self.lift_bindings:
                self.lift += self.lift_bindings[lower_key] * self.lift_step
                lift_changed = True

            elif lower_key == ' ':
                self.target_speed = 0.0
                self.target_steering = 0.0

            elif lower_key == 'r':
                self.target_speed = 0.0
                self.target_steering = 0.0
                self.lift = 0.0
                lift_changed = True

        # No key received recently → stop
        elapsed = (
            self.get_clock().now() - self.last_key_time
        ).nanoseconds / 1e9

        if elapsed > self.key_timeout:
            self.target_speed = 0.0
            self.target_steering = 0.0

        # Clamp values
        self.speed = self.clamp(self.speed, -self.max_speed, self.max_speed)
        self.steering = self.clamp(self.steering, -self.max_steering, self.max_steering)

        old_lift = self.lift
        self.lift = self.clamp(self.lift, 0.0, self.max_lift)
        if self.lift != old_lift:
            lift_changed = True

        # Smooth acceleration for speed
        dt = 1.0 / self.control_rate

        # Speed ramp
        speed_error = self.target_speed - self.speed

        if abs(speed_error) > 1e-6:
            if abs(self.target_speed) > abs(self.speed):
                step = self.acceleration * dt
            else:
                step = self.deceleration * dt

            if abs(speed_error) <= step:
                self.speed = self.target_speed
            else:
                self.speed += math.copysign(step, speed_error)

        # Steering can return smoothly to center
        # Smooth steering ramp
        steering_error = self.target_steering - self.steering
        steering_rate = 1.2
        steering_step = steering_rate * dt

        if abs(steering_error) <= steering_step:
            self.steering = self.target_steering
        else:
            self.steering += math.copysign(
                steering_step,
                steering_error
            )

        self.publish_drive()

        if lift_changed:
            self.publish_lift()

        self.print_status()

    def publish_drive(self):
        msg = Twist()

        msg.linear.x = float(self.speed)
        msg.linear.y = 0.0
        msg.linear.z = 0.0

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(self.steering)

        self.cmd_vel_pub.publish(msg)

    def publish_lift(self):
        """Publish lift position command."""
        msg = Float64()
        msg.data = float(self.lift)
        self.lift_pub.publish(msg)

    def print_status(self):
        """Print formatted current state to terminal."""
        steer_deg = math.degrees(self.steering)
        status = (
            f"\rSpeed: {self.speed:+.2f} m/s | "
            f"Steering: {self.steering:+.2f} rad ({steer_deg:+.1f}°) | "
            f"Lift: {self.lift:.3f} m     "
        )
        sys.stdout.write(status)
        sys.stdout.flush()

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        """Clamp value between minimum and maximum."""
        return max(minimum, min(maximum, value))

    def shutdown(self):
        """Stop the robot and restore terminal settings."""
        self.speed = 0.0
        self.steering = 0.0
        try:
            self.publish_drive()
        except Exception:
            pass
        sys.stdout.write("\nStopping robot...\n")
        sys.stdout.flush()

        if self.settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            except Exception:
                pass


def main(args=None):
    original_settings = None
    try:
        original_settings = termios.tcgetattr(sys.stdin)
    except Exception:
        pass

    rclpy.init(args=args)
    node = None

    try:
        node = BOPTKeyboard(original_terminal_settings=original_settings)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node:
            node.get_logger().error(f'Exception in BOPT Keyboard: {e}')
    finally:
        sys.stdout.write("\nExiting BOPT Keyboard...\n")
        sys.stdout.flush()
        if node is not None:
            try:
                node.shutdown()
                node.destroy_node()
            except Exception:
                pass
        if original_settings is not None:
            try:
                termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
