#!/usr/bin/env python3
"""
BOPT Keyboard Teleop
====================
Interactive keyboard controller for the BOPT robot.

Controls
--------
  W         : drive forward (keeps current steering)
  S         : drive backward (keeps current steering)
  A         : steer left  + auto-drive if robot is stopped
  D         : steer right + auto-drive if robot is stopped
  Q / E     : raise / lower front lift
  Space     : emergency stop (zero velocity, hold lift)
  R         : full reset – zero speed + zero steering + lift to 0
  Ctrl-C    : quit

The node publishes at 10 Hz:
  /cmd_vel   (geometry_msgs/Twist)   – drive commands
  /lift_cmd  (std_msgs/Float64)      – desired lift height [0…0.095 m]
"""

import sys
import threading
import select
import tty
import termios

import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# ── Key bindings ──────────────────────────────────────────────────────────────
KEY_MAP = {
    'w': 'fwd',
    's': 'back',
    'a': 'left',
    'd': 'right',
    'q': 'lift_up',
    'e': 'lift_down',
    ' ': 'stop',
    'r': 'reset',
}

BANNER = """
╔══════════════════════════════════════════╗
║      BOPT Robot  ·  Keyboard Teleop      ║
╠══════════════════════════════════════════╣
║  W / S   →  Drive Forward / Backward     ║
║  A / D   →  Steer Left / Right + Drive   ║
║  Q / E   →  Lift Up  / Lift Down         ║
║  SPACE   →  Emergency Stop               ║
║  R       →  Full Reset                   ║
║  Ctrl-C  →  Quit                         ║
╚══════════════════════════════════════════╝
"""

# ── Speed / lift configuration ────────────────────────────────────────────────
LINEAR_STEP      = 0.05   # m/s per W/S press
ANGULAR_STEP     = 0.20   # rad/s per A/D press
LIFT_STEP        = 0.005  # m per Q/E press
LINEAR_MAX       = 1.0    # m/s
ANGULAR_MAX      = 3.0    # rad/s
LIFT_MIN         = 0.0
LIFT_MAX         = 0.095

# Speed automatically applied when steering from a standstill
AUTO_DRIVE_SPEED = 0.2    # m/s


class TeleopKeyboardNode(Node):
    """Reads keyboard input and publishes drive + lift commands."""

    def __init__(self):
        super().__init__('teleop_keyboard')

        self._cmd_pub  = self.create_publisher(Twist,   '/cmd_vel',  10)
        self._lift_pub = self.create_publisher(Float64, '/lift_cmd', 10)

        # Publish at 10 Hz → keeps robot_controller watchdog satisfied
        self._timer = self.create_timer(0.1, self._publish_current)

        self._linear  = 0.0
        self._angular = 0.0
        self._lift    = 0.0

        self.get_logger().info('Keyboard teleop ready.')

    # ── Key handling ──────────────────────────────────────────────────────

    def handle_key(self, key: str):
        action = KEY_MAP.get(key.lower())
        if action is None:
            return

        if action == 'fwd':
            # Pure forward – increase speed, keep existing steering
            self._linear = min(self._linear + LINEAR_STEP, LINEAR_MAX)

        elif action == 'back':
            # Pure backward – decrease speed, keep existing steering
            self._linear = max(self._linear - LINEAR_STEP, -LINEAR_MAX)

        elif action == 'left':
            # Steer left; also auto-apply drive if robot is stationary
            self._angular = min(self._angular + ANGULAR_STEP, ANGULAR_MAX)
            if abs(self._linear) < AUTO_DRIVE_SPEED:
                self._linear = AUTO_DRIVE_SPEED

        elif action == 'right':
            # Steer right; also auto-apply drive if robot is stationary
            self._angular = max(self._angular - ANGULAR_STEP, -ANGULAR_MAX)
            if abs(self._linear) < AUTO_DRIVE_SPEED:
                self._linear = AUTO_DRIVE_SPEED

        elif action == 'lift_up':
            self._lift = min(self._lift + LIFT_STEP, LIFT_MAX)

        elif action == 'lift_down':
            self._lift = max(self._lift - LIFT_STEP, LIFT_MIN)

        elif action == 'stop':
            self._linear  = 0.0
            self._angular = 0.0

        elif action == 'reset':
            self._linear  = 0.0
            self._angular = 0.0
            self._lift    = 0.0

        self._print_state()

    # ── Periodic publisher ────────────────────────────────────────────────

    def _publish_current(self):
        twist = Twist()
        twist.linear.x  = self._linear
        twist.angular.z = self._angular
        self._cmd_pub.publish(twist)

        lift_msg = Float64()
        lift_msg.data = self._lift
        self._lift_pub.publish(lift_msg)

    def stop(self):
        """Publish zero velocity before shutdown (hold lift position)."""
        try:
            self._cmd_pub.publish(Twist())
            lift_msg = Float64()
            lift_msg.data = self._lift
            self._lift_pub.publish(lift_msg)
        except Exception:
            pass

    # ── Terminal display ──────────────────────────────────────────────────

    def _print_state(self):
        lift_pct = (self._lift / LIFT_MAX * 100.0) if LIFT_MAX > 0 else 0
        direction = 'FWD' if self._linear > 0 else ('REV' if self._linear < 0 else 'STP')
        steer_dir = 'L' if self._angular > 0 else ('R' if self._angular < 0 else '-')
        sys.stdout.write(
            f'\r  [{direction}] speed: {abs(self._linear):.2f} m/s  '
            f'steer: {steer_dir} {abs(self._angular):.2f} rad/s  '
            f'lift: {self._lift:.4f} m ({lift_pct:.0f}%)   '
        )
        sys.stdout.flush()


# ── Background spin using executor (fixes the crash) ─────────────────────────

def _spin_with_executor(node: Node, stop_event: threading.Event):
    """Spin node in background using an executor; exits cleanly when stop_event is set."""
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    while not stop_event.is_set() and rclpy.ok():
        executor.spin_once(timeout_sec=0.05)
    executor.shutdown()


# ── Raw terminal helpers ──────────────────────────────────────────────────────

def get_key(settings, timeout=0.1):
    """Read a single key from stdin without blocking longer than *timeout* s."""
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboardNode()

    # Use a stop event so we can cleanly shut down the executor thread
    stop_event = threading.Event()
    spin_thread = threading.Thread(
        target=_spin_with_executor,
        args=(node, stop_event),
        daemon=True
    )
    spin_thread.start()

    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)

    try:
        while rclpy.ok():
            key = get_key(settings)
            if key == '\x03':   # Ctrl-C
                break
            if key:
                node.handle_key(key)
    except Exception as exc:
        print(f'\nTeleop error: {exc}')
    finally:
        # 1. Restore terminal
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        # 2. Send a final stop command
        node.stop()
        # 3. Stop the executor thread cleanly
        stop_event.set()
        spin_thread.join(timeout=2.0)
        # 4. Destroy node and shutdown rclpy
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('\nTeleop stopped.')


if __name__ == '__main__':
    main()
