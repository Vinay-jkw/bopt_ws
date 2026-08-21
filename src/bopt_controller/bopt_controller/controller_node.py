import math

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class BOPTController(Node):
    """Controller node for BOPT (reverse tricycle / rear-steered pallet truck)."""

    def __init__(self):
        super().__init__('bopt_controller')

        # --- Declare ROS 2 Parameters ---
        self.declare_parameter('wheel_radius', 0.115)
        self.declare_parameter('wheelbase', 1.542)
        self.declare_parameter('max_wheel_velocity', 3.0)
        self.declare_parameter('max_steering_angle', 1.5708)
        self.declare_parameter('control_dt', 0.05)
        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.095)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('steering_tolerance', 0.03)
        self.declare_parameter('steering_delay', 0.15)

        # Retrieve parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.max_wheel_velocity = self.get_parameter('max_wheel_velocity').value
        self.max_steering_angle = self.get_parameter('max_steering_angle').value
        self.control_dt = self.get_parameter('control_dt').value
        self.lift_min = self.get_parameter('lift_min').value
        self.lift_max = self.get_parameter('lift_max').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.steering_tolerance = self.get_parameter('steering_tolerance').value
        self.steering_delay = self.get_parameter('steering_delay').value

        # --- State Variables ---
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_lift_position = 0.0
        self.current_wheel_velocity = 0.0
        self.current_steering_angle = 0.0
        self.target_steering_angle = 0.0
        self.target_wheel_velocity = 0.0
        self.steering_reached_time = None
        self.is_stopped = True

        # --- Subscribers ---
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_twist_callback,
            10
        )

        self.create_subscription(
            Float64,
            '/lift_cmd',
            self.lift_cmd_callback,
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # --- Publishers ---
        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            '/traction_joint_controller/commands',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            '/steering_joint_controller/commands',
            10
        )

        self.lift_pub = self.create_publisher(
            JointTrajectory,
            '/lift_joint_controller/joint_trajectory',
            10
        )

        # Periodic control & safety timer
        self.control_timer = self.create_timer(
            self.control_dt,
            self.control_loop
        )

        # Initialize hardware in stopped state
        self.publish_traction(0.0)
        self.publish_steering(0.0)

        self.get_logger().info(
            'BOPT Controller initialized: wheelbase=%.3fm, wheel_radius=%.3fm' %
            (self.wheelbase, self.wheel_radius)
        )


    def cmd_vel_twist_callback(self, msg: Twist):
        """Convert /cmd_vel into BOPT steering and traction commands."""

        v = msg.linear.x
        yaw_rate = msg.angular.z

        if math.isnan(v) or math.isinf(v):
            self.get_logger().warn('Invalid linear.x command, ignoring.')
            return

        if math.isnan(yaw_rate) or math.isinf(yaw_rate):
            self.get_logger().warn('Invalid angular.z command, ignoring.')
            return

        self.last_cmd_vel_time = self.get_clock().now()

        # ---------------------------------------------------------
        # BOPT steering and driving kinematics
        # Reference:
        # steering = atan2(-wheelbase * yaw_rate, v)
        # ---------------------------------------------------------

        if abs(yaw_rate) < 1e-6:
            steering_angle = 0.0
            drive_linear_velocity = v

        elif abs(v) > 1e-6:
            steering_angle = math.atan2(
                -self.wheelbase * yaw_rate,
                v
            )

            drive_linear_velocity = math.copysign(
                math.hypot(v, self.wheelbase * yaw_rate),
                v
            )

            # Keep the steering joint within its physical ±90° range.
            # When reversing, atan2() can return an angle beyond ±90°.
            # A 180° steering transformation is equivalent when the
            # wheel rotation direction is reversed.
            if steering_angle > math.pi / 2:
                steering_angle -= math.pi
            elif steering_angle < -math.pi / 2:
                steering_angle += math.pi

        else:
            # Pure rotation
            steering_angle = (
                -math.copysign(
                    self.max_steering_angle,
                    yaw_rate
                )
            )

            drive_linear_velocity = abs(     
                self.wheelbase * yaw_rate
            )

        # Steering limit
        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        # Convert linear wheel velocity [m/s]
        # to wheel angular velocity [rad/s]
        wheel_velocity = (
            drive_linear_velocity / self.wheel_radius
        )

        # Wheel velocity limit
        wheel_velocity = self.clamp(
            wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        # Smooth traction reduction while steering is changing.
        # IMPORTANT: compare the new target against the *actual measured*
        # steering angle (kept up to date by joint_state_callback) BEFORE
        # updating any state — otherwise steering_error is always 0 and the
        # truck drives at full speed before the wheel has physically turned.
        steering_error = abs(steering_angle - self.current_steering_angle)

        self.current_wheel_velocity = wheel_velocity
        self.is_stopped = abs(wheel_velocity) < 1e-4

        self.publish_steering(steering_angle)

        # Maximum steering error we consider significant
        # max_steering_error = 0.5  # rad ≈ 28.6°

        # # Calculate smooth speed factor
        # steering_factor = max(
        #     0.0,
        #     1.0 - (steering_error / max_steering_error)
        # )

        # # Don't reduce speed below 40% during steering
        # steering_factor = 0.4 + (0.6 * steering_factor)

        # smooth_wheel_velocity = wheel_velocity * steering_factor
        now = self.get_clock().now()

        if steering_error > self.steering_tolerance:
            self.steering_reached_time = None
            smooth_wheel_velocity = 0.0
        else:
            if self.steering_reached_time is None:
                self.steering_reached_time = now
            settled_for = (now - self.steering_reached_time).nanoseconds / 1e9
            smooth_wheel_velocity = wheel_velocity if settled_for >= self.steering_delay else 0.0

        self.publish_traction(smooth_wheel_velocity)

        self.get_logger().debug(
            f'BOPT | v={v:.3f} m/s | yaw_rate={yaw_rate:.3f} rad/s | '
            f'steering={steering_angle:.3f} rad | '
            f'wheel={wheel_velocity:.3f} rad/s'
        )

    

    def lift_cmd_callback(self, msg: Float64):
        """Process lift command (height in meters)."""
        target_height = msg.data

        if math.isnan(target_height) or math.isinf(target_height):
            self.get_logger().warn('Received invalid NaN/Inf lift command, ignoring.')
            return

        lift_position = self.clamp(
            target_height,
            self.lift_min,
            self.lift_max
        )

        distance = abs(lift_position - self.current_lift_position)
        duration_s = max(1.0, distance / 0.025 + 0.5)

        self.publish_lift(lift_position, duration_s)

        self.get_logger().info(
            f'LIFT | target_pos={lift_position:.4f} m | duration={duration_s:.2f} s'
        )

    def joint_state_callback(self, msg: JointState):
        """Track current steering and lift joint positions."""

        if 'drive_wheel_Ass_joint' in msg.name:
            index = msg.name.index('drive_wheel_Ass_joint')

            if index < len(msg.position):
                self.current_steering_angle = msg.position[index]

        if 'front_lift_joint' in msg.name:
            index = msg.name.index('front_lift_joint')
            if index < len(msg.position):
                self.current_lift_position = msg.position[index]

    def control_loop(self):
        """Watchdog timer for cmd_vel timeout safety."""
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout and not self.is_stopped:
            self.publish_traction(0.0)
            self.current_wheel_velocity = 0.0
            self.is_stopped = True
            self.get_logger().debug('cmd_vel timeout: Robot traction stopped for safety.')

    def publish_traction(self, velocity: float):
        """Publish wheel velocity command to traction joint controller."""
        msg = Float64MultiArray()
        msg.data = [float(velocity)]
        self.traction_pub.publish(msg)

    def publish_steering(self, angle: float):
        """Publish steering position command to steering joint controller."""
        msg = Float64MultiArray()
        msg.data = [float(angle)]
        self.steering_pub.publish(msg)

    def publish_lift(self, position: float, duration_s: float = 2.0):
        """Publish trajectory command to lift joint controller."""
        msg = JointTrajectory()
        msg.joint_names = ['front_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.velocities = [0.0]

        sec = int(duration_s)
        nanosec = int((duration_s - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)

        msg.points = [point]
        self.lift_pub.publish(msg)

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        """Clamp value between minimum and maximum."""
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)

    node = BOPTController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.publish_traction(0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()