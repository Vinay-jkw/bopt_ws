import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray


class BOPTController(Node):

    def __init__(self):
        super().__init__('bopt_controller')

        # ==========================================================
        # ROBOT PARAMETERS
        # ==========================================================

        # Drive wheel
        self.wheel_radius = 0.115
        self.max_wheel_velocity = 3.0

        # Steering
        self.steering_reference_length = 1.0
        self.max_steering_angle = 0.6

        # Lift
        self.max_lift_effort = 30000.0

        # ==========================================================
        # MIMIC PARAMETERS
        # ==========================================================

        # Lift range
        self.lift_min = 0.0
        self.lift_max = 0.095

        # Mimic joint angle limits
        self.mimic_min_angle = 0.0
        self.mimic_max_angle = 0.75391095

        # Relationship:
        #
        # lift = 0.0 m
        #       -> mimic_min_angle
        #
        # lift = 0.095 m
        #       -> mimic_max_angle
        #
        # We will replace this with the actual mechanical
        # relationship after the mechanism is stable.

        # ==========================================================
        # SAFETY
        # ==========================================================

        self.cmd_vel_timeout = 0.5

        self.last_cmd_vel_time = self.get_clock().now()

        # ==========================================================
        # CURRENT JOINT STATE
        # ==========================================================

        self.current_lift_position = 0.0

        # ==========================================================
        # SUBSCRIBERS
        # ==========================================================

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.lift_cmd_sub = self.create_subscription(
            Float64,
            '/lift_cmd',
            self.lift_cmd_callback,
            10
        )

        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # ==========================================================
        # PUBLISHERS
        # ==========================================================

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
            Float64MultiArray,
            '/lift_joint_controller/commands',
            10
        )

        self.mimic_pub = self.create_publisher(
            Float64MultiArray,
            '/mimic_joint_controller/commands',
            10
        )

        # ==========================================================
        # SAFETY / CONTROL TIMER
        # ==========================================================

        self.control_timer = self.create_timer(
            0.05,       # 20 Hz
            self.control_loop
        )

        # ==========================================================
        # IMPORTANT:
        # STOP ROBOT IMMEDIATELY AT STARTUP
        # ==========================================================

        self.publish_traction(0.0)
        self.publish_steering(0.0)

        self.get_logger().info(
            'BOPT Controller started - ROBOT STOPPED'
        )

    # ==============================================================
    # CMD_VEL
    # ==============================================================

    def cmd_vel_callback(self, msg):

        self.last_cmd_vel_time = self.get_clock().now()

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # ----------------------------------------------------------
        # DRIVE
        # ----------------------------------------------------------

        wheel_velocity = (
            linear_velocity / self.wheel_radius
        )

        wheel_velocity = self.clamp(
            wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        self.publish_traction(wheel_velocity)

        # ----------------------------------------------------------
        # STEERING
        # ----------------------------------------------------------

        if abs(linear_velocity) < 1e-6:

            steering_angle = 0.0

        elif abs(angular_velocity) < 1e-6:

            steering_angle = 0.0

        else:

            steering_angle = math.atan(
                self.steering_reference_length
                * angular_velocity
                / linear_velocity
            )

        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        self.publish_steering(steering_angle)

        self.get_logger().info(
            f'DRIVE | '
            f'v={linear_velocity:.3f} m/s | '
            f'wheel={wheel_velocity:.3f} rad/s | '
            f'steering={steering_angle:.3f} rad'
        )

    # ==============================================================
    # LIFT COMMAND
    # ==============================================================

    def lift_cmd_callback(self, msg):

        lift_effort = self.clamp(
            msg.data,
            -self.max_lift_effort,
            self.max_lift_effort
        )

        self.publish_lift(lift_effort)

        self.get_logger().info(
            f'LIFT | effort={lift_effort:.1f}'
        )

    # ==============================================================
    # JOINT STATES
    # ==============================================================

    def joint_state_callback(self, msg):

        if 'front_lift_joint' not in msg.name:
            return

        index = msg.name.index('front_lift_joint')

        if index < len(msg.position):

            self.current_lift_position = msg.position[index]

    # ==============================================================
    # MAIN CONTROL LOOP
    # ==============================================================

    def control_loop(self):

        # ----------------------------------------------------------
        # 1. SAFETY WATCHDOG
        # ----------------------------------------------------------

        current_time = self.get_clock().now()

        elapsed = (
            current_time - self.last_cmd_vel_time
        ).nanoseconds / 1e9

        if elapsed > self.cmd_vel_timeout:

            # Stop drive
            self.publish_traction(0.0)

            # Center steering
            self.publish_steering(0.0)

        # ----------------------------------------------------------
        # 2. MIMIC CONTROL
        # ----------------------------------------------------------
        #
        # Reads current lift position from joint_states and computes
        # the corresponding shaft angle, then sends it to
        # mimic_joint_controller so Gazebo physically moves the joints.
        #
        # Note: gazebo_ros2_control does NOT enforce URDF <mimic> tags
        # at runtime — we must command them explicitly here.

        mimic_angle = self.calculate_mimic_angle(
            self.current_lift_position
        )

        self.publish_mimic(mimic_angle)

        self.get_logger().debug(
            f'MIMIC | lift={self.current_lift_position:.4f} m | '
            f'shaft_angle={mimic_angle:.4f} rad'
        )

    # ==============================================================
    # MIMIC CALCULATION
    # ==============================================================

    def calculate_mimic_angle(self, lift_position):

        # Clamp lift position

        lift_position = self.clamp(
            lift_position,
            self.lift_min,
            self.lift_max
        )

        # Normalize:
        #
        # 0.0   -> 0
        # 0.095 -> 1

        lift_ratio = (
            lift_position - self.lift_min
        ) / (
            self.lift_max - self.lift_min
        )

        # Linear mimic relationship

        angle = (
            self.mimic_min_angle
            +
            lift_ratio
            * (
                self.mimic_max_angle
                - self.mimic_min_angle
            )
        )

        return self.clamp(
            angle,
            self.mimic_min_angle,
            self.mimic_max_angle
        )

    # ==============================================================
    # PUBLISH FUNCTIONS
    # ==============================================================

    def publish_traction(self, velocity):

        msg = Float64MultiArray()
        msg.data = [velocity]

        self.traction_pub.publish(msg)

    # --------------------------------------------------------------

    def publish_steering(self, angle):

        msg = Float64MultiArray()
        msg.data = [angle]

        self.steering_pub.publish(msg)

    # --------------------------------------------------------------

    def publish_lift(self, effort):

        msg = Float64MultiArray()
        msg.data = [effort]

        self.lift_pub.publish(msg)

    # --------------------------------------------------------------

    def publish_mimic(self, angle):

        msg = Float64MultiArray()
        msg.data = [angle, angle]

        self.mimic_pub.publish(msg)

    # ==============================================================
    # CLAMP
    # ==============================================================

    @staticmethod
    def clamp(value, minimum, maximum):

        return max(
            minimum,
            min(maximum, value)
        )


# ==================================================================
# MAIN
# ==================================================================

def main(args=None):

    rclpy.init(args=args)

    node = BOPTController()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        # Make absolutely sure robot stops before exiting

        node.publish_traction(0.0)
        node.publish_steering(0.0)

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()