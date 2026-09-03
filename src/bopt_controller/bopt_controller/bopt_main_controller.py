#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from bopt_interfaces.msg import BoptCommandStamped


class BoptMainController(Node):

    def __init__(self):
        super().__init__('bopt_main_controller')

        # =====================================================
        # PARAMETERS
        # Keep these identical to the original BOPT controller
        # =====================================================

        self.declare_parameter('wheel_radius', 0.115)
        self.declare_parameter('max_wheel_velocity', 3.0)
        self.declare_parameter('max_steering_angle', 1.5708)

        self.declare_parameter('control_dt', 0.05)

        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.095)

        self.declare_parameter('command_timeout', 0.5)

        self.declare_parameter('steering_tolerance', 0.03)
        self.declare_parameter('steering_delay', 0.15)

        self.wheel_radius = self.get_parameter(
            'wheel_radius'
        ).value

        self.max_wheel_velocity = self.get_parameter(
            'max_wheel_velocity'
        ).value

        self.max_steering_angle = self.get_parameter(
            'max_steering_angle'
        ).value

        self.control_dt = self.get_parameter(
            'control_dt'
        ).value

        self.lift_min = self.get_parameter(
            'lift_min'
        ).value

        self.lift_max = self.get_parameter(
            'lift_max'
        ).value

        self.command_timeout = self.get_parameter(
            'command_timeout'
        ).value

        self.steering_tolerance = self.get_parameter(
            'steering_tolerance'
        ).value

        self.steering_delay = self.get_parameter(
            'steering_delay'
        ).value

        # =====================================================
        # STATE
        # =====================================================

        self.last_command_time = self.get_clock().now()

        self.current_lift_position = 0.0
        self.current_steering_angle = 0.0
        self.current_wheel_velocity = 0.0

        self.target_steering_angle = 0.0
        self.target_wheel_velocity = 0.0
        self.target_lift_position = 0.0

        self.steering_reached_time = None

        self.is_stopped = True

        # =====================================================
        # INPUT
        # =====================================================

        self.command_sub = self.create_subscription(
            BoptCommandStamped,
            'bopt/relay_cmd',
            self.command_callback,
            10
        )

        # =====================================================
        # ACTUAL JOINT STATE
        # =====================================================

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # =====================================================
        # GAZEBO OUTPUTS
        # =====================================================

        self.traction_pub = self.create_publisher(
            Float64MultiArray,
            'traction_joint_controller/commands',
            10
        )

        self.steering_pub = self.create_publisher(
            Float64MultiArray,
            'steering_joint_controller/commands',
            10
        )

        self.lift_pub = self.create_publisher(
            JointTrajectory,
            'lift_joint_controller/joint_trajectory',
            10
        )

        # =====================================================
        # SAFETY TIMER
        # =====================================================

        self.control_timer = self.create_timer(
            self.control_dt,
            self.control_loop
        )

        # Start safely stopped
        self.publish_traction(0.0)
        self.publish_steering(0.0)

        self.get_logger().info(
            'BOPT MAIN CONTROLLER started'
        )

        self.get_logger().info(
            'Input:  bopt/relay_cmd'
        )

        self.get_logger().info(
            'Output: Gazebo joint controllers'
        )

    # =========================================================
    # BOPT COMMAND
    # =========================================================

    def command_callback(self, msg):

        self.last_command_time = self.get_clock().now()

        velocity = msg.traction_velocity
        steering_angle = msg.steering_angle
        lift_height = msg.lift_height

        # -----------------------------------------------------
        # Validate command
        # -----------------------------------------------------

        if not self.valid_number(velocity):
            self.get_logger().warn(
                'Invalid traction velocity'
            )
            return

        if not self.valid_number(steering_angle):
            self.get_logger().warn(
                'Invalid steering angle'
            )
            return

        if not self.valid_number(lift_height):
            self.get_logger().warn(
                'Invalid lift height'
            )
            return

        # -----------------------------------------------------
        # Steering limit
        # -----------------------------------------------------

        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        # -----------------------------------------------------
        # IMPORTANT:
        #
        # This is NOT cmd_vel → steering conversion.
        #
        # BOPT KEY / NMPC already provides steering_angle.
        #
        # We only convert vehicle velocity [m/s]
        # into wheel angular velocity [rad/s].
        # -----------------------------------------------------

        wheel_velocity = (
            velocity / self.wheel_radius
        )

        # -----------------------------------------------------
        # Wheel velocity limit
        # -----------------------------------------------------

        wheel_velocity = self.clamp(
            wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        # -----------------------------------------------------
        # Compare requested steering with ACTUAL steering
        #
        # Preserve original BOPT safety behavior.
        # -----------------------------------------------------

        steering_error = abs(
            steering_angle -
            self.current_steering_angle
        )

        self.current_wheel_velocity = wheel_velocity
        self.target_wheel_velocity = wheel_velocity
        self.target_steering_angle = steering_angle

        # -----------------------------------------------------
        # Steering command
        # -----------------------------------------------------

        self.publish_steering(
            steering_angle
        )

        now = self.get_clock().now()

        # -----------------------------------------------------
        # Steering settling safety
        # -----------------------------------------------------

        if steering_error > self.steering_tolerance:

            self.steering_reached_time = None

            smooth_wheel_velocity = 0.0

        else:

            if self.steering_reached_time is None:

                self.steering_reached_time = now

            settled_for = (
                now -
                self.steering_reached_time
            ).nanoseconds / 1e9

            if settled_for >= self.steering_delay:

                smooth_wheel_velocity = wheel_velocity

            else:

                smooth_wheel_velocity = 0.0

        # -----------------------------------------------------
        # Traction command
        # -----------------------------------------------------

        self.publish_traction(
            smooth_wheel_velocity
        )

        self.is_stopped = (
            abs(smooth_wheel_velocity) < 1e-4
        )

        # -----------------------------------------------------
        # Lift
        # -----------------------------------------------------

        lift_position = self.clamp(
            lift_height,
            self.lift_min,
            self.lift_max
        )

        # Only issue lift trajectory when target changes
        if abs(
            lift_position -
            self.target_lift_position
        ) > 1e-4:

            distance = abs(
                lift_position -
                self.current_lift_position
            )

            duration_s = max(
                1.0,
                distance / 0.025 + 0.5
            )

            self.publish_lift(
                lift_position,
                duration_s
            )

            self.target_lift_position = lift_position

        self.get_logger().debug(
            f'BOPT MAIN | '
            f'velocity={velocity:.3f} m/s | '
            f'wheel={wheel_velocity:.3f} rad/s | '
            f'steering={steering_angle:.3f} rad | '
            f'lift={lift_position:.4f} m'
        )

    # =========================================================
    # JOINT STATE
    # =========================================================

    def joint_state_callback(self, msg):

        if 'drive_wheel_Ass_joint' in msg.name:

            index = msg.name.index(
                'drive_wheel_Ass_joint'
            )

            if index < len(msg.position):

                self.current_steering_angle = (
                    msg.position[index]
                )

        if 'front_lift_joint' in msg.name:

            index = msg.name.index(
                'front_lift_joint'
            )

            if index < len(msg.position):

                self.current_lift_position = (
                    msg.position[index]
                )

    # =========================================================
    # WATCHDOG
    # =========================================================

    def control_loop(self):

        elapsed = (
            self.get_clock().now()
            -
            self.last_command_time
        ).nanoseconds / 1e9

        if (
            elapsed > self.command_timeout
            and not self.is_stopped
        ):

            self.publish_traction(0.0)

            self.current_wheel_velocity = 0.0

            self.is_stopped = True

            self.get_logger().debug(
                'BOPT command timeout: traction stopped'
            )

    # =========================================================
    # TRACTION OUTPUT
    # =========================================================

    def publish_traction(self, velocity):

        msg = Float64MultiArray()

        msg.data = [
            float(velocity)
        ]

        self.traction_pub.publish(msg)

    # =========================================================
    # STEERING OUTPUT
    # =========================================================

    def publish_steering(self, angle):

        msg = Float64MultiArray()

        msg.data = [
            float(angle)
        ]

        self.steering_pub.publish(msg)

    # =========================================================
    # LIFT OUTPUT
    # =========================================================

    def publish_lift(
        self,
        position,
        duration_s=2.0
    ):

        msg = JointTrajectory()

        msg.joint_names = [
            'front_lift_joint'
        ]

        point = JointTrajectoryPoint()

        point.positions = [
            float(position)
        ]

        point.velocities = [
            0.0
        ]

        sec = int(duration_s)

        nanosec = int(
            (duration_s - sec) * 1e9
        )

        point.time_from_start = Duration(
            sec=sec,
            nanosec=nanosec
        )

        msg.points = [
            point
        ]

        self.lift_pub.publish(msg)

    # =========================================================
    # HELPERS
    # =========================================================

    @staticmethod
    def valid_number(value):

        return not (
            math.isnan(value)
            or math.isinf(value)
        )

    @staticmethod
    def clamp(
        value,
        minimum,
        maximum
    ):

        return max(
            minimum,
            min(maximum, value)
        )


def main(args=None):

    rclpy.init(args=args)

    node = BoptMainController()

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

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()