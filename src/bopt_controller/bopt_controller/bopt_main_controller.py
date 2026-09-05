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
        # =====================================================

        self.declare_parameter('wheel_radius', 0.115)
        self.declare_parameter('max_wheel_velocity', 3.0)
        self.declare_parameter('max_steering_angle', 1.5708)

        self.declare_parameter('control_dt', 0.05)

        self.declare_parameter('lift_min', 0.0)
        self.declare_parameter('lift_max', 0.095)

        self.declare_parameter('command_timeout', 0.5)

        # Steering must be within this error before traction starts.
        # 0.03 rad ~= 1.7 degrees
        self.declare_parameter('steering_tolerance', 0.03)

        # Additional settling time after steering enters tolerance.
        self.declare_parameter('steering_delay', 0.05)

        # -----------------------------------------------------
        # ACCELERATION / DECELERATION
        # wheel angular velocity [rad/s^2]
        # -----------------------------------------------------

        self.declare_parameter(
            'wheel_acceleration',
            8.0
        )

        self.declare_parameter(
            'wheel_deceleration',
            12.0
        )

        # -----------------------------------------------------
        # Steering must move first when error exceeds this.
        # This is intentionally larger than steering_tolerance.
        # -----------------------------------------------------

        self.declare_parameter(
            'steering_start_threshold',
            0.05
        )

        # -----------------------------------------------------
        # Reverse direction change:
        #
        # If current wheel velocity is positive and requested
        # velocity is negative (or vice versa), stop first.
        # -----------------------------------------------------

        self.declare_parameter(
            'reverse_stop_threshold',
            0.02
        )

        # =====================================================
        # READ PARAMETERS
        # =====================================================

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

        self.wheel_acceleration = self.get_parameter(
            'wheel_acceleration'
        ).value

        self.wheel_deceleration = self.get_parameter(
            'wheel_deceleration'
        ).value

        self.steering_start_threshold = self.get_parameter(
            'steering_start_threshold'
        ).value

        self.reverse_stop_threshold = self.get_parameter(
            'reverse_stop_threshold'
        ).value

        # =====================================================
        # STATE
        # =====================================================

        self.last_command_time = self.get_clock().now()

        self.current_lift_position = 0.0

        # ACTUAL steering position from joint_states
        self.current_steering_angle = 0.0

        # ACTUAL/commanded wheel velocity state
        self.current_wheel_velocity = 0.0

        # Targets
        self.target_steering_angle = 0.0
        self.target_wheel_velocity = 0.0
        self.target_lift_position = 0.0

        # Time when steering first entered tolerance
        self.steering_reached_time = None

        # =====================================================
        # CONTROLLER STATE
        # =====================================================

        self.steering_aligned = True

        self.reverse_wait = False

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
        # JOINT STATES
        # =====================================================

        self.joint_state_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # =====================================================
        # OUTPUTS
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
        # CONTROL TIMER
        # =====================================================

        self.control_timer = self.create_timer(
            self.control_dt,
            self.control_loop
        )

        # =====================================================
        # SAFE START
        # =====================================================

        self.publish_traction(0.0)
        self.publish_steering(0.0)

        self.get_logger().info(
            'BOPT MAIN CONTROLLER started'
        )

        self.get_logger().info(
            'Steering-first interlock ENABLED'
        )

        self.get_logger().info(
            f'Wheel acceleration: '
            f'{self.wheel_acceleration:.2f} rad/s^2'
        )

        self.get_logger().info(
            f'Wheel deceleration: '
            f'{self.wheel_deceleration:.2f} rad/s^2'
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
        # VALIDATION
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
        # LIMIT STEERING
        # -----------------------------------------------------

        steering_angle = self.clamp(
            steering_angle,
            -self.max_steering_angle,
            self.max_steering_angle
        )

        # -----------------------------------------------------
        # VEHICLE VELOCITY -> WHEEL VELOCITY
        # -----------------------------------------------------

        wheel_velocity = (
            velocity / self.wheel_radius
        )

        wheel_velocity = self.clamp(
            wheel_velocity,
            -self.max_wheel_velocity,
            self.max_wheel_velocity
        )

        # -----------------------------------------------------
        # SAVE TARGETS
        # -----------------------------------------------------

        previous_target_steering = (
            self.target_steering_angle
        )

        previous_target_wheel = (
            self.target_wheel_velocity
        )

        self.target_steering_angle = steering_angle
        self.target_wheel_velocity = wheel_velocity

        # =====================================================
        # REVERSE DIRECTION DETECTION
        # =====================================================

        direction_change = (
            abs(previous_target_wheel) >
            self.reverse_stop_threshold
            and
            abs(wheel_velocity) >
            self.reverse_stop_threshold
            and
            (
                math.copysign(
                    1.0,
                    previous_target_wheel
                )
                !=
                math.copysign(
                    1.0,
                    wheel_velocity
                )
            )
        )

        if direction_change:

            self.reverse_wait = True

            self.get_logger().info(
                'Direction change detected: '
                'stopping before reversing'
            )

        # =====================================================
        # STEERING TARGET CHANGED
        # =====================================================

        steering_target_changed = (
            abs(
                steering_angle -
                previous_target_steering
            )
            >
            self.steering_tolerance
        )

        if steering_target_changed:

            self.steering_reached_time = None

        # =====================================================
        # ALWAYS SEND STEERING TARGET
        # =====================================================

        self.publish_steering(
            steering_angle
        )

        # =====================================================
        # LIFT
        # =====================================================

        lift_position = self.clamp(
            lift_height,
            self.lift_min,
            self.lift_max
        )

        if abs(
            lift_position -
            self.target_lift_position
        ) > 1e-4:

            distance = abs(
                lift_position -
                self.current_lift_position
            )

            duration_s = max(
                0.5,
                distance / 0.05
            )

            self.publish_lift(
                lift_position,
                duration_s
            )

            self.target_lift_position = lift_position

        # =====================================================
        # DEBUG
        # =====================================================

        steering_error = abs(
            steering_angle -
            self.current_steering_angle
        )

        self.get_logger().debug(
            f'CMD | '
            f'v={velocity:.3f} m/s | '
            f'wheel_target={wheel_velocity:.3f} | '
            f'steer_target={steering_angle:.3f} | '
            f'steer_actual={self.current_steering_angle:.3f} | '
            f'steer_error={steering_error:.3f}'
        )

    # =========================================================
    # JOINT STATE
    # =========================================================

    def joint_state_callback(self, msg):

        # -----------------------------------------------------
        # IMPORTANT:
        #
        # drive_wheel_Ass_joint is being used as the steering
        # assembly position based on your current robot model.
        # -----------------------------------------------------

        if 'drive_wheel_Ass_joint' in msg.name:

            index = msg.name.index(
                'drive_wheel_Ass_joint'
            )

            if index < len(msg.position):

                self.current_steering_angle = (
                    msg.position[index]
                )

        # -----------------------------------------------------
        # LIFT
        # -----------------------------------------------------

        if 'front_lift_joint' in msg.name:

            index = msg.name.index(
                'front_lift_joint'
            )

            if index < len(msg.position):

                self.current_lift_position = (
                    msg.position[index]
                )

    # =========================================================
    # MAIN CONTROL LOOP
    # =========================================================

    def control_loop(self):

        now = self.get_clock().now()

        # =====================================================
        # COMMAND WATCHDOG
        # =====================================================

        elapsed = (
            now -
            self.last_command_time
        ).nanoseconds / 1e9

        if elapsed > self.command_timeout:

            if not self.is_stopped:

                self.get_logger().warn(
                    'Command timeout: stopping traction'
                )

            self.target_wheel_velocity = 0.0

            self.reverse_wait = False

            self.is_stopped = True

        # =====================================================
        # STEERING ERROR
        # =====================================================

        steering_error = abs(
            self.target_steering_angle -
            self.current_steering_angle
        )

        # =====================================================
        # STEERING ALIGNMENT STATE
        # =====================================================

        if steering_error > self.steering_start_threshold:

            self.steering_aligned = False

            self.steering_reached_time = None

        else:

            if not self.steering_aligned:

                if self.steering_reached_time is None:

                    self.steering_reached_time = now

                settled_time = (
                    now -
                    self.steering_reached_time
                ).nanoseconds / 1e9

                if settled_time >= self.steering_delay:

                    self.steering_aligned = True

                    self.get_logger().debug(
                        'Steering aligned: traction released'
                    )

        # =====================================================
        # STEERING SAFETY INTERLOCK
        # =====================================================

        if not self.steering_aligned:

            # ABSOLUTELY NO TRACTION WHILE STEERING
            # IS FAR FROM TARGET.

            self.current_wheel_velocity = (
                self.ramp_velocity(
                    self.current_wheel_velocity,
                    0.0,
                    self.wheel_deceleration
                )
            )

            self.publish_traction(
                self.current_wheel_velocity
            )

            self.is_stopped = (
                abs(
                    self.current_wheel_velocity
                ) < 1e-4
            )

            return

        # =====================================================
        # REVERSE SAFETY
        # =====================================================

        if self.reverse_wait:

            self.current_wheel_velocity = (
                self.ramp_velocity(
                    self.current_wheel_velocity,
                    0.0,
                    self.wheel_deceleration
                )
            )

            self.publish_traction(
                self.current_wheel_velocity
            )

            if abs(
                self.current_wheel_velocity
            ) <= self.reverse_stop_threshold:

                self.current_wheel_velocity = 0.0

                self.publish_traction(0.0)

                self.reverse_wait = False

                self.get_logger().debug(
                    'Vehicle stopped: reverse command released'
                )

            return

        # =====================================================
        # NORMAL VELOCITY RAMP
        # =====================================================

        target = self.target_wheel_velocity

        # -----------------------------------------------------
        # Accelerating
        # -----------------------------------------------------

        if abs(target) > abs(
            self.current_wheel_velocity
        ):

            self.current_wheel_velocity = (
                self.ramp_velocity(
                    self.current_wheel_velocity,
                    target,
                    self.wheel_acceleration
                )
            )

        # -----------------------------------------------------
        # Decelerating
        # -----------------------------------------------------

        else:

            self.current_wheel_velocity = (
                self.ramp_velocity(
                    self.current_wheel_velocity,
                    target,
                    self.wheel_deceleration
                )
            )

        # =====================================================
        # OUTPUT TRACTION
        # =====================================================

        self.publish_traction(
            self.current_wheel_velocity
        )

        self.is_stopped = (
            abs(
                self.current_wheel_velocity
            ) < 1e-4
        )

        # =====================================================
        # KEEP STEERING COMMAND ALIVE
        # =====================================================

        self.publish_steering(
            self.target_steering_angle
        )

    # =========================================================
    # VELOCITY RAMP
    # =========================================================

    def ramp_velocity(
        self,
        current,
        target,
        rate
    ):

        max_change = (
            rate *
            self.control_dt
        )

        difference = (
            target -
            current
        )

        if abs(difference) <= max_change:

            return target

        if difference > 0:

            return current + max_change

        return current - max_change

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
            node.publish_steering(0.0)
        except Exception:
            pass

        node.destroy_node()

        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()