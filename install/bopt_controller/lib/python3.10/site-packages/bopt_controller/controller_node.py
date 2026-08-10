import math
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class BOPTController(Node):

    def __init__(self):
        super().__init__('bopt_controller')

        # --- Robot Parameters ---
        self.wheel_radius = 0.115
        self.max_wheel_velocity = 3.0
        self.steering_reference_length = 0.223
        self.max_steering_angle = 0.6

        # --- Mimic Parameters ---
        self.lift_min = 0.0
        self.lift_max = 0.095
        self.mimic_min_angle = 0.0
        self.mimic_max_angle = 0.78
        
        # Slew-rate limiter (rad/s) and control loop period (seconds)
        self.mimic_slew_rate = 0.40 
        self.control_dt = 0.05 

        self.commanded_mimic_angle = None
        self.joint_states_received = False

        # --- Safety & State ---
        self.cmd_vel_timeout = 0.5
        self.last_cmd_vel_time = self.get_clock().now()
        self.current_lift_position = 0.0
        self.current_shaft_angle = 0.0

        # --- Subscribers ---
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Float64, '/lift_cmd', self.lift_cmd_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        # --- Publishers ---
        self.traction_pub = self.create_publisher(Float64MultiArray, '/traction_joint_controller/commands', 10)
        self.steering_pub = self.create_publisher(Float64MultiArray, '/steering_joint_controller/commands', 10)
        # JointTrajectoryController uses a JointTrajectory topic, not Float64MultiArray
        self.lift_pub = self.create_publisher(JointTrajectory, '/lift_joint_controller/joint_trajectory', 10)
        self.mimic_pub = self.create_publisher(Float64MultiArray, '/mimic_joint_controller/commands', 10)

        # --- Timer ---
        self.control_timer = self.create_timer(self.control_dt, self.control_loop)

        # Stop robot immediately at startup
        self.publish_traction(0.0)
        self.publish_steering(0.0)
        self.get_logger().info('BOPT Controller started - ROBOT STOPPED')

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel_time = self.get_clock().now()

        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # --- Drive ---
        # FIX: Added negative sign to correct backward movement
        wheel_velocity = -(linear_velocity / self.wheel_radius)
        wheel_velocity = self.clamp(wheel_velocity, -self.max_wheel_velocity, self.max_wheel_velocity)
        
        self.publish_traction(wheel_velocity)

        # --- Steering ---
        if abs(linear_velocity) < 1e-6 or abs(angular_velocity) < 1e-6:
            steering_angle = 0.0
        else:
            steering_angle = math.atan(self.steering_reference_length * angular_velocity / linear_velocity)

        steering_angle = self.clamp(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        self.publish_steering(steering_angle)

        self.get_logger().info(
            f'DRIVE | v={linear_velocity:.3f} m/s | wheel={wheel_velocity:.3f} rad/s | steering={steering_angle:.3f} rad'
        )

    def lift_cmd_callback(self, msg):
        # msg.data is now a POSITION (m), not effort.
        # Range: 0.0 (down) to 0.095 (fully raised).
        lift_position = self.clamp(msg.data, self.lift_min, self.lift_max)

        # Compute travel duration: assume max velocity = 0.03 m/s.
        # Give enough time + 0.5 s margin so the trajectory doesn't time out.
        distance = abs(lift_position - self.current_lift_position)
        duration_s = max(1.0, distance / 0.025 + 0.5)

        self.publish_lift(lift_position, duration_s)
        self.get_logger().info(
            f'LIFT | target_pos={lift_position:.4f} m | duration={duration_s:.2f} s'
        )

    def joint_state_callback(self, msg):
        if 'front_lift_joint' in msg.name:
            index = msg.name.index('front_lift_joint')
            if index < len(msg.position):
                self.current_lift_position = msg.position[index]

        if 'front_wheel_shaft_left_joint' in msg.name:
            index = msg.name.index('front_wheel_shaft_left_joint')
            if index < len(msg.position):
                self.current_shaft_angle = msg.position[index]
        
        
        # Seed commanded_mimic_angle on the very first message
        if not self.joint_states_received:
            self.commanded_mimic_angle = self.current_shaft_angle
            self.joint_states_received = True
            self.get_logger().info(
                f'MIMIC | Seeded from actual shaft position: {self.commanded_mimic_angle:.4f} rad'
            )

    def control_loop(self):
        # 1. Safety Watchdog
        elapsed = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > self.cmd_vel_timeout:
            self.publish_traction(0.0)
            self.publish_steering(0.0)

        # 2. Mimic Control (slew-rate limiter)
        if not self.joint_states_received:
            return

        target_mimic_angle = self.calculate_mimic_angle(self.current_lift_position)
        max_step = self.mimic_slew_rate * self.control_dt
        error = target_mimic_angle - self.commanded_mimic_angle

        # Ramp commanded angle toward target
        if abs(error) <= max_step:
            self.commanded_mimic_angle = target_mimic_angle
        else:
            self.commanded_mimic_angle += math.copysign(max_step, error)

        self.commanded_mimic_angle = self.clamp(self.commanded_mimic_angle, self.mimic_min_angle, self.mimic_max_angle)
        self.publish_mimic(self.commanded_mimic_angle)

    def calculate_mimic_angle(self, lift_position):
        lift_position = self.clamp(lift_position, self.lift_min, self.lift_max)
        lift_ratio = (lift_position - self.lift_min) / (self.lift_max - self.lift_min)
        
        angle = self.mimic_min_angle + lift_ratio * (self.mimic_max_angle - self.mimic_min_angle)
        return self.clamp(angle, self.mimic_min_angle, self.mimic_max_angle)

    # --- Publish Functions ---
    def publish_traction(self, velocity):
        self.traction_pub.publish(Float64MultiArray(data=[velocity]))

    def publish_steering(self, angle):
        self.steering_pub.publish(Float64MultiArray(data=[angle]))

    def publish_lift(self, position, duration_s=2.0):
        """
        Send a JointTrajectory goal to lift_joint_controller.
        position : float  — target lift height in metres (0.0 to 0.095)
        duration_s : float — time allowed to reach the position
        """
        msg = JointTrajectory()
        msg.joint_names = ['front_lift_joint']

        point = JointTrajectoryPoint()
        point.positions = [position]
        point.velocities = [0.0]          # hold still at target
        point.time_from_start = Duration(
            sec=int(duration_s),
            nanosec=int((duration_s % 1.0) * 1e9)
        )
        msg.points = [point]
        self.lift_pub.publish(msg)

    def publish_mimic(self, angle):
        self.mimic_pub.publish(Float64MultiArray(data=[angle, angle]))

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))


def main(args=None):
    rclpy.init(args=args)
    node = BOPTController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure robot stops before exiting
        node.publish_traction(0.0)
        node.publish_steering(0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()