import argparse
import math
import json
import time
import pickle
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev
from rclpy.qos import QoSProfile, QoSReliabilityPolicy,QoSHistoryPolicy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import signal
from nav_msgs.msg import Path  # Import the Path message
from visualization_msgs.msg import Marker  # Import the Marker message


# SCALE = 60
MIN_VELOCITY = 0.1  # Define a minimum velocity limit
MAX_VELOCITY = 0.1  # Define a maximum velocity limit
SLOW_DOWN_DISTANCE = 1.0  # Distance within which the robot starts to slow down
VELOCITY_SMOOTHING_FACTOR = 0.01  # Smoothing factor for velocity adjustments


class RobotClient(Node):
    def __init__(self):
        super().__init__('robot_client')

        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('path_file', '/home/jkw/bopt_ws/src/nmpc_controller/nmpc_controller/test_path.pkl')

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.horizon = 5
        self.steering_angles = []

        self.path = []
        self.path_index = 0
        self.path_received = False
        self.goal_reached = False
        self.goal_tolerance = None
        self.path_file = None

        # Load the lookup table
        with open('/home/jkw/bopt_ws/src/nmpc_controller/nmpc_controller/mpc_lookup_table_1.425.pkl', 'rb') as f:
            self.lookup_table = pickle.load(f)

        self.velocity_publisher = self.create_publisher(Float64, '/velocity', 10)
        self.steering_angle_publisher = self.create_publisher(Float64, '/steering_angle', 10)
        self.state_publisher = self.create_publisher(String, '/state', 10)
        self.path_publisher = self.create_publisher(Path, '/visualization_path', 10)  # Path publisher
        self.target_point_publisher = self.create_publisher(Marker, '/target_point_marker', 10)  # Marker publisher for target point
        self.current_pose = None
        self.current_velocity = 0.0  # Initialize the current velocity to zero

        qos_settings = QoSProfile(depth=10)
        qos_settings.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/current_pose',
            self.current_pose_callback,
            qos_settings)

    

        self.timer = self.create_timer(0.05, self.follow_path)

        self.read_path_from_file(self.get_parameter('path_file').value)

    def set_parameters(self, path_file, goal_tolerance):
        self.goal_tolerance = goal_tolerance
        self.path_file = path_file
        self.read_path_from_file(self.path_file)
        self.get_logger().info(f'Path file set to: {self.path_file}')
        self.get_logger().info(f'Goal tolerance set to: {self.goal_tolerance}')


    def send_command(self, velocity, steering_angle):
        self.velocity_publisher.publish(Float64(data=velocity))
        self.steering_angle_publisher.publish(Float64(data=steering_angle))
        self.get_logger().info(f'Sending command - Velocity: {velocity:.2f} m/s, Steering Angle: {steering_angle:.2f} degrees')
        state_msg = String()
        state_msg.data = 'Custom'
        self.state_publisher.publish(state_msg)

    def read_path_from_file(self, file_path):
        with open(file_path, 'rb') as f:
            self.path = pickle.load(f)
        self.path_received = True
        self.publish_path_for_visualization() 

    def publish_path_for_visualization(self):
        """Convert the loaded path to a Path message and publish it."""
        if not self.path:
            self.get_logger().warn("No path loaded to publish.")
            return

        path_msg = Path()
        path_msg.header.frame_id = 'map'  # Set the frame_id to match your RViz2 configuration
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for point in self.path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = 0.0  # Assuming 2D path
            pose.pose.orientation.w = 1.0  # Assuming no rotation for visualization
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info('Path published for visualization in RViz2.')


    def transform_point(self, point, robot_x, robot_y, robot_orientation):
        cos_theta = np.cos(robot_orientation)
        sin_theta = np.sin(robot_orientation)

        dx = point[0] - robot_x
        dy = point[1] - robot_y
        transformed_x = dx * cos_theta + dy * sin_theta
        transformed_y = -dx * sin_theta + dy * cos_theta

        return [transformed_x, transformed_y]



    def find_nearest_key(self, position):
        nearest_key = None
        min_position_difference = float('inf')

        for key in self.lookup_table.keys():
            position_difference = np.sqrt((key[0] - position[0]) ** 2 + (key[1] - position[1]) ** 2)

            if position_difference < min_position_difference:
                min_position_difference = position_difference
                nearest_key = key

        return nearest_key

    def calculate_curvature_scipy(self, path):
        if len(path) < 3:
            return 0

        try:
            path = np.array(path)
            x = path[:, 0]
            y = path[:, 1]

            tck, u = splprep([x, y], s=0)
            dx, dy = splev(u, tck, der=1)
            ddx, ddy = splev(u, tck, der=2)

            curvature = np.abs(dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** 1.5
            return np.sum(curvature)
        except Exception as e:
            self.get_logger().error(f"Error in calculate_curvature_scipy: {e}")
            return 0

    def get_target_point(self, robot_x, robot_y):
        lookahead_distance = 0.2

        for i in range(self.path_index, len(self.path)):
            point = self.path[i]
            distance = math.hypot(point[0] - robot_x, point[1] - robot_y)
            if distance > lookahead_distance:
                self.path_index = i
                break

        segmented_path = self.path[self.path_index:self.path_index + 5]

        if len(segmented_path) < 3:
            self.get_logger().warn("Not enough points in segmented_path for curvature calculation")
            return self.path[-1]

        curvature = self.calculate_curvature_scipy(segmented_path)
        if curvature < 0.01:  # If curvature is low, increase the lookahead distance
            lookahead_distance += 0.3

        for i in range(self.path_index, len(self.path)):
            point = self.path[i]
            distance = math.hypot(point[0] - robot_x, point[1] - robot_y)
            if distance > lookahead_distance:
                self.path_index = i
                return point
        return self.path[-1]

    def follow_path(self):
        if self.goal_reached or not self.path_received:
            return

        state = self.current_pose
        if state is None:
            return

        robot_x = state.pose.position.x   # Convert back to pixels
        robot_y = state.pose.position.y  # Convert back to pixels
        robot_orientation = self.get_yaw_from_pose(state)
        robot_state = [robot_x, robot_y, robot_orientation]
        target_point_on_plan = self.get_target_point(robot_x, robot_y)

        ttp = self.transform_point(target_point_on_plan, robot_x, robot_y, robot_orientation)
        self.get_logger().info(f'TTP: {ttp}')

        s, v, d = self.lookup_table.get(self.find_nearest_key(ttp))
        velocity, steering_angle = (MAX_VELOCITY / 0.2) * v, math.degrees(s)

        self.steering_angles.append(steering_angle)  # Store the steering angle

        final_point = self.path[-1]
        distance_to_goal = math.hypot(final_point[0] - robot_x, final_point[1] - robot_y)
        if distance_to_goal <= self.goal_tolerance:
            self.goal_reached = True
            self.send_command(0.0, 0.0)  # Stop the robot
            return

        # Calculate the velocity based on the distance to the goal
        if distance_to_goal <= SLOW_DOWN_DISTANCE:
            if velocity < 0:
                velocity = -max(MIN_VELOCITY, (distance_to_goal / (SLOW_DOWN_DISTANCE)) * MAX_VELOCITY)
            else:
                velocity = max(MIN_VELOCITY, (distance_to_goal / (SLOW_DOWN_DISTANCE)) * MAX_VELOCITY)

        # Ensure the velocity is within safe limits based on the steering angle
        # safe_velocity = self.calculate_safe_velocity(steering_angle)
        # if velocity < 0:
        #     velocity = -min(abs(velocity), abs(safe_velocity))
        # else:
        #     velocity = min(abs(velocity), abs(safe_velocity))

        # velocity = -velocity
        # steering_angle = -steering_angle

        # smoothed_velocity = self.smooth_velocity(velocity)

        self.send_command(velocity, steering_angle)
        self.publish_target_point_marker(target_point_on_plan)

    def smooth_velocity(self, target_velocity):
        """Smoothly adjust the current velocity towards the target velocity."""
        velocity_difference = target_velocity - self.current_velocity
        smoothed_velocity = self.current_velocity + velocity_difference * VELOCITY_SMOOTHING_FACTOR
        self.current_velocity = smoothed_velocity
        return smoothed_velocity

    def publish_target_point_marker(self, target_point):
        """Publish the target point as a marker for visualization in RViz2."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'target_point'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Set the position of the marker to the target point
        marker.pose.position.x = target_point[0]
        marker.pose.position.y = target_point[1]
        marker.pose.position.z = 0.0  # Assuming 2D path
        marker.pose.orientation.w = 1.0  # No rotation

        # Set the scale of the marker (radius)
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Set the color of the marker (RGBA)
        marker.color.a = 1.0  # Opacity
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0  # Green
        marker.color.b = 0.0  # Blue

        self.target_point_publisher.publish(marker)
        self.get_logger().info(f'Target point published at ({target_point[0]}, {target_point[1]}) for visualization in RViz2.')


    def calculate_safe_velocity(self, steering_angle):
        mu = 0.3  # Coefficient of friction
        g = 9.81  # Acceleration due to gravity (m/s^2)
        L = 1.36  # Wheelbase (m)

        turn_radius = abs(L / np.tan(steering_angle + 1e-6))  # Adding a small value to avoid division by zero

        safe_velocity = 0.25 * np.sqrt(mu * g * turn_radius)
        return safe_velocity   # Scale the velocity to match the units used in your system

    def get_yaw_from_pose(self, pose):
        orientation_q = pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def stop_robot(self):
        """Send stop command to the robot and log the action."""
        self.send_command(0.0, 0.0)  # Stop the robot
        self.get_logger().info('Stopping the robot...')
        


def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='RobotClient Node')
    parser.add_argument('--path_file', type=str, required=True, help='Path to the file containing the path data')
    parser.add_argument('--goal_tolerance', type=float, default=0.1, help='Goal tolerance distance')
    parsed_args = parser.parse_args()

    client = RobotClient()
    client.set_parameters(
        path_file=parsed_args.path_file,
        goal_tolerance=parsed_args.goal_tolerance
    )

    def signal_handler(sig, frame):
        client.get_logger().info('Signal received, shutting down...')
        client.stop_robot()
        client.destroy_node()
        rclpy.shutdown()

    # Register the signal handler for SIGINT (Ctrl+C)
    signal.signal(signal.SIGINT, signal_handler)


    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        client.get_logger().info('Keyboard interrupt received, shutting down...')
        client.stop_robot()
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
