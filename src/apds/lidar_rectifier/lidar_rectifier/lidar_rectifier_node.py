import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class LidarRectifierNode(Node):
    def __init__(self):
        super().__init__('lidar_rectifier_node')

        # Fixed length of the rectified message, e.g., 310
        self.fixed_length = 310
        self.valid_values = np.zeros(self.fixed_length)  # Initialize with zeros
        self.initialized = False  # To check if valid_values have been initialized

        # Publisher for rectified scan data
        self.rectified_publisher = self.create_publisher(LaserScan, '/scan_rectified', 10)

        # Subscriber to raw LiDAR data
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10)

        self.get_logger().info("Lidar Rectifier Node Initialized")

    def lidar_callback(self, msg):
        """Callback function to process incoming LiDAR data."""
        ranges = np.array(msg.ranges)
        print(msg)
        print(len(ranges))

        # Ensure the incoming message length is within the fixed length
        ranges = ranges[:self.fixed_length]  # Trim the incoming data to the fixed length

        # Initialize valid_values with the first valid message
        if not self.initialized:
            self.valid_values[:len(ranges)] = ranges  # Initialize with the first message's valid values
            self.valid_values[np.isnan(self.valid_values)] = np.nan  # Ensure NaN values remain
            self.initialized = True
            print("Valid values initialized with the first LiDAR message.")
            return  # Skip processing for the first message as we just initialized

        # Rectification process: Update NaN/zero values using previous valid values
        for i in range(len(ranges)):
            if np.isnan(ranges[i]) or ranges[i] <= 0.0:
                # If current range is invalid (NaN or <= 0), use previous valid value
                if not np.isnan(self.valid_values[i]) and self.valid_values[i] > 0.0:
                    ranges[i] = self.valid_values[i]  # Replace with valid value
                #     print(f"Replaced NaN/invalid value at index {i} with {self.valid_values[i]}")
                # else:
                #     print(f"Leaving value at index {i} as NaN or invalid.")
            else:
                # Update the valid_values with current valid data
                self.valid_values[i] = ranges[i]
                #print(f"Updated valid value at index {i} to {self.valid_values[i]}")

        # Debug: Check if all values are valid (non-NaN and non-zero)
        if not self.is_message_valid(ranges):
            self.get_logger().warn("Rectified message is incomplete. Waiting for more data.")
            print("Rectified message is incomplete. Skipping publication.")
            return

        # Debug: Creating rectified message
        self.get_logger().debug("Creating rectified LaserScan message.")
        rectified_msg = LaserScan()
        rectified_msg.header = msg.header  # Copy header from the original message
        rectified_msg.angle_min = msg.angle_min
        rectified_msg.angle_max = msg.angle_max
        rectified_msg.angle_increment = msg.angle_increment
        rectified_msg.time_increment = msg.time_increment
        rectified_msg.scan_time = msg.scan_time
        rectified_msg.range_min = msg.range_min
        rectified_msg.range_max = msg.range_max
        rectified_msg.ranges = ranges.tolist()

        # Debug: Check rectified ranges
        #print(f"Rectified ranges: {rectified_msg.ranges}")

        # Publish the rectified message
        self.rectified_publisher.publish(rectified_msg)
        self.get_logger().info("Published rectified LiDAR data.")
        print("Rectified LiDAR data published successfully.")

    def is_message_valid(self, ranges):
        """
        Validate the ranges array to ensure all values are non-NaN and non-zero.
        """
        return not np.any(np.isnan(ranges)) and np.all(ranges > 0.0)

def main(args=None):
    rclpy.init(args=args)
    node = LidarRectifierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
