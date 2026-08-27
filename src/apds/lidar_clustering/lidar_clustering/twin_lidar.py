import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import List, Tuple, Optional
import numpy as np
import time


class TwinLidarNode(Node):
    def __init__(self):
        super().__init__('twin_lidar_node')

        # Subscribing to topics
        self.subscription_1 = self.create_subscription(String, '/map1', self.callback_1, 10)
        self.subscription_2 = self.create_subscription(String, '/map2', self.callback_2, 10)

        # Publisher for results
        self.result_publisher = self.create_publisher(String, 'pallet_detection_results', 10)

        # Buffers for accumulating data
        self.lidar1_buffer = None
        self.lidar2_buffer = None

        # Start times for accumulation
        self.lidar1_start_time = None
        self.lidar2_start_time = None

        # Flags to track if processing is ready
        self.lidar1_ready = False
        self.lidar2_ready = False

        # Flag to stop further processing
        self.processing_done = False

        self.get_logger().info("Twin Lidar Node Initialized")

    def callback_1(self, msg):
        """Callback for the first LiDAR topic."""
        if self.processing_done:
            return  # Stop further processing

        cluster_data = self.parse_cluster_data(msg.data)

        if self.lidar1_start_time is None:
            self.lidar1_start_time = time.time()  # Start the timer

        # Aggregate data
        if self.lidar1_buffer is None:
            self.lidar1_buffer = [[] for _ in range(len(cluster_data))]
        self.aggregate_clusters(self.lidar1_buffer, cluster_data)

        # Check if 3 seconds have passed
        if time.time() - self.lidar1_start_time >= 3.0:
            self.lidar1_ready = True
            self.lidar1_data = self.calculate_midpoint_clusters(self.lidar1_buffer)
            self.lidar1_buffer = None  # Clear the buffer
            self.lidar1_start_time = None  # Reset the timer
            #self.get_logger().info(f"Averaged Lidar1 Data: {self.lidar1_data}")
            self.check_and_process()

    def callback_2(self, msg):
        """Callback for the second LiDAR topic."""
        if self.processing_done:
            return  # Stop further processing

        cluster_data = self.parse_cluster_data(msg.data)

        if self.lidar2_start_time is None:
            self.lidar2_start_time = time.time()  # Start the timer

        # Aggregate data
        if self.lidar2_buffer is None:
            self.lidar2_buffer = [[] for _ in range(len(cluster_data))]
        self.aggregate_clusters(self.lidar2_buffer, cluster_data)

        # Check if 3 seconds have passed
        if time.time() - self.lidar2_start_time >= 3.0:
            self.lidar2_ready = True
            self.lidar2_data = self.calculate_midpoint_clusters(self.lidar2_buffer)
            self.lidar2_buffer = None  # Clear the buffer
            self.lidar2_start_time = None  # Reset the timer
            #self.get_logger().info(f"Averaged Lidar2 Data: {self.lidar2_data}")
            self.check_and_process()

    def parse_cluster_data(self, data: str) -> List[Optional[Tuple[float, float]]]:
        """Parse cluster data from the incoming message."""
        clusters = []
        lines = data.split('\n')
        for line in lines:
            line = line.strip()
            if "No Cluster" in line:
                clusters.append(None)
            elif '(' in line and ')' in line:
                try:
                    coords = line.split(':')[-1].strip().strip('()')
                    x, y = map(float, coords.split(','))
                    clusters.append((x, y))
                except ValueError as e:
                    self.get_logger().error(f"Failed to parse line: {line} with error {e}")
                    clusters.append(None)
        return clusters

    def aggregate_clusters(self, buffer: List[List[Optional[Tuple[float, float]]]], data: List[Optional[Tuple[float, float]]]):
        """Aggregate data into the buffer."""
        for i, cluster in enumerate(data):
            if cluster is not None:
                buffer[i].append(cluster)  # Append only valid (non-None) clusters

    def calculate_midpoint_clusters(self, buffer: List[List[Optional[Tuple[float, float]]]]) -> List[Optional[Tuple[float, float]]]:
        """Calculate the midpoint of each cluster."""
        midpoint_clusters = []
        for cluster_points in buffer:
            if len(cluster_points) > 0:  # Ensure there are valid points
                cluster_array = np.array(cluster_points)
                mean_x = np.mean(cluster_array[:, 0])
                mean_y = np.mean(cluster_array[:, 1])
                midpoint_clusters.append((round(mean_x, 2), round(mean_y, 2)))  # Return values up to 2 decimal points
            else:
                midpoint_clusters.append(None)  # No valid data for this cluster
        return midpoint_clusters

    def check_and_process(self):
        """Check if data from both LiDARs is available and process it."""
        if self.lidar1_ready and self.lidar2_ready:
            self.lidar1_ready = False  # Reset the flags
            self.lidar2_ready = False
            self.process_combined_data()

    def process_combined_data(self):
        """Process combined data from both LiDARs."""
        combined_data = []

        for c1, c2 in zip(self.lidar1_data, self.lidar2_data):
            if c1 is not None and c2 is not None:
                combined_data.append((round((c1[0] + c2[0]) / 2, 2), round((c1[1] + c2[1]) / 2, 2)))
            elif c1 is not None:
                combined_data.append(c1)
            elif c2 is not None:
                combined_data.append(c2)
            else:
                combined_data.append(None)

        #self.get_logger().info(f"Combined Data: {combined_data}")

        # Perform additional calculations and publish results
        self.publish_results(combined_data)

        # Stop further processing
        self.processing_done = True
        self.get_logger().info("Processing complete. Stopping further operations.")

    def publish_results(self, combined_data):
        """Publish pallet presence, middle offset, and angle offset."""
        # Step 1: Pallet presence
        clusters_detected = sum(1 for cluster in combined_data if cluster is not None)
        pallet_present = clusters_detected >= 6

        # Step 2: Middle offset
        ideal_position = (0.0, 0.3)  # Ideal coordinates for the 2nd cluster
        if combined_data[1] is not None:
            observed_position = combined_data[1]
            dx = observed_position[0] - ideal_position[0]
            dy = observed_position[1] - ideal_position[1]
            middle_offset = f"dx={dx:.2f}, dy={dy:.2f}"
        else:
            middle_offset = "No Data"

        # Step 3: Angle offset
        if combined_data[0] is not None and combined_data[2] is not None:
            x1, y1 = combined_data[0]
            x3, y3 = combined_data[2]
            if x3 - x1 != 0:
                slope = (y3 - y1) / (x3 - x1)
                angle_rad = np.arctan(slope)
                angle_deg = np.degrees(angle_rad)
                angle_offset = f"{angle_deg:.2f} degrees"
            else:
                angle_offset = "90.00 degrees (vertical line)"
        else:
            angle_offset = "No Data"

        # Publish results
        result_message = f"Pallet Present: {'Yes' if pallet_present else 'No'}, Middle Offset: {middle_offset}, Angle Offset: {angle_offset}"
        self.result_publisher.publish(String(data=result_message))
        self.get_logger().info(f"Published Results: {result_message}")


def main(args=None):
    rclpy.init(args=args)
    node = TwinLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
