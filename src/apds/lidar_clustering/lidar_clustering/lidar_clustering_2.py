import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np
from sensor_msgs.msg import LaserScan
import cv2
from sklearn.cluster import KMeans
from typing import List, Tuple
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3  # Correct import for scale
from std_msgs.msg import String

def calculate_rois(l: float, b: float) -> List[Tuple[float, float, float, float]]:
    """Calculate Regions of Interest (ROIs)."""
    b2 = b / 2  # Half of the breadth
    l2 = l / 2  # Half of the length
    lidar_offset = 0.15  # Approximate distance between LiDAR and pallet
    roi_width = 0.3     # Width of each ROI (x range)
    roi_height = 0.3    # Height of each ROI (y range)

    rois = [
        # Bottom row (ROI 1, 2, 3)
        (-b2 + 0.05 - roi_width / 2, -b2 + 0.05 + roi_width / 2, lidar_offset, lidar_offset + roi_height),
        (-roi_width / 2, roi_width / 2, lidar_offset, lidar_offset + roi_height),
        (b2 - 0.05 - roi_width / 2, b2 - 0.05 + roi_width / 2, lidar_offset, lidar_offset + roi_height),
        # Middle row (ROI 4, 5, 6)
        (-b2 + 0.05 - roi_width / 2, -b2 + 0.05 + roi_width / 2, lidar_offset - 0.05 + l2, lidar_offset - 0.05 + l2 + roi_height),
        (-roi_width / 2, roi_width / 2, lidar_offset - 0.05 + l2, lidar_offset - 0.05 + l2 + roi_height),
        (b2 - 0.05 - roi_width / 2, b2 - 0.05 + roi_width / 2, lidar_offset - 0.05 + l2, lidar_offset - 0.05 + l2 + roi_height),
        # Top row (ROI 7, 8, 9)
        (-b2 + 0.05 - roi_width / 2, -b2 + 0.05 + roi_width / 2, lidar_offset / 2 + l, lidar_offset / 2 + l + roi_height),
        (-roi_width / 2, roi_width / 2, lidar_offset / 2 + l, lidar_offset / 2 + l + roi_height),
        (b2 - 0.05 - roi_width / 2, b2 - 0.05 + roi_width / 2, lidar_offset / 2 + l, lidar_offset / 2 + l + roi_height),
    ]

    return rois


def apply_transformation(points, translation, rotation):
    """Apply a static translation and rotation to points."""
    tx, ty = translation
    yaw = rotation[2]  # Assuming 2D rotation (yaw)

    rotation_matrix = np.array([
        [np.cos(yaw), -np.sin(yaw)],
        [np.sin(yaw), np.cos(yaw)]
    ])

    transformed_points = np.dot(points, rotation_matrix.T) + np.array([tx, ty])
    return transformed_points


class LidarClusteringNode(Node):
    def __init__(self):
        super().__init__('lidar_clustering_node')
        self.roi_pub = self.create_publisher(Marker, 'roi_marker', 10)
        self.cluster_pub = self.create_publisher(String, 'map2', 10)  # New publisher for clusters
        self.subscription = self.create_subscription(
            LaserScan,
            '/Lidar_LFT',
            self.lidar_callback,
            10
        )
        self.translation = (0.18, 0)  # base_laser_2 to base_link translation(left)
        self.rotation = (0, 0, 0)    # base_laser_2 to base_link rotation
        #self.get_logger().info("Lidar Clustering Node Initialized")

    def lidar_callback(self, msg):
        rois = calculate_rois(1.22, 1.22)
        points = self.calculate_points(msg)
        self.roi_cluster_mapping = {i + 1: "No Data" for i in range(len(rois))}
        transformed_points = apply_transformation(points, self.translation, self.rotation)
    
        self.pro_lidar_callback(transformed_points, rois, "LIDAR_2")

    def calculate_points(self, msg):
        """Convert LaserScan to valid (x, y) points."""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        x_points = ranges[valid_mask] * np.cos(angles[valid_mask])
        y_points = ranges[valid_mask] * np.sin(angles[valid_mask])
        points = np.stack((x_points, y_points), axis=-1)
        #self.get_logger().info(f"Total valid points: {len(points)}")
        return points

    def pro_lidar_callback(self, points, rois, window_name):
        """Process and visualize clusters."""
        points_all_rois = {f'roi_{i+1}': self.filter_points_in_roi(points, roi) for i, roi in enumerate(rois)}

        cluster_centers_all_rois = self.perform_clustering(points_all_rois)
        # print(cluster_centers_all_rois, "=================")
        self.visualize_clusters(points, rois, cluster_centers_all_rois, window_name)

        for i, roi in enumerate(rois):
            self.publish_roi_marker(roi, i + 1)
        self.publish_cluster_centers(cluster_centers_all_rois)
    

    def publish_cluster_centers(self, cluster_centers_all_rois):
        """Publish cluster centers to a new topic."""
        message_data = []
        for roi_index, cluster_center in cluster_centers_all_rois:
            if cluster_center is not None:
                message_data.append(f"ROI {roi_index}: ({cluster_center[0]:.2f}, {cluster_center[1]:.2f})")
            else:
                message_data.append(f"ROI {roi_index}: No Cluster")
        # Convert to a String message
        msg = String()
        msg.data = "\n".join(message_data)
        self.cluster_pub.publish(msg)  # Publish the message
        #self.get_logger().info("Published cluster centers to 'map2' topic.")
    
    def perform_clustering(self, points_all_rois):

        """Perform KMeans clustering for points in each ROI."""
        cluster_centers = []
        for roi_index, (roi_name, roi_points) in enumerate(points_all_rois.items(), start=1):
            if roi_points.shape[0] > 3:
                kmeans = KMeans(n_clusters=1, random_state=42)
                kmeans.fit(roi_points)
                cluster_centers.append((roi_index, kmeans.cluster_centers_[0]))
            else:
                cluster_centers.append((roi_index, None))  # No cluster found
        return cluster_centers
    
    def filter_points_in_roi(self, points, roi):
        """Filter points within a specific ROI."""
        xmin, xmax, ymin, ymax = roi
        mask = (xmin <= points[:, 0]) & (points[:, 0] <= xmax) & (ymin <= points[:, 1]) & (points[:, 1] <= ymax)
        return points[mask]
    def create_header(self, frame_id: str):
        """Create a standard header for ROS messages."""
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = frame_id
        header.stamp = self.get_clock().now().to_msg()
        return header

    def publish_roi_marker(self, roi, marker_id):
        """Publish ROI rectangle as a marker in RViz."""
        rectangle_points = self.calculate_roi_rectangle(roi)

        marker = Marker()
        marker.header.frame_id = "base_link"  # Use appropriate frame for RViz
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.05, y=0.0, z=0.0)  # Correctly specify the scale
        marker.color = ColorRGBA(a=1.0, r=0.0, g=0.0, b=1.0)  # Blue with full opacity

        # Convert rectangle points to geometry_msgs/Point
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in rectangle_points]
        self.roi_pub.publish(marker)

    def calculate_roi_rectangle(self, roi):
        """Calculate rectangle points for a given ROI."""
        xmin, xmax, ymin, ymax = roi
        return [
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax),
            (xmin, ymin)  # Close the rectangle
        ]

    def visualize_clusters(self, points, rois, cluster_centers, window_name):
        """Visualize clusters and ROIs on an OpenCV window."""
        img = np.zeros((1000, 1000, 3), dtype=np.uint8)
        scale, offset = 300, 500

        # Draw ROIs
        for roi in rois:
            xmin, xmax, ymin, ymax = roi
            cv2.rectangle(
                img,
                (int(xmin * scale + offset), int(ymin * scale + offset)),
                (int(xmax * scale + offset), int(ymax * scale + offset)),
                (255, 255, 255), 1
            )

        # Plot points
        for point in points:
            x, y = int(point[0] * scale + offset), int(point[1] * scale + offset)
            cv2.circle(img, (x, y), 1, (0, 255, 0), -1)

        # Map clusters to ROIs and display on screen
        for i, (roi_index, cluster_center) in enumerate(cluster_centers):
            if cluster_center is not None:
                # Update the ROI cluster mapping
                self.roi_cluster_mapping[roi_index] = (cluster_center[0], cluster_center[1])
                
                # Draw cluster center on OpenCV screen
                x_center = int(cluster_center[0] * scale + offset)
                y_center = int(cluster_center[1] * scale + offset)
                cv2.circle(img, (x_center, y_center), 8, (0, 0, 255), -1)
                cluster_text = f"Cluster {roi_index}: ({cluster_center[0]:.2f}, {cluster_center[1]:.2f})"
                cv2.putText(img, cluster_text, (10, 20 + i * 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            else:
                # No cluster found for this ROI
                self.roi_cluster_mapping[roi_index] = "No Data"

        cv2.imshow(window_name, img)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    node = LidarClusteringNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
