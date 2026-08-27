import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from typing import List, Tuple, Optional

class DualLidarFusion(Node):
    def __init__(self):
        super().__init__("dual_lidar_fusion")

        # Subscribers
        self.sub_left = self.create_subscription(LaserScan, "/Lidar_LFTU", self.left_callback, 10)
        self.sub_right = self.create_subscription(LaserScan, "/Lidar_RFTU", self.right_callback, 10)

        # Publisher
        self.pub_cloud = self.create_publisher(PointCloud2, "/merged_lidar_cloud", 10)

        # Store latest scans
        self.left_scan = None
        self.right_scan = None

        # Extrinsics (correct)
        self.translation_left = (0.245, 0.0) 
        self.translation_right = (-0.245, 0.0) 
        self.rotation = (0.0, 0.0, 0.0)

        self.get_logger().info("Dual Lidar Fusion Initialized")

    def left_callback(self, msg):
        self.left_scan = msg
        self.try_publish()

    def right_callback(self, msg):
        self.right_scan = msg
        self.try_publish()

    def try_publish(self):
        if self.left_scan is None or self.right_scan is None:
            return

        pts_left = self.scan_to_xy(self.left_scan)
        pts_right = self.scan_to_xy(self.right_scan)

        # pts_left = self.apply_transformation(pts_left, self.translation_left, self.rotation)
        # pts_right = self.apply_transformation(pts_right, self.translation_left, self.rotation)

        merged = np.vstack((pts_left, pts_right))

        cloud_msg = self.points_to_cloud(merged)
        self.pub_cloud.publish(cloud_msg)

    def scan_to_xy(self, scan):
        angles = scan.angle_min + np.arange(len(scan.ranges)) * scan.angle_increment
        ranges = np.array(scan.ranges)

        valid = np.isfinite(ranges) & (ranges > 0.03) & (ranges < 2.0)
        r = ranges[valid]
        a = angles[valid]

        x = r * np.cos(a)
        y = r * np.sin(a)

        return np.vstack((x, y)).T

    def apply_transformation(self, points: np.ndarray, translation: Tuple[float, float], rotation: Tuple[float, float, float]) -> np.ndarray:
        """
        Apply a static 2D translation and rotation (yaw) to the (x, y) points.
        """
        tx, ty = translation
        yaw = rotation[2]  # Assuming 2D rotation about z-axis (yaw)

        rotation_matrix = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw),  np.cos(yaw)]
        ])

        transformed_points = np.dot(points, rotation_matrix.T) + np.array([tx, ty])
        return transformed_points

    def points_to_cloud(self, pts):
        header = self.left_scan.header  # base_link frame
        fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1)
        ]
        z = np.zeros((pts.shape[0], 1))
        points = np.hstack((pts, z))
        return pc2.create_cloud(header, fields, points)

def main(args=None):
    rclpy.init(args=args)
    node = DualLidarFusion()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
