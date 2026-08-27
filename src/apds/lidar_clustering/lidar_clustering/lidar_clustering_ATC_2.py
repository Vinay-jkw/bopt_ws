#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import json
import time
from typing import List, Tuple, Optional, Dict

from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA, String

########################################
# Helper Functions
########################################

def calculate_rois(l: float, b: float) -> List[Tuple[float, float, float, float]]:
    """
    Calculate three Regions of Interest (ROIs) across the pallet width:
     - left, center, right
    ROI format: (xmin, xmax, ymin, ymax)
    """
    b2 = b / 2.0  # half breadth
    lidar_offset = 0.3   # approximate y offset from lidar to pallet area
    roi_width = 0.3      # width of each ROI in x
    roi_height = l / 2.0 + 0.4  # y-range extent (tunable)

    # Centers for three columns: left, right
    centers_x = [-b2 + roi_width / 2.0, b2 - roi_width / 2.0]

    rois = []
    ymin = lidar_offset - 0.2
    ymax = ymin + roi_height
    for cx in centers_x:
        xmin = cx - roi_width / 2.0
        xmax = cx + roi_width / 2.0
        rois.append((xmin, xmax, ymin, ymax))

    return rois


def apply_transformation(points: np.ndarray, translation: Tuple[float, float], rotation: Tuple[float, float, float]) -> np.ndarray:
    """
    Apply a static 2D translation and rotation (yaw) to the (x, y) points.
    rotation is (roll, pitch, yaw) but we only use yaw.
    """
    if points.size == 0:
        return points.reshape((0, 2))

    tx, ty = translation
    yaw = rotation[2]  # using only yaw

    cos_y, sin_y = np.cos(yaw), np.sin(yaw)
    rot = np.array([[cos_y, -sin_y], [sin_y, cos_y]])

    transformed = np.dot(points, rot.T) + np.array([tx, ty])
    return transformed


########################################
# Base LIDAR Node (shared behavior)
########################################
class BaseLidarNode(Node):
    def __init__(self, node_name: str, scan_topic: str, out_topic: str, translation: Tuple[float, float], rotation: Tuple[float, float, float], marker_ns: str):
        super().__init__(node_name)
        self.roi_pub = self.create_publisher(Marker, 'roi_marker', 10)
        self.cluster_pub = self.create_publisher(String, out_topic, 10)
        self.subscription = self.create_subscription(LaserScan, scan_topic, self.lidar_callback, 10)
        self.translation = translation
        self.rotation = rotation
        self.marker_ns = marker_ns

        # single-shot behavior (default true). User can set param if desired.
        self.declare_parameter("single_shot", True)
        self.single_shot = self.get_parameter("single_shot").value
        self.processed = False

        self.get_logger().info(f"{node_name} initialized. sub: {scan_topic} pub: {out_topic}")

    def calculate_points(self, msg: LaserScan) -> np.ndarray:
        """Convert LaserScan ranges to Nx2 array of (x,y) filtered for finite and valid ranges."""
        ranges = np.array(msg.ranges, dtype=np.float64)
        # handle variable length
        n = ranges.size
        if n == 0:
            return np.zeros((0, 2))

        angles = np.linspace(msg.angle_min, msg.angle_max, n)
        valid_mask = (ranges >= msg.range_min) & (ranges <= msg.range_max) & np.isfinite(ranges)
        if not np.any(valid_mask):
            return np.zeros((0, 2))

        x = ranges[valid_mask] * np.cos(angles[valid_mask])
        y = ranges[valid_mask] * np.sin(angles[valid_mask])
        pts = np.stack((x, y), axis=-1)
        return pts

    def publish_roi_marker(self, roi: Tuple[float, float, float, float], marker_id: int):
        rectangle_points = self.calculate_roi_rectangle(roi)
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = self.marker_ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.05, y=0.0, z=0.0)
        marker.color = ColorRGBA(a=1.0, r=0.0, g=0.0, b=1.0)
        marker.points = [Point(x=p[0], y=p[1], z=0.0) for p in rectangle_points]
        self.roi_pub.publish(marker)

    @staticmethod
    def calculate_roi_rectangle(roi: Tuple[float, float, float, float]) -> List[Tuple[float, float]]:
        xmin, xmax, ymin, ymax = roi
        return [
            (xmin, ymin),
            (xmax, ymin),
            (xmax, ymax),
            (xmin, ymax),
            (xmin, ymin)
        ]


########################################
# Node: Right LIDAR
########################################
class LidarClusteringNodeRight(BaseLidarNode):
    def __init__(self):
        super().__init__('lidar_clustering_node_right',
                         scan_topic='/Lidar_RFTU',
                         out_topic='map1',
                         translation=(-0.245, 0.0),
                         rotation=(0.0, 0.0, 0.0),
                         marker_ns='lidar_right')

    def lidar_callback(self, msg: LaserScan):
        if self.single_shot and self.processed:
            return

        rois = calculate_rois(0.80, 1.4)
        points = self.calculate_points(msg)
        transformed_points = apply_transformation(points, self.translation, self.rotation)

        # split points per ROI
        points_all_rois = {f'roi_{i+1}': self.filter_points_in_roi(transformed_points, roi).tolist()
                           for i, roi in enumerate(rois)}

        # publish markers
        for i, roi in enumerate(rois):
            self.publish_roi_marker(roi, i + 1)

        # create envelope JSON containing metadata
        payload = {
            "header": {
                "frame_id": "base_link",
                "stamp_sec": self.get_clock().now().seconds_nanoseconds()[0]
            },
            "rois": points_all_rois
        }

        jmsg = String()
        jmsg.data = json.dumps(payload)
        self.cluster_pub.publish(jmsg)
        self.get_logger().info(f"Published map1 with {sum(len(v) for v in points_all_rois.values())} points")

        if self.single_shot:
            self.processed = True
            # optional: stop subscribing to reduce CPU
            try:
                self.destroy_subscription(self.subscription)
            except Exception:
                pass

    @staticmethod
    def filter_points_in_roi(points: np.ndarray, roi: Tuple[float, float, float, float]) -> np.ndarray:
        if points.size == 0:
            return np.zeros((0, 2))
        xmin, xmax, ymin, ymax = roi
        mask = (xmin <= points[:, 0]) & (points[:, 0] <= xmax) & (ymin <= points[:, 1]) & (points[:, 1] <= ymax)
        return points[mask]


########################################
# Node: Left LIDAR
########################################
class LidarClusteringNodeLeft(BaseLidarNode):
    def __init__(self):
        super().__init__('lidar_clustering_node_left',
                         scan_topic='/Lidar_LFTU',
                         out_topic='map2',
                         translation=(0.245, 0.0),
                         rotation=(0.0, 0.0, 0.0),
                         marker_ns='lidar_left')

    def lidar_callback(self, msg: LaserScan):
        if self.single_shot and self.processed:
            return

        rois = calculate_rois(0.80, 1.4)
        points = self.calculate_points(msg)
        transformed_points = apply_transformation(points, self.translation, self.rotation)

        points_all_rois = {f'roi_{i+1}': self.filter_points_in_roi(transformed_points, roi).tolist()
                           for i, roi in enumerate(rois)}

        for i, roi in enumerate(rois):
            # offset ids by 100 to avoid collision with right node
            self.publish_roi_marker(roi, 100 + i + 1)

        payload = {
            "header": {
                "frame_id": "base_link",
                "stamp_sec": self.get_clock().now().seconds_nanoseconds()[0]
            },
            "rois": points_all_rois
        }

        jmsg = String()
        jmsg.data = json.dumps(payload)
        self.cluster_pub.publish(jmsg)
        self.get_logger().info(f"Published map2 with {sum(len(v) for v in points_all_rois.values())} points")

        if self.single_shot:
            self.processed = True
            try:
                self.destroy_subscription(self.subscription)
            except Exception:
                pass

    @staticmethod
    def filter_points_in_roi(points: np.ndarray, roi: Tuple[float, float, float, float]) -> np.ndarray:
        if points.size == 0:
            return np.zeros((0, 2))
        xmin, xmax, ymin, ymax = roi
        mask = (xmin <= points[:, 0]) & (points[:, 0] <= xmax) & (ymin <= points[:, 1]) & (points[:, 1] <= ymax)
        return points[mask]


########################################
# Twin LIDAR Aggregator Node
########################################
class TwinLidarNode(Node):
    def __init__(self):
        super().__init__('twin_lidar_node')
        self.subscription_1 = self.create_subscription(String, 'map1', self.callback_1, 10)
        self.subscription_2 = self.create_subscription(String, 'map2', self.callback_2, 10)

        self.result_publisher = self.create_publisher(String, 'pallet_detection_results', 10)

        self.lidar1_data: Optional[Dict] = None
        self.lidar2_data: Optional[Dict] = None

        self.get_logger().info("Twin Lidar Node Initialized")

    def callback_1(self, msg: String):
        self.get_logger().info("Received data from map1")
        try:
            self.lidar1_data = self.parse_cluster_data(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse map1: {e}")
            return
        self.try_process_combined_data()

    def callback_2(self, msg: String):
        self.get_logger().info("Received data from map2")
        try:
            self.lidar2_data = self.parse_cluster_data(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse map2: {e}")
            return
        self.try_process_combined_data()

    def parse_cluster_data(self, msg: str) -> Dict:
        """
        Input payload is expected to be:
        {
          "header": {...},
          "rois": {
             "roi_1": [[x,y], [x,y], ...],
             "roi_2": [...]
          }
        }
        Returns the parsed dict (unchanged) but ensures numeric types.
        """
        data = json.loads(msg)
        rois = data.get("rois", {})
        # convert each element to float (defensive)
        cleaned = {}
        for roi_name, pts in rois.items():
            cleaned_pts = []
            for p in pts:
                try:
                    x, y = float(p[0]), float(p[1])
                    cleaned_pts.append([x, y])
                except Exception:
                    # skip malformed points
                    continue
            cleaned[roi_name] = cleaned_pts
        return {"header": data.get("header", {}), "rois": cleaned}

    def try_process_combined_data(self):
        """
        Called when either side arrives; only proceeds when both available.
        For now, combine the per-ROI point lists and publish a merged JSON.
        """
        if self.lidar1_data is None or self.lidar2_data is None:
            return

        # combine rois: create union of roi keys
        combined = {}
        all_keys = set(self.lidar1_data["rois"].keys()) | set(self.lidar2_data["rois"].keys())
        for k in sorted(all_keys):
            pts1 = self.lidar1_data["rois"].get(k, [])
            pts2 = self.lidar2_data["rois"].get(k, [])
            combined[k] = {
                "from_right": pts1,
                "from_left": pts2,
                "total_points": len(pts1) + len(pts2)
            }

        result_payload = {
            "header": {
                "processed_time": self.get_clock().now().seconds_nanoseconds()[0]
            },
            "combined_rois": combined
        }

        print(combined)
        self.get_logger().info("Published combined pallet_detection_results")

        # If you want to stop after one combined publish, destroy subscriptions:
        try:
            self.destroy_subscription(self.subscription_1)
            self.destroy_subscription(self.subscription_2)
        except Exception:
            pass


########################################
# Main: spin all three nodes together
########################################
def main(args=None):
    rclpy.init(args=args)

    node_right = LidarClusteringNodeRight()
    node_left = LidarClusteringNodeLeft()
    node_twin = TwinLidarNode()

    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node_right)
    executor.add_node(node_left)
    executor.add_node(node_twin)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node_right.destroy_node()
            node_left.destroy_node()
            node_twin.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except RuntimeError as e:
                print(f"Shutdown error: {e}")


if __name__ == '__main__':
    main()
