#!/usr/bin/env python3

import os
import pickle

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from message_filters import Subscriber, ApproximateTimeSynchronizer


class LidarDatasetRecorder(Node):

    def __init__(self):
        super().__init__("lidar_dataset_recorder")

        self.dataset_path = "/home/jkw/bopt_ws/lidar_dataset"
        os.makedirs(self.dataset_path, exist_ok=True)

        self.left_sub = Subscriber(self, LaserScan, "/Lidar_LFT")
        self.right_sub = Subscriber(self, LaserScan, "/Lidar_RFT")

        self.sync = ApproximateTimeSynchronizer(
            [self.left_sub, self.right_sub],
            queue_size=10,
            slop=0.05
        )

        self.sync.registerCallback(self.callback)

        self.saved = False

        self.get_logger().info("Waiting for synchronized LaserScan messages...")

    def callback(self, left_scan, right_scan):

        if self.saved:
            return

        import glob

        existing_files = sorted(
            glob.glob(os.path.join(self.dataset_path, "frame_*.pkl"))
        )

        frame_number = len(existing_files) + 1

        filename = os.path.join(
            self.dataset_path,
            f"frame_{frame_number:06d}.pkl"
        )

        dataset = {
            "left_scan": left_scan,
            "right_scan": right_scan
        }

        with open(filename, "wb") as f:
            pickle.dump(dataset, f)

        self.get_logger().info(f"Dataset saved to {filename}")

        self.saved = True

        # Shutdown after saving one frame
        self.get_logger().info("Exiting...")


def Save_Lidar_Data(args=None):

    rclpy.init(args=args)

    node = LidarDatasetRecorder()

    print("Starting loop")

    try:
        while rclpy.ok() and not node.saved:
            print("spin_once")
            rclpy.spin_once(node, timeout_sec=0.1)

        print("Exited loop")

    except KeyboardInterrupt:
        print("KeyboardInterrupt")

    finally:
        print("Destroying node")
        node.destroy_node()

        print("Shutting down")
        rclpy.shutdown()

        print("Done")


if __name__ == "__main__":
    Save_Lidar_Data()