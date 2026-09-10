#!/usr/bin/env python3

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import (
    LaserScan,
    PointCloud2,
)

from config.config_loader import PPTSConfig

from models.ppts_context import PPTSContext

from processors.scan_processor import ScanProcessor
from processors.transform_processor import TransformProcessor
from processors.point_cloud_merger import PointCloudMerger
from transforms.transform_manager import TransformManager
from transforms.tf_transform_provider import TFTransformProvider
from processors.cluster_processor import ClusterProcessor

from tf2_ros import (
    Buffer,
    TransformListener,
)
from visualization.point_cloud_visualizer import PointCloudVisualizer


class PPTSNode(Node):
    """
    PPTS Geometry Pipeline (v0.1)

    Pipeline
    --------
    LaserScan
        ↓
    ScanProcessor
        ↓
    TransformProcessor
        ↓
    PointCloudMerger
        ↓
    PointCloudVisualizer
    """

    def __init__(self):

        super().__init__("ppts")

        # ------------------------------------------
        # Initialization
        # ------------------------------------------

        self.load_configuration()

        self.create_context()

        self.create_tf()

        self.create_subscribers()

        self.create_publishers()

        self.create_transform_manager()

        self.create_visualizers()

        self.create_pipeline()

        self.create_tick_timer()

        self.get_logger().info(
            "PPTS v0.1 Geometry Pipeline Started"
        )

    # ==========================================================
    # Configuration
    # ==========================================================

    def load_configuration(self):

        self.config = PPTSConfig(self)

    # ==========================================================
    # Context
    # ==========================================================

    def create_context(self):

        self.ppts_context = PPTSContext()
    # ==========================================================
    # TF
    # ==========================================================
    def create_tf(self):
        """
        Initialize TF2 infrastructure.
        """

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )
    # ==========================================================
    # Subscribers
    # ==========================================================

    def create_subscribers(self):

        self.left_scan_sub = self.create_subscription(
            LaserScan,
            self.config.topics.left_scan,
            self.left_callback,
            10,
        )

        self.right_scan_sub = self.create_subscription(
            LaserScan,
            self.config.topics.right_scan,
            self.right_callback,
            10,
        )

    # ==========================================================
    # Publishers
    # ==========================================================

    def create_publishers(self):

        self.left_cloud_pub = self.create_publisher(
            PointCloud2,
            self.config.topics.left_cloud,
            10,
        )

        self.right_cloud_pub = self.create_publisher(
            PointCloud2,
            self.config.topics.right_cloud,
            10,
        )

        self.merged_cloud_pub = self.create_publisher(
            PointCloud2,
            self.config.topics.merged_cloud,
            10,
        )
        



    # ==========================================================
    # Transform Manager
    # ==========================================================

    def create_transform_manager(self):
        """
        Create the transform manager using the configured provider.
        """

        transform_provider = TFTransformProvider(
            tf_buffer=self.tf_buffer,
        )

        self.transform_manager = TransformManager(
            provider=transform_provider,
        )

    # ==========================================================
    # Visualizers
    # ==========================================================

    def create_visualizers(self):

        self.point_cloud_visualizer = PointCloudVisualizer(
            frame_id=self.config.general.frame_id,
        )

    # ==========================================================
    # Processing Pipeline
    # ==========================================================

    def create_pipeline(self):

        self.pipeline = [

            ScanProcessor(
                input_key="left_scan",
                output_key="left_cloud",
                source="left_lidar",
            ),

            ScanProcessor(
                input_key="right_scan",
                output_key="right_cloud",
                source="right_lidar",
            ),

            TransformProcessor(
                input_key="left_cloud",
                output_key="left_cloud",
                target_frame=self.config.general.frame_id,
                transform_manager=self.transform_manager,
            ),

            TransformProcessor(
                input_key="right_cloud",
                output_key="right_cloud",
                target_frame=self.config.general.frame_id,
                transform_manager=self.transform_manager,
            ),

            PointCloudMerger(
                input_keys=[
                    "left_cloud",
                    "right_cloud",
                ],
                output_key="merged_cloud",
            ),

        ]

    # ==========================================================
    # Timer
    # ==========================================================

    def create_tick_timer(self):

        self.timer = self.create_timer(
            1.0 / self.config.general.processing_rate,
            self.process_pipeline,
        )

    # ==========================================================
    # Callbacks
    # ==========================================================

    def left_callback(
        self,
        msg: LaserScan,
    ):

        self.ppts_context.left_scan = msg

    # ----------------------------------------------------------

    def right_callback(
        self,
        msg: LaserScan,
    ):

        self.ppts_context.right_scan = msg

    # ==========================================================
    # Pipeline
    # ==========================================================

    def process_pipeline(self):

        # Wait until both scans are available
        if (
            self.ppts_context.left_scan is None
            or self.ppts_context.right_scan is None
        ):
            return

        # Execute the complete pipeline
        for processor in self.pipeline:

            processor.process(self.ppts_context)

        # Publish results
        self.publish_results()

        # Debug
        self.log_statistics()

    # ==========================================================
    # Publishers
    # ==========================================================

    def publish_results(self):

        # -----------------------------
        # Left Cloud
        # -----------------------------

        left_cloud_msg = (
            self.point_cloud_visualizer.create_cloud_msg(
                self.ppts_context.left_cloud
            )
        )

        self.left_cloud_pub.publish(
            left_cloud_msg
        )

        # -----------------------------
        # Right Cloud
        # -----------------------------

        right_cloud_msg = (
            self.point_cloud_visualizer.create_cloud_msg(
                self.ppts_context.right_cloud
            )
        )

        self.right_cloud_pub.publish(
            right_cloud_msg
        )

        # -----------------------------
        # Merged Cloud
        # -----------------------------

        merged_cloud_msg = (
            self.point_cloud_visualizer.create_cloud_msg(
                self.ppts_context.merged_cloud
            )
        )

        self.merged_cloud_pub.publish(
            merged_cloud_msg
        )

    # ==========================================================
    # Logging
    # ==========================================================

    def log_statistics(self):

        left_points = (
            self.ppts_context.left_cloud.size
            if self.ppts_context.left_cloud
            else 0
        )

        right_points = (
            self.ppts_context.right_cloud.size
            if self.ppts_context.right_cloud
            else 0
        )

        merged_points = (
            self.ppts_context.merged_cloud.size
            if self.ppts_context.merged_cloud
            else 0
        )

        self.get_logger().debug(

            f"Left: {left_points} | "
            f"Right: {right_points} | "
            f"Merged: {merged_points}"

        )

def main(args=None):

    rclpy.init(args=args)

    node = PPTSNode()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Shutting down PPTS..."
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()