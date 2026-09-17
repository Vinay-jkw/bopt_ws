#!/usr/bin/env python3

import pickle
from ast import Tuple
from ast import List
from detectors.pallete_pole_algorithm import PalletPoleAlgorithm
from processors.pallete_pole_processor import PalletPoleProcessor
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from models.enums import ClusterAlgorithm, TransformProviderType
from models.ppts_context import PPTSContext

from config.config_loader import PPTSConfig

from algorithms.dbscan_cluster import DBSCANCluster
from algorithms.kmeans_cluster import KMeansCluster
from algorithms.euclidean_cluster import EuclideanCluster

from detectors.pallete_detector import PalletDetector

from processors.scan_processor import ScanProcessor
from processors.transform_processor import TransformProcessor
from processors.point_cloud_merger import PointCloudMerger
from processors.cluster_processor import ClusterProcessor
from processors.cluster_feature_processor import ClusterFeatureProcessor
from processors.cluster_validation_processor import ClusterValidationProcessor
from processors.cluster_refinement_processor import ClusterRefinementProcessor
from processors.roi_filter_processor import ROIFilterProcessor
from processors.pallete_feature_processor import PalletFeatureProcessor
from processors.pallete_detection_processor import PalletDetectionProcessor
from processors.pole_candidate_processor import PoleCandidateProcessor
from processors.pallete_path_processor import PalletPathProcessor

from transforms.transform_manager import TransformManager
from transforms.tf_transform_provider import TFTransformProvider
from transforms.yaml_transform_provider import YAMLTransformProvider

from visualization.cluster_visualizer import ClusterVisualizer
from visualization.roi_visualizer import ROIVisualizer
from visualization.point_cloud_visualizer import PointCloudVisualizer
from visualization.pallete_path_visualizer import PalletPathVisualizer
from detectors.pallete_path_detector import PalletPathAlgorithm
from utils.roi_creation import ROIGenerator

class PPTSNode(Node):
    """
    PPTS geometry and pallet-structure processing node.

    Pipeline:
        LaserScan
            -> PointCloud
            -> Transform
            -> Merge
            -> ROI filtering
            -> Clustering
            -> Cluster feature extraction
            -> Cluster validation
            -> Optional K-Means refinement
            -> Pallet feature extraction

    Configuration is accessed exclusively through the refactored
    PPTSConfig object.
    """

    def __init__(self):
        super().__init__("ppts")

        self.config = PPTSConfig(self)
        self.ppts_context = PPTSContext()

        self._initialize_geometry()
        self._initialize_ros_interfaces()
        self._initialize_pipeline()
        self._pipeline_completed = False
        self.path_file_path = "/home/jkw/bopt_ws/src/workflow_node/constructed_rs_path_pp_control.pkl"
        self.get_logger().info("PPTS geometry pipeline started")

    # ==========================================================
    # INITIALIZATION
    # ==========================================================

    def _initialize_geometry(self):
        self._create_roi()
        self._create_tf()
        self._create_transform_manager()
        self._create_processing_components()

    def _initialize_ros_interfaces(self):
        self._create_subscribers()
        self._create_publishers()

    def _initialize_pipeline(self):
        self.pipeline = self._build_pipeline()

    def _initialize_timer(self):
        rate = self.config.general.processing_rate

        if rate <= 0:
            raise ValueError(
                "general.processing_rate must be greater than zero"
            )

        self.timer = self.create_timer(
            1.0 / rate,
            self.process_pipeline,
        )

    # ==========================================================
    # ROI
    # ==========================================================

    def _create_roi(self):
        self.roi_generator = ROIGenerator(
            pallet_config=self.config.pallet,
            roi_config=self.config.roi,
        )

        self.ppts_context.rois = self.roi_generator.generate()

    # ==========================================================
    # TF
    # ==========================================================

    def _create_tf(self):
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

    def _create_transform_manager(self):
        transform_config = self.config.input.transform
        provider = transform_config.provider

        if provider == TransformProviderType.TF:
            transform_provider = TFTransformProvider(
                tf_buffer=self.tf_buffer,
            )

        elif provider == TransformProviderType.YAML:
            transform_provider = YAMLTransformProvider(
                yaml_file=transform_config.yaml_file,
            )

        else:
            raise ValueError(
                f"Unsupported transform provider: {provider}"
            )

        self.transform_manager = TransformManager(
            provider=transform_provider,
        )

    # ==========================================================
    # PROCESSING COMPONENTS
    # ==========================================================

    def _create_processing_components(self):
        self.cluster_algorithm = self._create_cluster_algorithm()
        self.cluster_refinement_algorithm = (
            self._create_cluster_refinement_algorithm()
        )

        self.cluster_feature_processor = ClusterFeatureProcessor()

        self.cluster_validation_processor = (
            ClusterValidationProcessor(
                min_valid_points=(
                    self.config.cluster_validation.min_points
                )
            )
        )
        self.pallet_detector = self._create_pallet_detector()
        self.pallet_path_algorithm = PalletPathAlgorithm()
        self.pallet_pole_algorithm = PalletPoleAlgorithm()
    def _create_pallet_detector(self):
        return PalletDetector(
            expected_poles=(
                self.config.pallet.poles.expected
            ),

            min_detected_poles=(
                self.config.pallet.detection.min_detected_poles
            ),

            expected_row_a_poles=(
                self.config.pallet.poles.rows.row_a.expected
            ),

            expected_row_b_poles=(
                self.config.pallet.poles.rows.row_b.expected
            ),

            expected_longitudinal_spacing=(
                self.config
                .pallet
                .poles
                .spacing
                .longitudinal
                .expected
            ),

            longitudinal_spacing_tolerance=(
                self.config
                .pallet
                .poles
                .spacing
                .longitudinal
                .tolerance
            ),

            expected_row_spacing=(
                self.config
                .pallet
                .geometry
                .width
            ),

            row_spacing_tolerance=0.12,
        )

    def _create_cluster_algorithm(self):
        algorithm = self.config.clustering.algorithm.algorithm

        if algorithm == ClusterAlgorithm.DBSCAN:
            dbscan = self.config.clustering.dbscan

            return DBSCANCluster(
                eps=dbscan.eps,
                min_samples=dbscan.min_samples,
            )
        if algorithm == ClusterAlgorithm.EUCLIDEAN:
            euclidean = self.config.clustering.euclidean
            return EuclideanCluster(
                cluster_tolerance=euclidean.cluster_tolerance,
                min_cluster_size=euclidean.min_cluster_size,
                max_cluster_size=euclidean.max_cluster_size,
            )

        raise ValueError(
            f"Unsupported clustering algorithm: {algorithm}"
        )
    def _create_cluster_refinement_algorithm(self):
        kmeans = self.config.refinement.kmeans

        return KMeansCluster(
            n_clusters=kmeans.n_clusters,
            random_state=kmeans.random_state,
        )

    # ==========================================================
    # SUBSCRIBERS
    # ==========================================================

    def _create_subscribers(self):
        topics = self.config.input.topics

        self.left_scan_sub = self.create_subscription(
            LaserScan,
            topics.left_scan,
            self._left_scan_callback,
            10,
        )

        self.right_scan_sub = self.create_subscription(
            LaserScan,
            topics.right_scan,
            self._right_scan_callback,
            10,
        )

    def _left_scan_callback(self, msg: LaserScan):
        self.ppts_context.left_scan = msg

    def _right_scan_callback(self, msg: LaserScan):
        self.ppts_context.right_scan = msg

    # ==========================================================
    # PUBLISHERS
    # ==========================================================

    def _create_publishers(self):
        self.path_marker_pub = self.create_publisher(
            MarkerArray,
            "/ppts/pallet_path",
            10,
        )
    # ==========================================================
    # PIPELINE
    # ==========================================================

    def _build_pipeline(self):
        preprocessing = self.config.preprocessing

        pipeline = []

        if preprocessing.laser_scan_to_cloud.enabled:
            pipeline.extend([
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
            ])

        if preprocessing.transform.enabled:
            target_frame = self.config.general.frame_id

            pipeline.extend([
                TransformProcessor(
                    input_key="left_cloud",
                    output_key="left_cloud",
                    target_frame=target_frame,
                    transform_manager=self.transform_manager,
                ),
                TransformProcessor(
                    input_key="right_cloud",
                    output_key="right_cloud",
                    target_frame=target_frame,
                    transform_manager=self.transform_manager,
                ),
            ])

        if preprocessing.merge.enabled:
            pipeline.append(
                PointCloudMerger(
                    input_keys=[
                        "left_cloud",
                        "right_cloud",
                    ],
                    output_key="merged_cloud",
                )
            )

        # ROI filtering and clustering operate on the merged cloud.
        pipeline.extend([
            ROIFilterProcessor(
                input_key="merged_cloud",
                output_key="roi_clouds",
            ),
            ClusterProcessor(
                algorithm=self.cluster_algorithm,
            ),
        ])

        if self.config.feature_extraction.enabled:
            pipeline.append(self.cluster_feature_processor)

        pipeline.append(self.cluster_validation_processor)

        refinement = self.config.refinement.kmeans

        if refinement.enabled:
            pipeline.append(
                ClusterRefinementProcessor(
                    algorithm=self.cluster_refinement_algorithm,
                    feature_processor=self.cluster_feature_processor,
                    validation_processor=self.cluster_validation_processor,
                    enabled=True,
                    max_iterations=refinement.max_iteration,
                )
            )

        pipeline.append(
            PoleCandidateProcessor(
                config=self.config.pole_candidate,
                input_key="clusters",
                output_key="pole_candidates",
            )
        )

        pipeline.append(
            PalletFeatureProcessor(
                input_key="pole_candidates",
                output_key="pallet_features",
                row_tolerance=(
                    self.config.pallet.poles.spacing.longitudinal.tolerance
                ),
            )
        )
        pipeline.append(        
            PalletPoleProcessor(
                algorithm=self.pallet_pole_algorithm,
                input_key= "pole_candidates",
                feature_key= "pallet_features",
                output_key= "pallet_pole_input",
            )
        )
        pipeline.append(
            PalletDetectionProcessor(
                detector=self.pallet_detector,
                input_key="pallet_pole_input",
                output_key="pallet_detection",
            )
        )
        pipeline.append(
            PalletPathProcessor(
                input_key="pallet_detection",
                output_key="pallet_path",
                algorithm=self.pallet_path_algorithm,
            ),
        )

        return pipeline
    

    # ==========================================================
    # PIPELINE EXECUTION
    # ==========================================================

    def process_pipeline_once(self):
        if self._pipeline_completed:
            return

        if not self._inputs_ready():
            return

        try:
            for processor in self.pipeline:
                processor.process(self.ppts_context)

            self._log_pallet_detection()
            self._pipeline_completed = True
            data = {
                    'pallet_path': [
                        [wp.x, wp.y, 0.0, wp.yaw]
                        for wp in self.ppts_context.pallet_path.waypoints
                    ],
                    'valid': self.ppts_context.pallet_path.valid,
                    'reason': self.ppts_context.pallet_path.reason
                }

            self._save_path(data)
            self.get_logger().info(
                "PPTS pipeline completed successfully. Shutting down."
            )

        except Exception as exc:
            self.get_logger().error(
                f"Pipeline failed: {type(exc).__name__}: {exc}"
            )
        finally:
            self.pipeline_completed = True

    def process_pipeline(self):
        self.process_pipeline_once()

    def _save_path(self, output) -> None:
        try:
            with open(self.path_file_path, 'wb') as file:
                pickle.dump(output, file)
        except Exception as e:
            self.get_logger().error(f"Failed to save path: {e}")
    
    def _inputs_ready(self):
        return (
            self.ppts_context.left_scan is not None
            and self.ppts_context.right_scan is not None
        )
    def publish_visualization(self):
        path = self.ppts_context.pallet_path

        if path is None or not path.valid:
            empty_path = Path()
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.header.frame_id = "map"

            self.path_pub.publish(empty_path)
            return

        marker_array = self.path_visualizer.create_markers(
            path=path,
            pallet_centroid=(
                self.ppts_context
                .pallet_detection
                .centroid
            ),
            pallet_orientation=(
                self.ppts_context
                .pallet_detection
                .orientation
            ),
        )

        self.path_marker_pub.publish(marker_array)
    
    def _log_pallet_detection(self):
            detection = getattr(
                self.ppts_context,
                "pallet_detection",
                None,
            )

            if detection is None:
                return

            self.get_logger().info(
                f"Pallet Detection | "
                f"detected={detection.detected} | "
                f"score={detection.score:.3f} | "
                f"detected_poles={detection.detected_poles} | "
                f"expected_poles={detection.expected_poles} | "
                f"row_a_count={detection.row_a_count} | "
                f"row_b_count={detection.row_b_count} | "
                f"x_deviation={detection.x_deviation} | "
                f"y_deviation={detection.y_deviation} | "
                f"orientation={detection.orientation}"
            )

            self.get_logger().info(
                f"Pallet Detection Result | "
                f"reason={detection.reason}"
            )



    # ==========================================================
    # SHUTDOWN
    # ==========================================================

    def destroy_node(self):
        if rclpy.ok():
            self.get_logger().info("Shutting down PPTS...")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = PPTSNode()

    try:
        while rclpy.ok() and not node._pipeline_completed:

            rclpy.spin_once(
                node,
                timeout_sec=0.1
            )

            if node._inputs_ready():
                node.process_pipeline_once()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()