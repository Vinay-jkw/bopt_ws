#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import pickle
from sensor_msgs.msg import LaserScan, PointCloud2
from visualization_msgs.msg import MarkerArray
from tf2_ros import Buffer, TransformListener
import os
from models.enums import ClusterAlgorithm, TransformProviderType
from models.ppts_context import PPTSContext
from datetime import datetime
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

from transforms.transform_manager import TransformManager
from transforms.tf_transform_provider import TFTransformProvider
from transforms.yaml_transform_provider import YAMLTransformProvider

from visualization.cluster_visualizer import ClusterVisualizer
from visualization.roi_visualizer import ROIVisualizer
from visualization.point_cloud_visualizer import PointCloudVisualizer

from utils.roi_creation import ROIGenerator
from utils.save_ld import Save_Lidar_Data

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
        self._initialize_visualization()
        self._initialize_pipeline()
        self._pipeline_completed = False

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

    def _initialize_visualization(self):
        frame_id = self.config.general.frame_id

        self.point_cloud_visualizer = PointCloudVisualizer(
            frame_id=frame_id
        )
        self.cluster_visualizer = ClusterVisualizer(
            frame_id=frame_id
        )
        self.roi_visualizer = ROIVisualizer(
            frame_id=frame_id
        )

    def _initialize_pipeline(self):
        self.pipeline = self._build_pipeline()

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
            geometry_threshold = 0.60,

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
    def _save_ld(self, left_scan: LaserScan, right_scan: LaserScan, dataset_path: str):
        import glob
        os.makedirs(dataset_path, exist_ok=True)
        existing_files = sorted(
            glob.glob(os.path.join(dataset_path, "frame_*.pkl"))
        )

        frame_number = len(existing_files) + 1

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")

        filename = os.path.join(
            dataset_path,
            f"BOPT002_frame_{timestamp}.pkl"
        )

        dataset = {
            "left_scan": left_scan,
            "right_scan": right_scan
        }

        with open(filename, "wb") as f:
            pickle.dump(dataset, f)

        self.get_logger().info(f"Dataset saved to {filename}")

        self.saved = True

    def _create_publishers(self):
        topics = self.config.input.topics
        output = self.config.output.publish

        self.left_cloud_pub = None
        self.right_cloud_pub = None
        self.merged_cloud_pub = None
        self.cluster_pub = None
        self.roi_pub = None

        if output.point_cloud:
            self.left_cloud_pub = self.create_publisher(
                PointCloud2,
                topics.left_cloud,
                10,
            )
            self.right_cloud_pub = self.create_publisher(
                PointCloud2,
                topics.right_cloud,
                10,
            )
            self.merged_cloud_pub = self.create_publisher(
                PointCloud2,
                topics.merged_cloud,
                10,
            )

        if output.clusters:
            self.cluster_pub = self.create_publisher(
                MarkerArray,
                topics.clusters,
                10,
            )

        if output.rois:
            self.roi_pub = self.create_publisher(
                MarkerArray,
                topics.rois,
                10,
            )

        self.roi_cloud_publishers = {}

        if output.point_cloud:
            for roi in self.ppts_context.rois:
                self.roi_cloud_publishers[roi.id] = (
                    self.create_publisher(
                        PointCloud2,
                        f"/ppts/{roi.id}/cloud",
                        10,
                    )
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
            PalletDetectionProcessor(
                detector=self.pallet_detector,
                input_key="pallet_features",
                output_key="pallet_detection",
            )
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
            # self._publish_results()
            # self._log_statistics()
            self._save_ld(left_scan = self.ppts_context.left_scan, right_scan = self.ppts_context.right_scan, dataset_path = "/home/jkw/bopt_ws/lidar_dataset")
            self._pipeline_completed = True

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

    def _inputs_ready(self):
        return (
            self.ppts_context.left_scan is not None
            and self.ppts_context.right_scan is not None
        )

    # ==========================================================
    # PUBLISH RESULTS
    # ==========================================================

    def _publish_results(self):
        output = self.config.output.publish

        if output.point_cloud:
            self._publish_point_clouds()

        if output.clusters:
            self._publish_clusters()

        if output.rois:
            self._publish_rois()

    def _publish_point_clouds(self):
        clouds = (
            ("left_cloud", self.left_cloud_pub),
            ("right_cloud", self.right_cloud_pub),
            ("merged_cloud", self.merged_cloud_pub),
        )

        for key, publisher in clouds:
            cloud = getattr(self.ppts_context, key, None)

            if cloud is None or publisher is None:
                continue

            publisher.publish(
                self.point_cloud_visualizer.create_cloud_msg(cloud)
            )

        for roi_cloud in self.ppts_context.roi_clouds or []:
            if roi_cloud.cloud is None:
                continue

            publisher = self.roi_cloud_publishers.get(roi_cloud.roi.id)

            if publisher is None:
                continue

            publisher.publish(
                self.point_cloud_visualizer.create_cloud_msg(
                    roi_cloud.cloud
                )
            )

    def _publish_clusters(self):
        if not self.ppts_context.roi_clusters:
            return

        if self.cluster_pub is None:
            return

        self.cluster_pub.publish(
            self.cluster_visualizer.create_markers(
                self.ppts_context.roi_clusters
            )
        )

    def _publish_rois(self):
        if not self.ppts_context.rois or self.roi_pub is None:
            return

        self.roi_pub.publish(
            self.roi_visualizer.create_markers(
                self.ppts_context.rois
            )
        )

    # ==========================================================
    # LOGGING
    # ==========================================================

    def _log_statistics(self):
        left_points = self._cloud_size("left_cloud")
        right_points = self._cloud_size("right_cloud")
        merged_points = self._cloud_size("merged_cloud")
        cluster_count = self._cluster_count()

        self.get_logger().info(
            f"Left: {left_points} | "
            f"Right: {right_points} | "
            f"Merged: {merged_points} | "
            f"Clusters: {cluster_count}"
        )

        self._log_cluster_features()
        self._log_pole_candidates()
        self._log_pallet_features()
        self._log_pallet_detection()

    def _cloud_size(self, attribute):
        cloud = getattr(self.ppts_context, attribute, None)
        return cloud.size if cloud is not None else 0

    def _cluster_count(self):
        count = 0

        for roi_cluster in self.ppts_context.roi_clusters or []:
            count += len(roi_cluster.clusters or [])

        return count

    def _log_cluster_features(self):
        for roi_cluster in self.ppts_context.roi_clusters or []:
            for cluster in roi_cluster.clusters or []:
                self.get_logger().info(
                    f"Cluster {cluster.id} | "
                    f"quality={cluster.quality} | "
                    f"points={cluster.point_count} | "
                    f"centroid={cluster.centroid} | "
                    f"width={cluster.width:.3f} | "
                    f"height={cluster.height:.3f} | "
                    f"yaw={cluster.yaw:.3f} | "
                    f"density={cluster.density:.2f} | "
                    f"major={cluster.major_spread:.4f} | "
                    f"minor={cluster.minor_spread:.4f}"
                )

    def _log_pole_candidates(self):
        candidates = getattr(
            self.ppts_context,
            "pole_candidates",
            None,
        )

        if candidates is None:
            return

        for candidate in candidates:
            cluster = candidate.cluster

            self.get_logger().info(
                f"PoleCandidate | "
                f"cluster={cluster.id} | "
                f"quality={cluster.quality} | "
                f"score={candidate.score:.3f} | "
                f"accepted={candidate.accepted} | "
                f"points={candidate.point_score:.3f} | "
                f"size={candidate.size_score:.3f} | "
                f"aspect={candidate.aspect_score:.3f} | "
                f"shape={candidate.shape_score:.3f}"
            )

        accepted_count = sum(
            candidate.accepted
            for candidate in candidates
        )

        self.get_logger().info(
            f"Pole Candidates | "
            f"total={len(candidates)} | "
            f"accepted={accepted_count}"
        )

    def _log_pallet_features(self):
        pallet_features = getattr(
            self.ppts_context,
            "pallet_features",
            None,
        )

        if pallet_features is None:
            return

        candidates = pallet_features.candidates
        row_a = pallet_features.row_a
        row_b = pallet_features.row_b

        self.get_logger().info(
            f"Pallet Features | "
            f"candidates={len(candidates)} | "
            f"row_a={len(row_a)} | "
            f"row_b={len(row_b)} | "
            f"row_a_order={[c.id for c in row_a]} | "
            f"row_b_order={[c.id for c in row_b]}"
        )

        self.get_logger().info(
            f"Pallet Geometry | "
            f"centroid={pallet_features.centroid} | "
            f"x_range=({pallet_features.min_x:.3f}, "
            f"{pallet_features.max_x:.3f}) | "
            f"y_range=({pallet_features.min_y:.3f}, "
            f"{pallet_features.max_y:.3f})"
        )

        self.get_logger().info(
            f"Pallet Spacing | "
            f"row_a_gaps="
            f"{[round(g, 3) for g in pallet_features.row_a_gaps]} | "
            f"row_b_gaps="
            f"{[round(g, 3) for g in pallet_features.row_b_gaps]} | "
            f"row_a_spacing={pallet_features.row_a_spacing:.3f} | "
            f"row_b_spacing={pallet_features.row_b_spacing:.3f} | "
            f"row_spacing={pallet_features.row_spacing:.3f}"
        )

        for index, cluster in enumerate(row_a):
            self.get_logger().info(
                f"Pallet Row A [{index}] | "
                f"id={cluster.id} | "
                f"x={cluster.centroid[0]:.3f} | "
                f"y={cluster.centroid[1]:.3f}"
            )

        for index, cluster in enumerate(row_b):
            self.get_logger().info(
                f"Pallet Row B [{index}] | "
                f"id={cluster.id} | "
                f"x={cluster.centroid[0]:.3f} | "
                f"y={cluster.centroid[1]:.3f}"
            )

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