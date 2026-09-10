#!/usr/bin/env python3

from config.base_config import BaseConfig
from config.config_models import (
    GeneralConfig,
    InputConfig,
    TopicsConfig,
    TransformConfig,
    LaserScanToCloudConfig,
    PreprocessingTransformConfig,
    AlignmentConfig,
    MergeConfig,
    UseROI,
    PreprocessingConfig,
    ROIConfig,
    ROIGridConfig,
    ROIGeometryConfig,
    ROICustomConfig,
    CustomROIRegion,
    AlgorithmConfig,
    ClusteringConfig,
    DBSCANConfig,
    EuclideanConfig,
    ClusterValidationConfig,
    RefinementConfig,
    KMeansConfig,
    FeatureExtractionConfig,
    PoleDetectionConfig,
    PoleGeometryConfig,
    PoleWidthConfig,
    PoleHeightConfig,
    PoleAspectRatioConfig,
    PoleScoringConfig,
    PalletConfig,
    PalletGeometryConfig,
    PalletPolesConfig,
    PoleRowsConfig,
    PoleRowConfig,
    PoleSpacingConfig,
    LongitudinalSpacingConfig,
    PalletDetectionConfig,
    OutputConfig,
    OutputPublishConfig,
    VisualizationConfig,
    PoleCandidateConfig,
    PoleCandidateWeightsConfig,
    PoleCandidateQualityConfig,
)
from models.enums import ClusterAlgorithm, TransformProviderType


class PPTSConfig(BaseConfig):
    """
    Strongly typed PPTS configuration.

    The object structure mirrors the YAML hierarchy so pipeline code can
    use explicit configuration objects instead of raw parameter names.
    """

    def __init__(self, node, config_file=None):
        super().__init__(node, config_file)

        self.general = self._build_general()
        self.input = self._build_input()
        self.preprocessing = self._build_preprocessing()
        self.roi = self._build_roi()
        self.clustering = self._build_clustering()
        self.cluster_validation = self._build_cluster_validation()
        self.refinement = self._build_refinement()
        self.feature_extraction = self._build_feature_extraction()
        self.pole_candidate = self._build_pole_candidate()
        self.pole_detection = self._build_pole_detection()
        self.pallet = self._build_pallet()
        self.output = self._build_output()
        self.visualization = self._build_visualization()

        self._validate()

    # ==========================================================
    # BUILDERS
    # ==========================================================

    def _build_general(self):
        return GeneralConfig(
            frame_id=self.get_str(
                "general.frame_id",
                "load_wheel_base_link",
            ),
            processing_rate=self.get_float(
                "general.processing_rate",
                10.0,
            ),
        )

    def _build_input(self):
        topics = TopicsConfig(
            left_scan=self.get_str(
                "input.topics.left_scan", "/Lidar_LFT"
            ),
            right_scan=self.get_str(
                "input.topics.right_scan", "/Lidar_RFT"
            ),
            left_cloud=self.get_str(
                "input.topics.left_cloud", "/ppts/left_cloud"
            ),
            right_cloud=self.get_str(
                "input.topics.right_cloud", "/ppts/right_cloud"
            ),
            merged_cloud=self.get_str(
                "input.topics.merged_cloud", "/ppts/merged_cloud"
            ),
            clusters=self.get_str(
                "input.topics.clusters", "/ppts/clusters"
            ),
            rois=self.get_str(
                "input.topics.rois", "/ppts/rois"
            ),
        )

        provider_name = self.get_str(
            "input.transform.provider", "tf"
        ).lower()

        try:
            provider = TransformProviderType(provider_name)
        except ValueError as exc:
            valid = [item.value for item in TransformProviderType]
            raise ValueError(
                f"Invalid input.transform.provider '{provider_name}'. "
                f"Expected one of {valid}"
            ) from exc

        transform = TransformConfig(
            provider=provider,
            timeout=self.get_float(
                "input.transform.timeout", 0.2
            ),
            yaml_file=self.get_str(
                "input.transform.yaml_file", ""
            ),
        )

        return InputConfig(
            topics=topics,
            transform=transform,
        )

    def _build_preprocessing(self):
        return PreprocessingConfig(
            laser_scan_to_cloud=LaserScanToCloudConfig(
                enabled=self.get_bool(
                    "preprocessing.laser_scan_to_cloud.enabled",
                    True,
                )
            ),
            transform=PreprocessingTransformConfig(
                enabled=self.get_bool(
                    "preprocessing.transform.enabled",
                    True,
                )
            ),
            alignment=AlignmentConfig(
                enabled=self.get_bool(
                    "preprocessing.alignment.enabled",
                    True,
                )
            ),
            merge=MergeConfig(
                enabled=self.get_bool(
                    "preprocessing.merge.enabled",
                    True,
                )
            ),
            use_roi = UseROI(
                enabled=self.get_bool(
                    "preprocessing.use_roi.enabled",
                    True,
                )
            )
        )

    def _build_roi(self):
        layout = self.get_str("roi.layout", "grid").lower()

        grid = ROIGridConfig(
            rows=self.get_int("roi.grid.rows", 1),
            columns=self.get_int("roi.grid.columns", 1),
        )

        geometry = ROIGeometryConfig(
            inset_width=self.get_float(
                "roi.geometry.inset_width", 0.20
            ),
            inset_length=self.get_float(
                "roi.geometry.inset_length", 0.30
            ),
            lidar_offset=self.get_float(
                "roi.geometry.lidar_offset", 0.435
            ),
        )

        raw_regions = self.get("roi.custom.regions", []) or []
        if not isinstance(raw_regions, list):
            raise ValueError("'roi.custom.regions' must be a list")

        regions = []
        for index, region in enumerate(raw_regions):
            if not isinstance(region, dict):
                raise ValueError(
                    f"Custom ROI region #{index} must be a dictionary"
                )

            required = {"id", "xmin", "xmax", "ymin", "ymax"}
            missing = required - region.keys()

            if missing:
                raise ValueError(
                    f"Custom ROI region #{index} is missing: "
                    f"{sorted(missing)}"
                )

            regions.append(
                CustomROIRegion(
                    id=str(region["id"]),
                    xmin=float(region["xmin"]),
                    xmax=float(region["xmax"]),
                    ymin=float(region["ymin"]),
                    ymax=float(region["ymax"]),
                )
            )

        return ROIConfig(
            layout=layout,
            grid=grid,
            geometry=geometry,
            custom=ROICustomConfig(regions=regions),
        )

    def _build_clustering(self):
        algorithm_name = self.get_str(
            "clustering.algorithm",
            "dbscan",
        ).lower()

        try:
            algorithm = ClusterAlgorithm(algorithm_name)
        except ValueError as exc:
            valid = [item.value for item in ClusterAlgorithm]
            raise ValueError(
                f"Invalid clustering.algorithm '{algorithm_name}'. "
                f"Expected one of {valid}"
            ) from exc

        return ClusteringConfig(
            algorithm=AlgorithmConfig(
                algorithm=algorithm
            ),

            dbscan=DBSCANConfig(
                eps=self.get_float(
                    "clustering.dbscan.eps",
                    0.06,
                ),

                min_samples=self.get_int(
                    "clustering.dbscan.min_samples",
                    3,
                ),
            ),

            euclidean=EuclideanConfig(
                cluster_tolerance=self.get_float(
                    "clustering.euclidean.cluster_tolerance",
                    0.05,
                ),

                min_cluster_size=self.get_int(
                    "clustering.euclidean.min_cluster_size",
                    10,
                ),

                max_cluster_size=self.get_int(
                    "clustering.euclidean.max_cluster_size",
                    5000,
                ),
            ),
        )

    def _build_cluster_validation(self):
        return ClusterValidationConfig(
            min_points=self.get_int(
                "cluster_validation.min_points", 4
            )
        )

    def _build_refinement(self):
        return RefinementConfig(
            kmeans=KMeansConfig(
                enabled=self.get_bool(
                    "refinement.kmeans.enabled", True
                ),
                max_iteration=self.get_int(
                    "refinement.kmeans.max_iteration", 3
                ),
                n_clusters=self.get_int(
                    "refinement.kmeans.n_clusters", 2
                ),
                random_state=self.get_int(
                    "refinement.kmeans.random_state", 42
                ),
            )
        )

    def _build_feature_extraction(self):
        features = self.get(
            "feature_extraction.features",
            [
                "point_count",
                "centroid",
                "width",
                "height",
                "major_spread",
                "minor_spread",
                "aspect_ratio",
                "density",
                "yaw",
            ],
        )

        if not isinstance(features, list):
            raise ValueError(
                "feature_extraction.features must be a list"
            )

        return FeatureExtractionConfig(
            enabled=self.get_bool(
                "feature_extraction.enabled", True
            ),
            features=[str(feature) for feature in features],
        )

    def _build_pole_candidate(self):
        return PoleCandidateConfig(
            min_points=self.get_int(
                "pole_candidate.min_points",
                3,
            ),
            saturation_points = self.get_int(
                "saturation_points",
                10,
            ),
            min_width=self.get_float(
                "pole_candidate.min_width",
                0.005,
            ),

            max_width=self.get_float(
                "pole_candidate.max_width",
                0.10,
            ),

            min_height=self.get_float(
                "pole_candidate.min_height",
                0.04,
            ),

            max_height=self.get_float(
                "pole_candidate.max_height",
                0.16,
            ),

            min_aspect_ratio=self.get_float(
                "pole_candidate.min_aspect_ratio",
                1.3,
            ),

            min_score=self.get_float(
                "pole_candidate.min_score",
                0.55,
            ),

            weights=PoleCandidateWeightsConfig(
                point=self.get_float(
                    "pole_candidate.weights.point",
                    0.25,
                ),

                size=self.get_float(
                    "pole_candidate.weights.size",
                    0.25,
                ),

                aspect=self.get_float(
                    "pole_candidate.weights.aspect",
                    0.25,
                ),

                shape=self.get_float(
                    "pole_candidate.weights.shape",
                    0.25,
                ),
            ),

            quality=PoleCandidateQualityConfig(
                weak_factor=self.get_float(
                    "pole_candidate.quality.weak_factor",
                    0.70,
                ),

                suspicious_factor=self.get_float(
                    "pole_candidate.quality.suspicious_factor",
                    0.50,
                ),
            ),
        )
    def _build_pole_detection(self):
        return PoleDetectionConfig(
            geometry=PoleGeometryConfig(
                width=PoleWidthConfig(
                    min=self.get_float(
                        "pole_detection.geometry.width.min",
                        0.005,
                    ),
                    max=self.get_float(
                        "pole_detection.geometry.width.max",
                        0.10,
                    ),
                ),
                height=PoleHeightConfig(
                    min=self.get_float(
                        "pole_detection.geometry.height.min",
                        0.04,
                    ),
                    max=self.get_float(
                        "pole_detection.geometry.height.max",
                        0.16,
                    ),
                ),
                aspect_ratio=PoleAspectRatioConfig(
                    min=self.get_float(
                        "pole_detection.geometry.aspect_ratio.min",
                        1.3,
                    ),
                ),
            ),
            scoring=PoleScoringConfig(
                min_candidate_score=self.get_float(
                    "pole_detection.scoring.min_candidate_score",
                    0.55,
                )
            ),
        )

    def _build_pallet(self):
        return PalletConfig(
            type=self.get_str("pallet.type", "standard"),
            geometry=PalletGeometryConfig(
                length=self.get_float(
                    "pallet.geometry.length", 1.40
                ),
                width=self.get_float(
                    "pallet.geometry.width", 0.80
                ),
            ),
            poles=PalletPolesConfig(
                expected=self.get_int(
                    "pallet.poles.expected", 8
                ),
                rows=PoleRowsConfig(
                    row_a=PoleRowConfig(
                        expected=self.get_int(
                            "pallet.poles.rows.row_a.expected", 4
                        )
                    ),
                    row_b=PoleRowConfig(
                        expected=self.get_int(
                            "pallet.poles.rows.row_b.expected", 4
                        )
                    ),
                ),
                spacing=PoleSpacingConfig(
                    longitudinal=LongitudinalSpacingConfig(
                        expected=self.get_float(
                            "pallet.poles.spacing.longitudinal.expected",
                            0.46,
                        ),
                        tolerance=self.get_float(
                            "pallet.poles.spacing.longitudinal.tolerance",
                            0.08,
                        ),
                    )
                ),
            ),
            detection=PalletDetectionConfig(
                min_detected_poles=self.get_int(
                    "pallet.detection.min_detected_poles", 6
                )
            ),
        )

    def _build_output(self):
        return OutputConfig(
            publish=OutputPublishConfig(
                point_cloud=self.get_bool(
                    "output.publish.point_cloud", True
                ),
                rois=self.get_bool(
                    "output.publish.rois", True
                ),
                clusters=self.get_bool(
                    "output.publish.clusters", True
                ),
                pole_candidates=self.get_bool(
                    "output.publish.pole_candidates", True
                ),
                pallet_result=self.get_bool(
                    "output.publish.pallet_result", True
                ),
            )
        )

    def _build_visualization(self):
        return VisualizationConfig(
            point_size=self.get_float(
                "visualization.point_size", 0.03
            ),
            bbox_width=self.get_float(
                "visualization.bbox_width", 0.02
            ),
            text_height=self.get_float(
                "visualization.text_height", 0.15
            ),
        )

    # ==========================================================
    # VALIDATION
    # ==========================================================

    def _validate(self):
        self._validate_general()
        self._validate_transform()
        self._validate_preprocessing()
        self._validate_roi()
        self._validate_clustering()
        self._validate_refinement()
        self._validate_features()
        self._validate_pole_candidate()
        self._validate_pole_geometry()
        self._validate_pallet()

    def _validate_general(self):
        if self.general.processing_rate <= 0:
            raise ValueError(
                "general.processing_rate must be greater than 0"
            )

        if not self.general.frame_id.strip():
            raise ValueError("general.frame_id cannot be empty")

    def _validate_transform(self):
        if self.input.transform.timeout <= 0:
            raise ValueError(
                "input.transform.timeout must be greater than 0"
            )

        if (
            self.input.transform.provider.value == "yaml"
            and not self.input.transform.yaml_file.strip()
        ):
            raise ValueError(
                "input.transform.yaml_file is required when "
                "input.transform.provider is 'yaml'"
            )

    def _validate_preprocessing(self):
        # No cross-field constraints currently required.
        pass

    def _validate_roi(self):
        if self.roi.layout not in {"grid", "custom"}:
            raise ValueError(
                f"Invalid roi.layout '{self.roi.layout}'. "
                "Expected 'grid' or 'custom'"
            )

        if self.roi.grid.rows <= 0:
            raise ValueError("roi.grid.rows must be greater than 0")

        if self.roi.grid.columns <= 0:
            raise ValueError(
                "roi.grid.columns must be greater than 0"
            )

        if self.roi.geometry.inset_length < 0:
            raise ValueError(
                "roi.geometry.inset_length cannot be negative"
            )

        if self.roi.geometry.inset_width < 0:
            raise ValueError(
                "roi.geometry.inset_width cannot be negative"
            )

        if self.roi.geometry.lidar_offset < 0:
            raise ValueError(
                "roi.geometry.lidar_offset cannot be negative"
            )

        if self.roi.layout != "custom":
            return

        if not self.roi.custom.regions:
            raise ValueError(
                "Custom ROI layout requires at least one region"
            )

        ids = set()

        for region in self.roi.custom.regions:
            if region.id in ids:
                raise ValueError(
                    f"Duplicate custom ROI id '{region.id}'"
                )
            ids.add(region.id)

            values = (
                region.xmin,
                region.xmax,
                region.ymin,
                region.ymax,
            )

            if any(value < 0.0 or value > 1.0 for value in values):
                raise ValueError(
                    f"Custom ROI '{region.id}' coordinates must be "
                    "between 0.0 and 1.0"
                )

            if region.xmin >= region.xmax:
                raise ValueError(
                    f"Custom ROI '{region.id}': "
                    "xmin must be smaller than xmax"
                )

            if region.ymin >= region.ymax:
                raise ValueError(
                    f"Custom ROI '{region.id}': "
                    "ymin must be smaller than ymax"
                )

    def _validate_clustering(self):
        # ------------------------------------------------------
        # DBSCAN
        # ------------------------------------------------------

        if self.clustering.dbscan.eps <= 0:
            raise ValueError(
                "clustering.dbscan.eps must be greater than 0"
            )

        if self.clustering.dbscan.min_samples < 1:
            raise ValueError(
                "clustering.dbscan.min_samples must be >= 1"
            )

        # ------------------------------------------------------
        # Cluster validation
        # ------------------------------------------------------

        if self.cluster_validation.min_points < 1:
            raise ValueError(
                "cluster_validation.min_points must be >= 1"
            )

        # ------------------------------------------------------
        # Euclidean clustering
        # ------------------------------------------------------

        euclidean = self.clustering.euclidean

        if euclidean.cluster_tolerance <= 0:
            raise ValueError(
                "clustering.euclidean.cluster_tolerance "
                "must be greater than 0"
            )

        if euclidean.min_cluster_size < 1:
            raise ValueError(
                "clustering.euclidean.min_cluster_size must be >= 1"
            )

        if euclidean.max_cluster_size < 1:
            raise ValueError(
                "clustering.euclidean.max_cluster_size must be >= 1"
            )

        if euclidean.max_cluster_size < euclidean.min_cluster_size:
            raise ValueError(
                "clustering.euclidean.max_cluster_size must be "
                "greater than or equal to min_cluster_size"
            )

    def _validate_refinement(self):
        kmeans = self.refinement.kmeans

        if kmeans.max_iteration < 1:
            raise ValueError(
                "refinement.kmeans.max_iteration must be >= 1"
            )

        if kmeans.n_clusters < 2:
            raise ValueError(
                "refinement.kmeans.n_clusters must be >= 2"
            )

    def _validate_features(self):
        if self.feature_extraction.enabled and not self.feature_extraction.features:
            raise ValueError(
                "feature_extraction.features cannot be empty when "
                "feature extraction is enabled"
            )

    def _validate_pole_candidate(self):
        candidate = self.pole_candidate

        # ------------------------------------------------------
        # Basic point count
        # ------------------------------------------------------

        if candidate.min_points < 1:
            raise ValueError(
                "pole_candidate.min_points must be >= 1"
            )

        # ------------------------------------------------------
        # Width
        # ------------------------------------------------------

        if candidate.min_width <= 0:
            raise ValueError(
                "pole_candidate.min_width must be greater than 0"
            )

        if candidate.max_width <= candidate.min_width:
            raise ValueError(
                "pole_candidate.max_width must be greater than "
                "pole_candidate.min_width"
            )

        # ------------------------------------------------------
        # Height
        # ------------------------------------------------------

        if candidate.min_height <= 0:
            raise ValueError(
                "pole_candidate.min_height must be greater than 0"
            )

        if candidate.max_height <= candidate.min_height:
            raise ValueError(
                "pole_candidate.max_height must be greater than "
                "pole_candidate.min_height"
            )

        # ------------------------------------------------------
        # Aspect ratio
        # ------------------------------------------------------

        if candidate.min_aspect_ratio <= 0:
            raise ValueError(
                "pole_candidate.min_aspect_ratio must be greater than 0"
            )

        # ------------------------------------------------------
        # Candidate score
        # ------------------------------------------------------

        if not 0.0 <= candidate.min_score <= 1.0:
            raise ValueError(
                "pole_candidate.min_score must be between 0.0 and 1.0"
            )

        # ------------------------------------------------------
        # Weights
        # ------------------------------------------------------

        weights = candidate.weights

        if weights.point < 0:
            raise ValueError(
                "pole_candidate.weights.point cannot be negative"
            )

        if weights.size < 0:
            raise ValueError(
                "pole_candidate.weights.size cannot be negative"
            )

        if weights.aspect < 0:
            raise ValueError(
                "pole_candidate.weights.aspect cannot be negative"
            )

        if weights.shape < 0:
            raise ValueError(
                "pole_candidate.weights.shape cannot be negative"
            )

        weight_sum = (
            weights.point
            + weights.size
            + weights.aspect
            + weights.shape
        )

        if abs(weight_sum - 1.0) > 1e-6:
            raise ValueError(
                "pole_candidate.weights must sum to 1.0"
            )

        # ------------------------------------------------------
        # Quality factors
        # ------------------------------------------------------

        quality = candidate.quality

        if quality.weak_factor < 0:
            raise ValueError(
                "pole_candidate.quality.weak_factor cannot be negative"
            )

        if quality.suspicious_factor < 0:
            raise ValueError(
                "pole_candidate.quality.suspicious_factor "
                "cannot be negative"
            )

        if quality.suspicious_factor > quality.weak_factor:
            raise ValueError(
                "pole_candidate.quality.suspicious_factor must be "
                "less than or equal to weak_factor"
            )
    def _validate_pole_geometry(self):
        geometry = self.pole_detection.geometry

        if geometry.width.min <= 0:
            raise ValueError(
                "pole_detection.geometry.width.min must be greater than 0"
            )

        if geometry.width.max <= geometry.width.min:
            raise ValueError(
                "pole_detection.geometry.width.max must be greater than min"
            )

        if geometry.height.min <= 0:
            raise ValueError(
                "pole_detection.geometry.height.min must be greater than 0"
            )

        if geometry.height.max <= geometry.height.min:
            raise ValueError(
                "pole_detection.geometry.height.max must be greater than min"
            )

        if geometry.aspect_ratio.min <= 0:
            raise ValueError(
                "pole_detection.geometry.aspect_ratio.min must be greater than 0"
            )

        score = self.pole_detection.scoring.min_candidate_score
        if not 0.0 <= score <= 1.0:
            raise ValueError(
                "pole_detection.scoring.min_candidate_score must be "
                "between 0.0 and 1.0"
            )

    def _validate_pallet(self):
        pallet = self.pallet

        if pallet.geometry.length <= 0:
            raise ValueError(
                "pallet.geometry.length must be greater than 0"
            )

        if pallet.geometry.width <= 0:
            raise ValueError(
                "pallet.geometry.width must be greater than 0"
            )

        if pallet.geometry.length <= 2 * self.roi.geometry.inset_length:
            raise ValueError(
                "roi.geometry.inset_length is too large for "
                "pallet.geometry.length"
            )

        if pallet.geometry.width <= 2 * self.roi.geometry.inset_width:
            raise ValueError(
                "roi.geometry.inset_width is too large for "
                "pallet.geometry.width"
            )

        if pallet.poles.expected <= 0:
            raise ValueError(
                "pallet.poles.expected must be greater than 0"
            )

        row_a = pallet.poles.rows.row_a.expected
        row_b = pallet.poles.rows.row_b.expected

        if row_a <= 0 or row_b <= 0:
            raise ValueError(
                "pallet.poles.rows expected values must be greater than 0"
            )

        if row_a + row_b != pallet.poles.expected:
            raise ValueError(
                "pallet pole row counts must sum to pallet.poles.expected"
            )

        spacing = pallet.poles.spacing.longitudinal
        if spacing.expected <= 0:
            raise ValueError(
                "pallet.poles.spacing.longitudinal.expected "
                "must be greater than 0"
            )

        if spacing.tolerance < 0:
            raise ValueError(
                "pallet.poles.spacing.longitudinal.tolerance "
                "cannot be negative"
            )

        min_poles = pallet.detection.min_detected_poles

        if min_poles <= 0:
            raise ValueError(
                "pallet.detection.min_detected_poles must be greater than 0"
            )

        if min_poles > pallet.poles.expected:
            raise ValueError(
                "pallet.detection.min_detected_poles cannot be greater "
                "than pallet.poles.expected"
            )