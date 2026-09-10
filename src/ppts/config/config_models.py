#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import List

from models.enums import ClusterAlgorithm, TransformProviderType


# ==========================================================
# 1. GENERAL
# ==========================================================

@dataclass
class GeneralConfig:
    frame_id: str
    processing_rate: float


# ==========================================================
# 2. INPUT
# ==========================================================

@dataclass
class TopicsConfig:
    left_scan: str
    right_scan: str
    left_cloud: str
    right_cloud: str
    merged_cloud: str
    clusters: str
    rois: str


@dataclass
class TransformConfig:
    provider: TransformProviderType
    timeout: float
    yaml_file: str = ""


@dataclass
class InputConfig:
    topics: TopicsConfig
    transform: TransformConfig


# ==========================================================
# 3. PREPROCESSING
# ==========================================================

@dataclass
class LaserScanToCloudConfig:
    enabled: bool = True


@dataclass
class PreprocessingTransformConfig:
    """
    Controls whether coordinate transformation is applied.

    The actual transform provider/timeout/file are configured under
    InputConfig.transform.
    """
    enabled: bool = True


@dataclass
class AlignmentConfig:
    enabled: bool = True


@dataclass
class MergeConfig:
    enabled: bool = True
@dataclass
class UseROI:
    enabled: bool = False

@dataclass
class PreprocessingConfig:
    laser_scan_to_cloud: LaserScanToCloudConfig
    transform: PreprocessingTransformConfig
    alignment: AlignmentConfig
    merge: MergeConfig
    use_roi: UseROI

# ==========================================================
# 4. ROI
# ==========================================================

@dataclass
class CustomROIRegion:
    id: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float


@dataclass
class ROIGeometryConfig:
    inset_width: float = 0.20
    inset_length: float = 0.30
    lidar_offset: float = 0.435


@dataclass
class ROIGridConfig:
    rows: int = 1
    columns: int = 1


@dataclass
class ROICustomConfig:
    regions: List[CustomROIRegion] = field(default_factory=list)


@dataclass
class ROIConfig:
    layout: str = "grid"
    grid: ROIGridConfig = field(default_factory=ROIGridConfig)
    geometry: ROIGeometryConfig = field(default_factory=ROIGeometryConfig)
    custom: ROICustomConfig = field(default_factory=ROICustomConfig)


# ==========================================================
# 5. CLUSTERING
# ==========================================================

@dataclass
class AlgorithmConfig:
    algorithm: ClusterAlgorithm


@dataclass
class DBSCANConfig:
    eps: float = 0.06
    min_samples: int = 3

@dataclass
class EuclideanConfig:
    cluster_tolerance: float
    min_cluster_size: int
    max_cluster_size: int

@dataclass
class ClusteringConfig:
    algorithm: AlgorithmConfig
    dbscan: DBSCANConfig = field(default_factory=DBSCANConfig)
    euclidean: EuclideanConfig = field(default_factory=EuclideanConfig)

# ==========================================================
# 6. CLUSTER VALIDATION
# ==========================================================

@dataclass
class ClusterValidationConfig:
    """
    Generic cluster usability validation.

    This stage does not decide whether a cluster is a pole.
    """
    min_points: int = 4


# ==========================================================
# 7. CLUSTER REFINEMENT
# ==========================================================

@dataclass
class KMeansConfig:
    enabled: bool = True
    max_iteration: int = 3
    n_clusters: int = 2
    random_state: int = 42


@dataclass
class RefinementConfig:
    kmeans: KMeansConfig = field(default_factory=KMeansConfig)


# ==========================================================
# 8. FEATURE EXTRACTION
# ==========================================================

@dataclass
class FeatureExtractionConfig:
    enabled: bool = True
    features: List[str] = field(
        default_factory=lambda: [
            "point_count",
            "centroid",
            "width",
            "height",
            "major_spread",
            "minor_spread",
            "aspect_ratio",
            "density",
            "yaw",
        ]
    )


# ==========================================================
# 9. POLE CANDIDATE DETECTION
# ==========================================================

@dataclass
class PoleWidthConfig:
    min: float = 0.005
    max: float = 0.10


@dataclass
class PoleHeightConfig:
    min: float = 0.04
    max: float = 0.16


@dataclass
class PoleAspectRatioConfig:
    min: float = 1.3


@dataclass
class PoleGeometryConfig:
    width: PoleWidthConfig = field(default_factory=PoleWidthConfig)
    height: PoleHeightConfig = field(default_factory=PoleHeightConfig)
    aspect_ratio: PoleAspectRatioConfig = field(
        default_factory=PoleAspectRatioConfig
    )


@dataclass
class PoleScoringConfig:
    min_candidate_score: float = 0.55


@dataclass
class PoleDetectionConfig:
    geometry: PoleGeometryConfig = field(
        default_factory=PoleGeometryConfig
    )
    scoring: PoleScoringConfig = field(
        default_factory=PoleScoringConfig
    )


# ==========================================================
# 10. PALLET VALIDATION
# ==========================================================

@dataclass
class PalletGeometryConfig:
    length: float = 1.40
    width: float = 0.80


@dataclass
class PoleRowConfig:
    expected: int


@dataclass
class PoleRowsConfig:
    row_a: PoleRowConfig = field(
        default_factory=lambda: PoleRowConfig(expected=4)
    )
    row_b: PoleRowConfig = field(
        default_factory=lambda: PoleRowConfig(expected=4)
    )


@dataclass
class LongitudinalSpacingConfig:
    expected: float = 0.46
    tolerance: float = 0.08


@dataclass
class PoleSpacingConfig:
    longitudinal: LongitudinalSpacingConfig = field(
        default_factory=LongitudinalSpacingConfig
    )


@dataclass
class PalletPolesConfig:
    expected: int = 8
    rows: PoleRowsConfig = field(default_factory=PoleRowsConfig)
    spacing: PoleSpacingConfig = field(
        default_factory=PoleSpacingConfig
    )


@dataclass
class PalletDetectionConfig:
    min_detected_poles: int = 6


@dataclass
class PalletConfig:
    type: str = "standard"
    geometry: PalletGeometryConfig = field(
        default_factory=PalletGeometryConfig
    )
    poles: PalletPolesConfig = field(
        default_factory=PalletPolesConfig
    )
    detection: PalletDetectionConfig = field(
        default_factory=PalletDetectionConfig
    )


# ==========================================================
# 11. OUTPUT
# ==========================================================

@dataclass
class OutputPublishConfig:
    point_cloud: bool = True
    rois: bool = True
    clusters: bool = True
    pole_candidates: bool = True
    pallet_result: bool = True


@dataclass
class OutputConfig:
    publish: OutputPublishConfig = field(
        default_factory=OutputPublishConfig
    )


# ==========================================================
# 12. VISUALIZATION
# ==========================================================

@dataclass
class VisualizationConfig:
    point_size: float = 0.03
    bbox_width: float = 0.02
    text_height: float = 0.15

@dataclass
class PoleCandidateWeightsConfig:
    point: float
    size: float
    aspect: float
    shape: float


@dataclass
class PoleCandidateQualityConfig:
    weak_factor: float
    suspicious_factor: float


@dataclass
class PoleCandidateConfig:

    min_points: int
    saturation_points: int
    min_width: float
    max_width: float

    min_height: float
    max_height: float

    min_aspect_ratio: float

    min_score: float

    weights: PoleCandidateWeightsConfig

    quality: PoleCandidateQualityConfig
# ==========================================================
# COMPLETE PPTS CONFIGURATION
# ==========================================================

@dataclass
class PPTSConfig:
    general: GeneralConfig
    input: InputConfig
    preprocessing: PreprocessingConfig
    roi: ROIConfig
    clustering: ClusteringConfig
    cluster_validation: ClusterValidationConfig
    refinement: RefinementConfig
    feature_extraction: FeatureExtractionConfig
    pole_detection: PoleDetectionConfig
    pallet: PalletConfig
    output: OutputConfig
    visualization: VisualizationConfig
    pole_candidate: PoleCandidateConfig