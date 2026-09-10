from dataclasses import dataclass, field
from typing import Optional, List

from sensor_msgs.msg import LaserScan

from models.point_cloud import PointCloud, ROICloud
from models.cluster import Cluster, ROICluster
from models.roi import ROI
from models.pallete_perception import PalletDetectionResult
from models.pallete_feature_set import PalletFeatureSet
from models.pole_candidate import PoleCandidate

@dataclass
class PPTSContext:
    """
    Shared context passed through the entire PPTS pipeline.
    """

    # ----------------------------
    # Raw Sensor Data
    # ----------------------------
    left_scan: Optional[LaserScan] = None
    right_scan: Optional[LaserScan] = None

    # ----------------------------
    # Geometry Layer
    # ----------------------------
    left_cloud: Optional[PointCloud] = None
    right_cloud: Optional[PointCloud] = None
    merged_cloud: Optional[PointCloud] = None

    # ----------------------------
    # ROI Layer
    # ----------------------------
    rois: list[ROI] = field(default_factory=list)

    roi_clouds: list[ROICloud] = field(default_factory=list)

    # ----------------------------
    # Clustering Layer
    # ----------------------------
    

    clusters: List[Cluster] = field(default_factory=list)
    roi_clusters: list[ROICluster] = field(default_factory=list)

    # ----------------------------
    # Decision Layer
    # ----------------------------

    pole_candidates: list[PoleCandidate] = field(
        default_factory=list
    )
    pallet_features: PalletFeatureSet = field(
            default_factory=PalletFeatureSet
        )
    pallet_detection: PalletDetectionResult = field(
            default_factory=PalletDetectionResult
    )

    # ----------------------------
    # Tracking
    # ----------------------------
    # confidence: float = 0.0
    # tracking_state: str = "IDLE"