#!/usr/bin/env python3

from dataclasses import dataclass, field
from enum import Enum

import numpy as np
from models.enums import ClusterQuality, ClusterLabel
from models.point_cloud import PointCloud
from models.roi import ROI





@dataclass
class Cluster:
    """
    Represents a single spatial cluster detected from a point cloud.

    The clustering algorithm is responsible only for creating:

        id
        cloud

    Geometric features are populated later by ClusterFeatureProcessor.

    Semantic classification is performed by higher-level perception
    modules.
    """

    # ----------------------------------------------------------
    # Identification
    # ----------------------------------------------------------

    id: int

    # ----------------------------------------------------------
    # Raw Data
    # ----------------------------------------------------------

    cloud: PointCloud

    # ----------------------------------------------------------
    # Geometric Features
    # ----------------------------------------------------------

    centroid: np.ndarray = field(
        default_factory=lambda: np.zeros(
            2,
            dtype=np.float32,
        )
    )

    width: float = 0.0

    height: float = 0.0

    yaw: float = 0.0

    density: float = 0.0

    major_spread: float = 0.0

    minor_spread: float = 0.0
    # ----------------------------------------------------------
    # Classification
    # ----------------------------------------------------------
    quality: ClusterQuality = ClusterQuality.REJECTED

    label: ClusterLabel = ClusterLabel.UNKNOWN

    # ----------------------------------------------------------
    # Convenience
    # ----------------------------------------------------------

    @property
    def point_count(self) -> int:
        """
        Number of points in the cluster.
        """
        return self.cloud.size

@dataclass
class ROICluster:

    roi: ROI

    cloud: PointCloud | None = None

    clusters: list[Cluster] = field(
        default_factory=list
    )