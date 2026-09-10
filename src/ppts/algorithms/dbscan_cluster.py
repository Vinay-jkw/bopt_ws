#!/usr/bin/env python3

import numpy as np
from sklearn.cluster import DBSCAN

from algorithms.base_cluster_algorithm import BaseClusterAlgorithm
from models.cluster import Cluster
from models.point_cloud import PointCloud


class DBSCANCluster(BaseClusterAlgorithm):
    """
    DBSCAN-based spatial clustering.

    DBSCAN is used as the primary clustering algorithm for
    pallet perception.

    Noise points are ignored.
    """

    def __init__(
        self,
        eps: float = 0.1,
        min_samples: int = 8,
    ):
        if eps <= 0:
            raise ValueError(
                "DBSCAN eps must be > 0"
            )

        if min_samples < 1:
            raise ValueError(
                "DBSCAN min_samples must be >= 1"
            )

        self.eps = eps
        self.min_samples = min_samples

    def detect(
        self,
        cloud: PointCloud,
    ) -> list[Cluster]:

        if cloud is None or cloud.is_empty:
            return []

        model = DBSCAN(
            eps=self.eps,
            min_samples=self.min_samples,
        )

        labels = model.fit_predict(cloud.points)

        clusters: list[Cluster] = []

        cluster_id = 0

        for label in np.unique(labels):

            # DBSCAN uses -1 for noise.
            if label == -1:
                continue

            cluster_points = cloud.points[
                labels == label
            ]

            if cluster_points.size == 0:
                continue

            cluster_cloud = cloud.copy_with_points(
                cluster_points
            )

            clusters.append(
                Cluster(
                    id=cluster_id,
                    cloud=cluster_cloud,
                )
            )

            cluster_id += 1

        return clusters