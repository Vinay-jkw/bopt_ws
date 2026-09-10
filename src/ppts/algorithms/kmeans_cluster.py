#!/usr/bin/env python3

import numpy as np
from sklearn.cluster import KMeans

from algorithms.base_cluster_algorithm import BaseClusterAlgorithm
from models.cluster import Cluster
from models.point_cloud import PointCloud


class KMeansCluster(BaseClusterAlgorithm):
    """
    K-Means clustering.

    This algorithm is primarily intended for refinement/splitting
    of a suspicious DBSCAN cluster.
    """

    def __init__(
        self,
        n_clusters: int = 2,
        random_state: int = 42,
    ):
        if n_clusters < 2:
            raise ValueError(
                "KMeans refinement requires at least 2 clusters"
            )

        self.n_clusters = n_clusters
        self.random_state = random_state

    def detect(
        self,
        cloud: PointCloud,
    ) -> list[Cluster]:

        if cloud is None or cloud.is_empty:
            return []

        if cloud.size < self.n_clusters:
            return []

        # --------------------------------------------------
        # KMeans
        # --------------------------------------------------

        model = KMeans(
            n_clusters=self.n_clusters,
            random_state=self.random_state,
            n_init=10,
        )

        # Use only X/Y for 2D pallet perception
        points = cloud.points[:, :2]

        labels = model.fit_predict(points)

        # --------------------------------------------------
        # Create clusters
        # --------------------------------------------------

        clusters: list[Cluster] = []

        for cluster_id in range(self.n_clusters):

            cluster_points = cloud.points[
                labels == cluster_id
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

        return clusters