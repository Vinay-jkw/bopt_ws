#!/usr/bin/env python3

import numpy as np
from sklearn.neighbors import NearestNeighbors

from algorithms.base_cluster_algorithm import BaseClusterAlgorithm
from models.cluster import Cluster
from models.point_cloud import PointCloud


class EuclideanCluster(BaseClusterAlgorithm):
    """
    Euclidean spatial clustering.

    Points are grouped together when they are spatially connected
    through neighboring points within `cluster_tolerance`.

    Unlike DBSCAN, this algorithm does not use density-based
    core/border/noise classification. It performs region growing
    based purely on Euclidean distance.

    Parameters
    ----------
    cluster_tolerance : float
        Maximum Euclidean distance between neighboring points
        for them to be considered connected.

    min_cluster_size : int
        Minimum number of points required for a cluster to be
        considered valid.

    max_cluster_size : int | None
        Maximum number of points allowed in a cluster.
        None means no maximum.
    """

    def __init__(
        self,
        cluster_tolerance: float = 0.1,
        min_cluster_size: int = 8,
        max_cluster_size: int | None = None,
    ):
        if cluster_tolerance <= 0:
            raise ValueError(
                "Euclidean cluster_tolerance must be > 0"
            )

        if min_cluster_size < 1:
            raise ValueError(
                "Euclidean min_cluster_size must be >= 1"
            )

        if (
            max_cluster_size is not None
            and max_cluster_size < min_cluster_size
        ):
            raise ValueError(
                "max_cluster_size must be >= min_cluster_size"
            )

        self.cluster_tolerance = cluster_tolerance
        self.min_cluster_size = min_cluster_size
        self.max_cluster_size = max_cluster_size

    def detect(
        self,
        cloud: PointCloud,
    ) -> list[Cluster]:

        if cloud is None or cloud.is_empty:
            return []

        points = cloud.points

        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(
                "PointCloud points must have shape (N, 3)"
            )

        if len(points) == 0:
            return []

        # ------------------------------------------------------
        # Build spatial search structure
        # ------------------------------------------------------

        neighbor_search = NearestNeighbors(
            radius=self.cluster_tolerance,
            algorithm="kd_tree",
        )

        neighbor_search.fit(points)

        # ------------------------------------------------------
        # Find neighboring points for every point
        # ------------------------------------------------------

        neighbors = neighbor_search.radius_neighbors(
            points,
            return_distance=False,
        )

        # ------------------------------------------------------
        # Euclidean region growing
        # ------------------------------------------------------

        visited = np.zeros(
            len(points),
            dtype=bool,
        )

        clusters: list[Cluster] = []

        cluster_id = 0

        for seed_index in range(len(points)):

            if visited[seed_index]:
                continue

            # Start a new region
            queue = [seed_index]
            visited[seed_index] = True

            cluster_indices = []

            while queue:

                current_index = queue.pop()

                cluster_indices.append(
                    current_index
                )

                # Expand the current region
                for neighbor_index in neighbors[current_index]:

                    if visited[neighbor_index]:
                        continue

                    visited[neighbor_index] = True
                    queue.append(neighbor_index)

            cluster_size = len(cluster_indices)

            # --------------------------------------------------
            # Cluster size validation
            # --------------------------------------------------

            if cluster_size < self.min_cluster_size:
                continue

            if (
                self.max_cluster_size is not None
                and cluster_size > self.max_cluster_size
            ):
                continue

            # --------------------------------------------------
            # Extract cluster points
            # --------------------------------------------------

            cluster_points = points[
                cluster_indices
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