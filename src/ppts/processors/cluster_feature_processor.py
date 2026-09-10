#!/usr/bin/env python3

import numpy as np

from models.cluster import Cluster
from processors.base_processor import BaseProcessor


class ClusterFeatureProcessor(BaseProcessor):
    """
    Calculates geometric features for detected clusters.

    Features
    --------
    - centroid
    - width
    - height
    - yaw
    - density
    - major_spread
    - minor_spread

    Features are calculated using the XY plane.

    This processor operates on:

        context.roi_clusters
            └── ROICluster
                  └── clusters
                        ├── Cluster
                        ├── Cluster
                        └── ...

    The processor is intentionally independent of the clustering
    algorithm. Therefore it can be executed both:

        1. After initial DBSCAN clustering
        2. After K-Means refinement
    """

    # ==========================================================
    # Public API
    # ==========================================================

    def process(self, context) -> None:
        """
        Calculate features for every cluster in every ROI.

        Existing feature values are overwritten so that newly
        generated K-Means clusters always receive fresh features.
        """

        roi_clusters = getattr(
            context,
            "roi_clusters",
            None,
        )

        if not roi_clusters:
            return

        for roi_cluster in roi_clusters:

            if roi_cluster is None:
                continue

            clusters = getattr(
                roi_cluster,
                "clusters",
                None,
            )

            if not clusters:
                continue

            for cluster in clusters:

                if cluster is None:
                    continue

                if cluster.cloud is None:
                    continue

                if cluster.cloud.is_empty:
                    continue

                self._calculate_features(
                    cluster
                )

    # ==========================================================
    # Feature calculation
    # ==========================================================

    def _calculate_features(
        self,
        cluster: Cluster,
    ) -> None:
        """
        Calculate and update all geometric features for one
        cluster.
        """

        points = np.asarray(
            cluster.cloud.points[:, :2],
            dtype=np.float64,
        )

        if points.size == 0:
            return

        # ------------------------------------------------------
        # Centroid
        # ------------------------------------------------------

        cluster.centroid = np.mean(
            points,
            axis=0,
        ).astype(np.float32)

        # ------------------------------------------------------
        # Axis-aligned dimensions
        # ------------------------------------------------------

        min_point = np.min(
            points,
            axis=0,
        )

        max_point = np.max(
            points,
            axis=0,
        )

        dimensions = (
            max_point - min_point
        )

        cluster.width = float(
            dimensions[0]
        )

        cluster.height = float(
            dimensions[1]
        )

        # ------------------------------------------------------
        # Orientation
        # ------------------------------------------------------

        cluster.yaw = self._calculate_yaw(
            points
        )

        # ------------------------------------------------------
        # Density
        # ------------------------------------------------------

        cluster.density = (
            self._calculate_density(
                points,
                dimensions,
            )
        )

        # ------------------------------------------------------
        # PCA spread
        # ------------------------------------------------------

        (
            cluster.major_spread,
            cluster.minor_spread,
        ) = self._calculate_pca_spread(
            points
        )

    # ==========================================================
    # Yaw
    # ==========================================================

    @staticmethod
    def _calculate_yaw(
        points: np.ndarray,
    ) -> float:
        """
        Calculate the principal orientation of the cluster.

        Returns yaw normalized to:

            [-pi/2, pi/2)
        """

        if len(points) < 2:
            return 0.0

        centered = (
            points
            - np.mean(
                points,
                axis=0,
            )
        )

        covariance = np.cov(
            centered,
            rowvar=False,
        )

        if covariance.shape != (2, 2):
            return 0.0

        # Protect against invalid covariance values.
        if not np.all(
            np.isfinite(covariance)
        ):
            return 0.0

        eigenvalues, eigenvectors = (
            np.linalg.eigh(covariance)
        )

        principal_axis = eigenvectors[
            :,
            np.argmax(eigenvalues),
        ]

        yaw = np.arctan2(
            principal_axis[1],
            principal_axis[0],
        )

        # Normalize to [-pi/2, pi/2)
        yaw = (
            (yaw + np.pi / 2)
            % np.pi
        ) - np.pi / 2

        return float(yaw)

    # ==========================================================
    # Density
    # ==========================================================

    @staticmethod
    def _calculate_density(
        points: np.ndarray,
        dimensions: np.ndarray,
    ) -> float:
        """
        Calculate point density using the axis-aligned
        bounding-box area.

        density = number_of_points / area
        """

        area = (
            float(dimensions[0])
            * float(dimensions[1])
        )

        if area <= 0.0:
            return 0.0

        return float(
            len(points) / area
        )

    # ==========================================================
    # PCA spread
    # ==========================================================

    @staticmethod
    def _calculate_pca_spread(
        points: np.ndarray,
    ) -> tuple[float, float]:
        """
        Calculate the standard deviation along the principal
        and secondary PCA axes.

        Returns:

            major_spread
            minor_spread
        """

        if len(points) < 2:
            return 0.0, 0.0

        centered = (
            points
            - np.mean(
                points,
                axis=0,
            )
        )

        covariance = np.cov(
            centered,
            rowvar=False,
        )

        if covariance.shape != (2, 2):
            return 0.0, 0.0

        if not np.all(
            np.isfinite(covariance)
        ):
            return 0.0, 0.0

        eigenvalues, _ = np.linalg.eigh(
            covariance
        )

        eigenvalues = np.sort(
            eigenvalues
        )[::-1]

        major_spread = float(
            np.sqrt(
                max(
                    eigenvalues[0],
                    0.0,
                )
            )
        )

        minor_spread = float(
            np.sqrt(
                max(
                    eigenvalues[1],
                    0.0,
                )
            )
        )

        return (
            major_spread,
            minor_spread,
        )