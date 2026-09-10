#!/usr/bin/env python3

from models.cluster import (
    Cluster,
    ClusterQuality,
)

from processors.base_processor import BaseProcessor


class ClusterValidationProcessor(BaseProcessor):
    """
    Validates clusters using their geometric features.

    This processor does not remove clusters.

    It assigns a quality:

        VALID
            Normal cluster.

        WEAK
            Sparse / partially visible cluster.

        SUSPICIOUS
            Possible merged cluster that should be refined.

    KMeans refinement is handled separately by
    ClusterRefinementProcessor.

    Expected context structure:

        context.roi_clusters
            └── ROICluster
                  └── clusters
                        ├── Cluster
                        ├── Cluster
                        └── ...
    """

    def __init__(
        self,
        min_valid_points: int = 4,
        max_width: float = 0.10,
        max_height: float = 0.16,
        max_major_spread: float = 0.06,
    ):
        self.min_valid_points = min_valid_points
        self.max_width = max_width
        self.max_height = max_height
        self.max_major_spread = max_major_spread

    # ==========================================================
    # Public API
    # ==========================================================

    def process(self, context) -> None:
        """
        Validate every cluster in every ROI.

        This method is intentionally safe to call multiple times.

        That is important for the refinement loop:

            KMeans
                ↓
            FeatureProcessor
                ↓
            ValidationProcessor
                ↓
            suspicious?
                ↓
            KMeans again

        Every call recalculates the quality of the current
        clusters.
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

                cluster.quality = self._validate(
                    cluster
                )

    # ==========================================================
    # Validation
    # ==========================================================

    def _validate(
        self,
        cluster: Cluster,
    ) -> ClusterQuality:
        """
        Determine the quality of one cluster.
        """

        # ------------------------------------------------------
        # Sparse / partial cluster
        # ------------------------------------------------------

        if (
            cluster.point_count
            < self.min_valid_points
        ):
            return ClusterQuality.WEAK

        # ------------------------------------------------------
        # Possible merged cluster
        # ------------------------------------------------------

        if self._looks_merged(cluster):
            return ClusterQuality.SUSPICIOUS

        # ------------------------------------------------------
        # Normal cluster
        # ------------------------------------------------------

        return ClusterQuality.VALID

    # ==========================================================
    # Merged-cluster detection
    # ==========================================================

    def _looks_merged(
        self,
        cluster: Cluster,
    ) -> bool:
        """
        Determine whether a cluster looks like multiple objects
        merged into one cluster.

        Current criteria:

            width
            height
            major PCA spread
        """

        if (
            cluster.width
            > self.max_width
        ):
            return True

        if (
            cluster.height
            > self.max_height
        ):
            return True

        if (
            cluster.major_spread
            > self.max_major_spread
        ):
            return True

        return False