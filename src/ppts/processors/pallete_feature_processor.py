#!/usr/bin/env python3

import numpy as np

from models.pallete_feature_set import PalletFeatureSet
from processors.base_processor import BaseProcessor
from models.pole_candidate import PoleCandidate

class PalletFeatureProcessor(BaseProcessor):
    """
    Calculates pallet-level geometric features from accepted
    pole candidates.

    Responsibilities
    ----------------
    - Select accepted pole candidates
    - Extract their underlying clusters
    - Separate clusters into two pallet rows
    - Order each row along the X axis
    - Calculate pole-to-pole spacing
    - Calculate row-to-row spacing
    - Calculate overall pallet geometry

    This processor does NOT determine whether a pallet exists.

    Pole classification is handled by:
        PoleCandidateProcessor

    Pallet detection is handled later by:
        PalletDetector
    """

    def __init__(
        self,
        input_key: PoleCandidate = "pole_candidates",
        output_key: PalletFeatureSet = "pallet_features",
        row_tolerance: float = 0.60,
    ):
        self.input_key = input_key
        self.output_key = output_key
        self.row_tolerance = row_tolerance

    # ======================================================
    # PROCESS
    # ======================================================

    def process(self, context) -> None:
        """
        Process accepted pole candidates and calculate
        pallet-level geometric features.
        """

        pole_candidates = getattr(
            context,
            self.input_key,
            [],
        )
        features = self._calculate_features(
            pole_candidates
        )

        setattr(
            context,
            self.output_key,
            features,
        )

    # ======================================================
    # MAIN FEATURE CALCULATION
    # ======================================================

    def _calculate_features(
        self,
        pole_candidates,
    ) -> PalletFeatureSet:
        """
        Convert accepted PoleCandidates into clusters and
        calculate pallet-level geometry.
        """

        candidates = self._select_candidates(
            pole_candidates
        )

        if not candidates:
            return PalletFeatureSet()

        # --------------------------------------------------
        # Split candidates into two rows
        # --------------------------------------------------

        row_a, row_b = self._split_rows(
            candidates
        )

        # --------------------------------------------------
        # Sort each row along X
        # --------------------------------------------------

        row_a = self._sort_by_x(
            row_a
        )

        row_b = self._sort_by_x(
            row_b
        )

        # --------------------------------------------------
        # Calculate longitudinal gaps
        # --------------------------------------------------

        row_a_gaps = self._calculate_x_gaps(
            row_a
        )

        row_b_gaps = self._calculate_x_gaps(
            row_b
        )

        # --------------------------------------------------
        # Calculate average row spacing
        # --------------------------------------------------

        row_a_spacing = (
            self._calculate_average_spacing(
                row_a_gaps
            )
        )

        row_b_spacing = (
            self._calculate_average_spacing(
                row_b_gaps
            )
        )

        # --------------------------------------------------
        # Calculate distance between rows
        # --------------------------------------------------

        row_spacing = (
            self._calculate_row_spacing(
                row_a,
                row_b,
            )
        )

        # --------------------------------------------------
        # Calculate overall pallet extent
        # --------------------------------------------------

        min_x, max_x, min_y, max_y = (
            self._calculate_extent(
                candidates
            )
        )

        # --------------------------------------------------
        # Calculate overall centroid
        # --------------------------------------------------

        centroid = self._calculate_centroid(
            candidates
        )

        # --------------------------------------------------
        # Build feature set
        # --------------------------------------------------

        return PalletFeatureSet(
            candidates=candidates,

            row_a=row_a,
            row_b=row_b,

            row_a_gaps=row_a_gaps,
            row_b_gaps=row_b_gaps,

            row_a_spacing=row_a_spacing,
            row_b_spacing=row_b_spacing,

            row_spacing=row_spacing,

            centroid=centroid,

            min_x=min_x,
            max_x=max_x,
            min_y=min_y,
            max_y=max_y,
        )

    # ======================================================
    # CANDIDATE SELECTION
    # ======================================================

    @staticmethod
    def _select_candidates(
        pole_candidates,
    ):
        """
        Extract Cluster objects from accepted PoleCandidates.

        PoleCandidateProcessor decides whether a cluster is
        a pole candidate.

        PalletFeatureProcessor only consumes candidates that
        have already been accepted.
        """

        candidates = []

        for candidate in pole_candidates:

            if candidate is None:
                continue

            # ----------------------------------------------
            # Only accepted pole candidates
            # ----------------------------------------------

            if not candidate.accepted:
                continue

            # ----------------------------------------------
            # Get underlying cluster
            # ----------------------------------------------

            cluster = candidate.cluster

            if cluster is None:
                continue

            # ----------------------------------------------
            # Validate cluster data
            # ----------------------------------------------

            if cluster.cloud is None:
                continue

            if cluster.cloud.is_empty:
                continue

            candidates.append(
                cluster
            )

        return candidates

    def _estimate_pallet_orientation(
        self,
        clusters,
    ):
        """
        Estimate pallet longitudinal orientation from
        nearest-neighbor cluster directions.

        Assumption:
            Distance between poles in the same row is
            significantly smaller than the distance between
            the two pallet rows.

        The nearest-neighbor vectors therefore provide a
        good estimate of the pallet longitudinal direction.
        """

        if len(clusters) < 2:
            return 0.0

        points = np.array(
            [
                cluster.centroid[:2]
                for cluster in clusters
            ],
            dtype=np.float32,
        )

        angles = []

        # --------------------------------------------------
        # Find nearest-neighbor direction for each cluster
        # --------------------------------------------------

        for i, point in enumerate(points):

            distances = np.linalg.norm(
                points - point,
                axis=1,
            )

            # Ignore itself
            distances[i] = np.inf

            nearest_index = int(
                np.argmin(distances)
            )

            nearest_point = points[
                nearest_index
            ]

            dx = float(
                nearest_point[0] - point[0]
            )

            dy = float(
                nearest_point[1] - point[1]
            )

            # Ignore degenerate pairs
            if np.hypot(dx, dy) < 1e-6:
                continue

            angle = np.arctan2(
                dy,
                dx,
            )

            # Direction of a row is an axis, not
            # a one-way direction.
            #
            # Therefore:
            #     angle
            # and
            #     angle + pi
            #
            # represent the same pallet orientation.
            angles.append(angle)

        if not angles:
            return 0.0

        # --------------------------------------------------
        # Circular mean with 180-degree ambiguity removed
        # --------------------------------------------------

        angles = np.asarray(
            angles,
            dtype=np.float64,
        )

        mean_angle = 0.5 * np.arctan2(
            np.mean(
                np.sin(2.0 * angles)
            ),
            np.mean(
                np.cos(2.0 * angles)
            ),
        )

        return float(mean_angle)


    # ======================================================
    # SPLIT INTO TWO ROWS
    # ======================================================

    def _split_rows(
        self,
        clusters,
    ):
        """
        Separate clusters into two pallet rows.

        The pallet orientation is first estimated from
        nearest-neighbor cluster directions.

        Cluster centroids are then transformed into the
        pallet coordinate frame:

            local X -> along pallet rows
            local Y -> across pallet rows

        The largest gap in local Y is used to separate
        the two rows.
        """

        if len(clusters) < 2:
            return clusters, []

        # --------------------------------------------------
        # Estimate pallet orientation
        # --------------------------------------------------

        angle = self._estimate_pallet_orientation(
            clusters
        )

        cos_angle = np.cos(angle)
        sin_angle = np.sin(angle)

        # --------------------------------------------------
        # Transform clusters into pallet coordinates
        # --------------------------------------------------

        local_clusters = []

        for cluster in clusters:

            x = float(
                cluster.centroid[0]
            )

            y = float(
                cluster.centroid[1]
            )

            # Rotate global coordinates into pallet frame.
            local_x = (
                cos_angle * x
                + sin_angle * y
            )

            local_y = (
                -sin_angle * x
                + cos_angle * y
            )

            local_clusters.append(
                (
                    cluster,
                    local_x,
                    local_y,
                )
            )

        # --------------------------------------------------
        # Sort by local Y
        # --------------------------------------------------

        local_clusters.sort(
            key=lambda item: item[2]
        )

        # --------------------------------------------------
        # Extract local Y values
        # --------------------------------------------------

        local_y_values = np.array(
            [
                local_y
                for _, _, local_y
                in local_clusters
            ],
            dtype=np.float32,
        )

        # --------------------------------------------------
        # Find largest gap between the two rows
        # --------------------------------------------------

        gaps = np.diff(
            local_y_values
        )

        if len(gaps) == 0:
            return clusters, []

        split_index = int(
            np.argmax(gaps)
        )

        # --------------------------------------------------
        # Split into rows
        # --------------------------------------------------

        row_a_data = local_clusters[
            :split_index + 1
        ]

        row_b_data = local_clusters[
            split_index + 1:
        ]

        row_a = [
            cluster
            for cluster, _, _
            in row_a_data
        ]

        row_b = [
            cluster
            for cluster, _, _
            in row_b_data
        ]

        # --------------------------------------------------
        # Safety check
        # --------------------------------------------------

        if not row_a or not row_b:
            return clusters, []

        # --------------------------------------------------
        # Calculate row centers in local Y
        # --------------------------------------------------

        row_a_y = float(
            np.mean(
                [
                    local_y
                    for _, _, local_y
                    in row_a_data
                ]
            )
        )

        row_b_y = float(
            np.mean(
                [
                    local_y
                    for _, _, local_y
                    in row_b_data
                ]
            )
        )

        row_separation = abs(
            row_b_y - row_a_y
        )

        # --------------------------------------------------
        # Validate row separation
        # --------------------------------------------------

        if row_separation < self.row_tolerance:
            return clusters, []

        return row_a, row_b

    # ======================================================
    # SORT ROW BY X
    # ======================================================

    @staticmethod
    def _sort_by_x(
        clusters,
    ):
        """
        Sort clusters from nearest/leftmost to
        farthest/rightmost along X.
        """

        return sorted(
            clusters,
            key=lambda cluster: float(
                cluster.centroid[0]
            ),
        )

    # ======================================================
    # X GAPS
    # ======================================================

    @staticmethod
    def _calculate_x_gaps(
        clusters,
    ) -> list[float]:
        """
        Calculate longitudinal distance between
        consecutive poles.
        """

        if len(clusters) < 2:
            return []

        gaps = []

        for current, next_cluster in zip(
            clusters,
            clusters[1:],
        ):

            gap = abs(
                float(
                    next_cluster.centroid[0]
                    - current.centroid[0]
                )
            )

            gaps.append(
                gap
            )

        return gaps

    # ======================================================
    # AVERAGE SPACING
    # ======================================================

    @staticmethod
    def _calculate_average_spacing(
        gaps,
    ) -> float:
        """
        Calculate average longitudinal pole spacing.
        """

        if not gaps:
            return 0.0

        return float(
            np.mean(gaps)
        )

    # ======================================================
    # ROW-TO-ROW SPACING
    # ======================================================

    @staticmethod
    def _calculate_row_spacing(
        row_a,
        row_b,
    ) -> float:
        """
        Calculate average Y distance between the two rows.
        """

        if not row_a or not row_b:
            return 0.0

        y_a = float(
            np.mean(
                [
                    cluster.centroid[1]
                    for cluster in row_a
                ]
            )
        )

        y_b = float(
            np.mean(
                [
                    cluster.centroid[1]
                    for cluster in row_b
                ]
            )
        )

        return abs(
            y_a - y_b
        )

    # ======================================================
    # OVERALL EXTENT
    # ======================================================

    @staticmethod
    def _calculate_extent(
        clusters,
    ):
        """
        Calculate bounding extent of the detected
        pole centroids.
        """

        if not clusters:
            return (
                0.0,
                0.0,
                0.0,
                0.0,
            )

        points = np.array(
            [
                cluster.centroid[:2]
                for cluster in clusters
            ],
            dtype=np.float32,
        )

        return (
            float(
                np.min(
                    points[:, 0]
                )
            ),
            float(
                np.max(
                    points[:, 0]
                )
            ),
            float(
                np.min(
                    points[:, 1]
                )
            ),
            float(
                np.max(
                    points[:, 1]
                )
            ),
        )

    # ======================================================
    # OVERALL CENTROID
    # ======================================================

    @staticmethod
    def _calculate_centroid(
        clusters,
    ):
        """
        Calculate centroid of all accepted pole clusters.
        """

        if not clusters:
            return np.zeros(
                2,
                dtype=np.float32,
            )

        points = np.array(
            [
                cluster.centroid[:2]
                for cluster in clusters
            ],
            dtype=np.float32,
        )

        return np.mean(
            points,
            axis=0,
        ).astype(
            np.float32
        )