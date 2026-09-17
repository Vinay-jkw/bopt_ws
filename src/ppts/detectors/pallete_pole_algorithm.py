#!/usr/bin/env python3

import numpy as np

from detectors.base_detector_algorithm import BaseDetectorAlgorithm
from models.pallete_pole_input import PalletPoleInput
from models.pallete_feature_set import PalletFeatureSet


class PalletPoleAlgorithm(
    BaseDetectorAlgorithm[
        PalletPoleInput,
        PalletPoleInput,
    ]
):

    def __init__(
        self,
        line_tolerance: float = 0.10,
    ):
        self.line_tolerance = line_tolerance

    # ======================================================
    # DETECT
    # ======================================================

    def detect(
        self,
        data: PalletPoleInput,
    ) -> PalletPoleInput:

        if data is None:
            return PalletPoleInput()

        candidates = data.candidates
        features = data.features

        # --------------------------------------------------
        # Reset pallet-pole classification
        # --------------------------------------------------

        for candidate in candidates:
            candidate.pallet_pole = False

        # --------------------------------------------------
        # Classify rows
        # --------------------------------------------------

        row_a = self._classify_row(
            features.row_a,
            candidates,
            "A",
        )

        row_b = self._classify_row(
            features.row_b,
            candidates,
            "B",
        )

        # --------------------------------------------------
        # Create UPDATED feature set
        #
        # Only validated pallet poles remain in the rows.
        # --------------------------------------------------

        updated_features = self._build_feature_set(
            row_a,
            row_b,
        )

        # --------------------------------------------------
        # Update the input object
        # --------------------------------------------------

        data.features = updated_features

        return data

    # ======================================================
    # CLASSIFY ROW
    # ======================================================

    def _classify_row(
        self,
        row,
        candidates,
        row_name,
    ):

        if not row:
            return []

        # --------------------------------------------------
        # One or two points
        # --------------------------------------------------

        if len(row) <= 2:

            validated = []

            for cluster in row:

                candidate = self._find_candidate(
                    cluster,
                    candidates,
                )

                if candidate is None:
                    continue

                candidate.pallet_pole = True
                validated.append(cluster)

                # print(
                #     f"PalletPole | "
                #     f"row={row_name} | "
                #     f"cluster={cluster.id} | "
                #     f"line_distance=0.000 | "
                #     f"tolerance={self.line_tolerance:.3f} | "
                #     f"pallet_pole=True"
                # )

            return validated

        # --------------------------------------------------
        # Get points
        # --------------------------------------------------

        points = np.array(
            [
                [
                    float(cluster.centroid[0]),
                    float(cluster.centroid[1]),
                ]
                for cluster in row
            ],
            dtype=np.float64,
        )

        # --------------------------------------------------
        # Find best straight-line subset
        # --------------------------------------------------

        best_inliers = []
        best_line = None

        for i in range(len(row)):

            for j in range(i + 1, len(row)):

                p1 = points[i]
                p2 = points[j]

                direction = p2 - p1

                length = np.linalg.norm(
                    direction
                )

                if length < 1e-6:
                    continue

                direction = direction / length

                inliers = []

                for index, point in enumerate(points):

                    vector = point - p1

                    projection = (
                        np.dot(vector, direction)
                        * direction
                    )

                    perpendicular = (
                        vector - projection
                    )

                    distance = np.linalg.norm(
                        perpendicular
                    )

                    if distance <= self.line_tolerance:
                        inliers.append(index)

                # --------------------------------------------------
                # Keep the line containing the most candidates
                # --------------------------------------------------

                if len(inliers) > len(best_inliers):

                    best_inliers = inliers

                    best_line = (
                        p1,
                        direction,
                    )

        # --------------------------------------------------
        # No valid line
        # --------------------------------------------------

        if best_line is None:
            return []

        # --------------------------------------------------
        # Classify against best line
        # --------------------------------------------------

        validated = []

        line_point, direction = best_line

        for index, cluster in enumerate(row):

            point = points[index]

            vector = point - line_point

            projection = (
                np.dot(vector, direction)
                * direction
            )

            perpendicular = (
                vector - projection
            )

            line_distance = float(
                np.linalg.norm(perpendicular)
            )

            candidate = self._find_candidate(
                cluster,
                candidates,
            )

            if candidate is None:
                continue

            is_pallet_pole = (
                line_distance
                <= self.line_tolerance
                and index in best_inliers
            )

            candidate.pallet_pole = (
                is_pallet_pole
            )

            if is_pallet_pole:
                validated.append(cluster)

            # print(
            #     f"PalletPole | "
            #     f"row={row_name} | "
            #     f"cluster={cluster.id} | "
            #     f"line_distance={line_distance:.3f} | "
            #     f"tolerance={self.line_tolerance:.3f} | "
            #     f"pallet_pole={is_pallet_pole}"
            # )

        return validated

    # ======================================================
    # BUILD UPDATED FEATURE SET
    # ======================================================

    def _build_feature_set(
        self,
        row_a,
        row_b,
    ):

        all_candidates = (
            row_a + row_b
        )

        features = PalletFeatureSet(
            candidates=all_candidates,
            row_a=row_a,
            row_b=row_b,
        )

        # --------------------------------------------------
        # Row A spacing
        # --------------------------------------------------

        features.row_a_gaps = (
            self._calculate_gaps(row_a)
        )

        if features.row_a_gaps:
            features.row_a_spacing = float(
                np.mean(
                    features.row_a_gaps
                )
            )

        # --------------------------------------------------
        # Row B spacing
        # --------------------------------------------------

        features.row_b_gaps = (
            self._calculate_gaps(row_b)
        )

        if features.row_b_gaps:
            features.row_b_spacing = float(
                np.mean(
                    features.row_b_gaps
                )
            )

        # --------------------------------------------------
        # Row-to-row spacing
        # --------------------------------------------------

        features.row_spacing = (
            self._calculate_row_spacing(
                row_a,
                row_b,
            )
        )

        # --------------------------------------------------
        # Overall geometry
        # --------------------------------------------------

        self._calculate_geometry(
            features
        )

        return features

    # ======================================================
    # CALCULATE GAPS
    # ======================================================

    @staticmethod
    def _calculate_gaps(row):

        if len(row) < 2:
            return []

        gaps = []

        for i in range(len(row) - 1):

            p1 = np.asarray(
                row[i].centroid,
                dtype=np.float64,
            )

            p2 = np.asarray(
                row[i + 1].centroid,
                dtype=np.float64,
            )

            distance = np.linalg.norm(
                p2 - p1
            )

            gaps.append(float(distance))

        return gaps

    # ======================================================
    # ROW SPACING
    # ======================================================

    @staticmethod
    def _calculate_row_spacing(
        row_a,
        row_b,
    ):

        if not row_a or not row_b:
            return 0.0

        distances = []

        for cluster_a in row_a:

            point_a = np.asarray(
                cluster_a.centroid,
                dtype=np.float64,
            )

            for cluster_b in row_b:

                point_b = np.asarray(
                    cluster_b.centroid,
                    dtype=np.float64,
                )

                distances.append(
                    float(
                        np.linalg.norm(
                            point_a - point_b
                        )
                    )
                )

        if not distances:
            return 0.0

        return float(
            np.median(distances)
        )

    # ======================================================
    # GEOMETRY
    # ======================================================

    @staticmethod
    def _calculate_geometry(features):

        clusters = (
            features.row_a
            + features.row_b
        )

        if not clusters:
            return

        points = np.asarray(
            [
                cluster.centroid
                for cluster in clusters
            ],
            dtype=np.float64,
        )

        features.centroid = (
            np.mean(
                points,
                axis=0,
            ).astype(np.float32)
        )

        features.min_x = float(
            np.min(points[:, 0])
        )

        features.max_x = float(
            np.max(points[:, 0])
        )

        features.min_y = float(
            np.min(points[:, 1])
        )

        features.max_y = float(
            np.max(points[:, 1])
        )

    # ======================================================
    # FIND CANDIDATE
    # ======================================================

    @staticmethod
    def _find_candidate(
        cluster,
        candidates,
    ):

        for candidate in candidates:

            if candidate.cluster is cluster:
                return candidate

            if candidate.cluster.id == cluster.id:
                return candidate

        return None