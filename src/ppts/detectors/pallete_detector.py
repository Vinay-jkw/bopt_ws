#!/usr/bin/env python3

import numpy as np

from detectors.base_detector_algorithm import (
    BaseDetectorAlgorithm,
)

from models.pallete_perception import (
    PalletDetectionResult,
)


class PalletDetector(BaseDetectorAlgorithm):
    """
    Detects a pallet from pallet-level geometric features.

    The detector does NOT perform clustering.

    It consumes the output of PalletFeatureProcessor and
    validates:

        - minimum number of detected poles
        - two-row structure
        - longitudinal pole spacing
        - row-to-row spacing
        - pallet geometry
    """

    def __init__(
        self,
        expected_poles: int = 8,
        min_detected_poles: int = 5,

        expected_row_a_poles: int = 4,
        expected_row_b_poles: int = 4,

        expected_longitudinal_spacing: float = 0.46,
        longitudinal_spacing_tolerance: float = 0.08,

        expected_row_spacing: float = 0.80,
        row_spacing_tolerance: float = 0.12,
        geometry_threshold: float = 0.60,
    ):

        self.expected_poles = expected_poles

        self.min_detected_poles = (
            min_detected_poles
        )

        self.expected_row_a_poles = (
            expected_row_a_poles
        )

        self.expected_row_b_poles = (
            expected_row_b_poles
        )

        self.expected_longitudinal_spacing = (
            expected_longitudinal_spacing
        )

        self.longitudinal_spacing_tolerance = (
            longitudinal_spacing_tolerance
        )

        self.expected_row_spacing = (
            expected_row_spacing
        )

        self.row_spacing_tolerance = (
            row_spacing_tolerance
        )

        self.geometry_threshold = (
            geometry_threshold
        )

    # ==================================================
    # DETECT
    # ==================================================

    def detect(
        self,
        features,
    ) -> PalletDetectionResult:

        if features is None:

            return PalletDetectionResult(
                detected=False,
                reason="No pallet features",
            )

        candidates = getattr(
            features,
            "candidates",
            [],
        )

        detected_poles = len(
            candidates
        )

        # --------------------------------------------------
        # Minimum pole check
        # --------------------------------------------------

        if (
            detected_poles
            < self.min_detected_poles
        ):

            return PalletDetectionResult(
                detected=False,

                detected_poles=detected_poles,

                expected_poles=(
                    self.expected_poles
                ),

                reason=(
                    f"Only {detected_poles} "
                    f"pole candidates detected; "
                    f"minimum required is "
                    f"{self.min_detected_poles}"
                ),
            )

        # --------------------------------------------------
        # Get rows
        # --------------------------------------------------

        row_a = getattr(
            features,
            "row_a",
            [],
        )

        row_b = getattr(
            features,
            "row_b",
            [],
        )

        row_a_count = len(row_a)
        row_b_count = len(row_b)

        # --------------------------------------------------
        # Two-row check
        # --------------------------------------------------

        if not row_a or not row_b:

            return PalletDetectionResult(
                detected=False,

                detected_poles=detected_poles,

                expected_poles=(
                    self.expected_poles
                ),

                row_a_count=row_a_count,
                row_b_count=row_b_count,

                row_a=row_a,
                row_b=row_b,

                reason=(
                    "Pallet candidates do not "
                    "form two rows"
                ),
            )

        # --------------------------------------------------
        # Spacing validation
        # --------------------------------------------------

        row_a_score = (
            self._spacing_score(
                features.row_a_spacing
            )
        )

        row_b_score = (
            self._spacing_score(
                features.row_b_spacing
            )
        )

        row_spacing_score = (
            self._row_spacing_score(
                features.row_spacing
            )
        )

        # --------------------------------------------------
        # Calculate final score
        # --------------------------------------------------

        geometry_score = float(
            np.mean(
                [
                    row_a_score,
                    row_b_score,
                    row_spacing_score,
                ]
            )
        )

        # --------------------------------------------------
        # Require reasonable geometry
        # --------------------------------------------------


        detected = (
            geometry_score
            >= self.geometry_threshold
        )

        start_a = row_a[0].centroid
        start_b = row_b[0].centroid

        x_deviation = float(
            (start_a[1] + start_b[1]) / 2.0
        )

        y_position = float(
            (start_a[0] + start_b[0]) / 2.0
        )
        # x_deviation = float(features.centroid[1])

        # y_position = float(features.centroid[0])

        orientation = self._calculate_orientation(row_a,row_b)
        if detected:

            reason = (
                "Pallet geometry validated"
            )

        else:

            reason = (
                "Pallet geometry does not "
                "match expected dimensions"
            )

        return PalletDetectionResult(
            detected=detected,

            score=geometry_score,

            detected_poles=detected_poles,

            expected_poles=(
                self.expected_poles
            ),

            row_a_count=row_a_count,
            row_b_count=row_b_count,

            row_a=row_a,
            row_b=row_b,

            row_a_spacing=(
                features.row_a_spacing
            ),

            row_b_spacing=(
                features.row_b_spacing
            ),

            row_spacing=(
                features.row_spacing
            ),

            centroid=features.centroid,

            x_deviation=x_deviation,

            y_deviation=y_position,

            orientation=orientation,

            reason=reason,
        )

    # ==================================================
    # LONGITUDINAL SPACING SCORE
    # ==================================================
    def _calculate_orientation(
        self,
        row_a,
        row_b,
    ) -> float:

        row = row_a if len(row_a) >= 2 else row_b

        if len(row) < 2:
            return 0.0

        first = row[0].centroid
        last = row[-1].centroid

        dy = float(
            last[0] - first[0]
        )

        dx = float(
            last[1] - first[1]
        )

        # Angle relative to +Y axis
        orientation = np.arctan2(
            dx,
            dy,
        )

        return float(orientation)


    def _spacing_score(
        self,
        spacing: float,
    ) -> float:

        if spacing <= 0.0:
            return 0.0

        error = abs(
            spacing
            - self.expected_longitudinal_spacing
        )

        tolerance = (
            self.longitudinal_spacing_tolerance
        )

        if error > tolerance:
            return 0.0

        return float(
            1.0
            - (
                error
                / tolerance
            )
        )

    # ==================================================
    # ROW SPACING SCORE
    # ==================================================

    def _row_spacing_score(
        self,
        spacing: float,
    ) -> float:

        if spacing <= 0.0:
            return 0.0

        error = abs(
            spacing
            - self.expected_row_spacing
        )

        tolerance = (
            self.row_spacing_tolerance
        )

        if error > tolerance:
            return 0.0

        return float(
            1.0
            - (
                error
                / tolerance
            )
        )

    