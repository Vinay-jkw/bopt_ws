#!/usr/bin/env python3

import numpy as np

from detectors.base_detector_algorithm import (
    BaseDetectorAlgorithm,
)

from models.pallete_pole_input import PalletPoleInput

from models.pallete_perception import (
    PalletDetectionResult,
)


class PalletDetector(
    BaseDetectorAlgorithm[
        PalletPoleInput,
        PalletDetectionResult,
    ]
):

    """
    Detects a pallet from pallet-pole classification.

    PalletPoleAlgorithm determines whether a PoleCandidate
    is a pallet pole.

    PalletDetector then:

        1. Selects pallet poles.
        2. Uses pallet rows.
        3. Validates pallet geometry.
        4. Estimates missing pole positions.
        5. Calculates the expected pallet centroid.

    Expected pallet structure:

        4 poles in Row A
        4 poles in Row B
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

        if (
            self.expected_poles
            != (
                self.expected_row_a_poles
                + self.expected_row_b_poles
            )
        ):
            raise ValueError(
                "expected_poles must equal "
                "expected_row_a_poles + expected_row_b_poles"
            )

    # ==================================================
    # DETECT
    # ==================================================

    def detect(
        self,
        data: PalletPoleInput,
    ) -> PalletDetectionResult:

        if data is None:
            return PalletDetectionResult(
                detected=False,
                reason="No pallet pole input",
            )

        candidates = data.candidates
        features = data.features

        # --------------------------------------------------
        # No candidates
        # --------------------------------------------------

        if not candidates:
            return PalletDetectionResult(
                detected=False,
                reason="No pole candidates",
            )

        # --------------------------------------------------
        # ONLY pallet poles
        # --------------------------------------------------

        pallet_candidates = [
            candidate
            for candidate in candidates
            if candidate.pallet_pole
        ]

        detected_poles = len(
            pallet_candidates
        )

        # print(
        #     f"PalletDetector | "
        #     f"candidates={len(candidates)} | "
        #     f"pallet_poles={detected_poles}"
        # )

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
                expected_poles=self.expected_poles,
                reason=(
                    f"Only {detected_poles} "
                    f"pallet poles detected; "
                    f"minimum required is "
                    f"{self.min_detected_poles}"
                ),
            )

        # --------------------------------------------------
        # Get pallet-pole cluster IDs
        # --------------------------------------------------

        pallet_cluster_ids = {
            candidate.cluster.id
            for candidate in pallet_candidates
        }

        # --------------------------------------------------
        # Filter existing feature rows
        # using ONLY pallet poles
        # --------------------------------------------------

        row_a = [
            cluster
            for cluster in features.row_a
            if cluster.id in pallet_cluster_ids
        ]

        row_b = [
            cluster
            for cluster in features.row_b
            if cluster.id in pallet_cluster_ids
        ]

        # --------------------------------------------------
        # Sort longitudinally
        # --------------------------------------------------

        row_a = sorted(
            row_a,
            key=lambda cluster: cluster.centroid[0],
        )

        row_b = sorted(
            row_b,
            key=lambda cluster: cluster.centroid[0],
        )

        row_a_count = len(row_a)
        row_b_count = len(row_b)

        # --------------------------------------------------
        # Two-row validation
        # --------------------------------------------------

        if not row_a or not row_b:
            return PalletDetectionResult(
                detected=False,
                detected_poles=detected_poles,
                expected_poles=self.expected_poles,
                row_a_count=row_a_count,
                row_b_count=row_b_count,
                row_a=row_a,
                row_b=row_b,
                reason=(
                    "Pallet poles do not form two rows"
                ),
            )

        # --------------------------------------------------
        # Calculate detected spacing
        # --------------------------------------------------

        row_a_gaps = self._calculate_gaps(
            row_a
        )

        row_b_gaps = self._calculate_gaps(
            row_b
        )

        row_a_spacing = (
            float(np.mean(row_a_gaps))
            if row_a_gaps
            else 0.0
        )

        row_b_spacing = (
            float(np.mean(row_b_gaps))
            if row_b_gaps
            else 0.0
        )

        # --------------------------------------------------
        # Row spacing
        # --------------------------------------------------

        row_spacing = (
            self._calculate_row_spacing(
                row_a,
                row_b,
            )
        )

        # --------------------------------------------------
        # Spacing score
        # --------------------------------------------------

        row_a_score = self._spacing_score(
            row_a_spacing
        )

        row_b_score = self._spacing_score(
            row_b_spacing
        )

        row_spacing_score = (
            self._row_spacing_score(
                row_spacing
            )
        )

        # --------------------------------------------------
        # Geometry score
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

        geometry_threshold = 0.60

        detected = (
            geometry_score
            >= geometry_threshold
        )

        # ==================================================
        # RECONSTRUCT PALLET GEOMETRY
        # ==================================================

        reconstructed_row_a = (
            self._reconstruct_row(
                row_a,
                self.expected_row_a_poles,
            )
        )

        reconstructed_row_b = (
            self._reconstruct_row(
                row_b,
                self.expected_row_b_poles,
            )
        )

        # --------------------------------------------------
        # Calculate pallet centroid from reconstructed
        # geometry
        # --------------------------------------------------

        centroid = (
            self._calculate_pallet_centroid(
                reconstructed_row_a,
                reconstructed_row_b,
            )
        )

        # --------------------------------------------------
        # Position
        # --------------------------------------------------

        x_deviation = float(
            centroid[1]
        )

        y_position = float(
            centroid[0]
        )

        # --------------------------------------------------
        # Orientation
        # --------------------------------------------------

        orientation = (
            self._calculate_orientation(
                reconstructed_row_a,
                reconstructed_row_b,
            )
        )

        # --------------------------------------------------
        # Reason
        # --------------------------------------------------

        if detected:
            reason = (
                "Pallet geometry validated"
            )
        else:
            reason = (
                "Pallet geometry does not "
                "match expected dimensions"
            )

        # --------------------------------------------------
        # Result
        # --------------------------------------------------

        return PalletDetectionResult(
            detected=detected,
            score=geometry_score,

            detected_poles=detected_poles,
            expected_poles=self.expected_poles,

            row_a_count=row_a_count,
            row_b_count=row_b_count,

            row_a=row_a,
            row_b=row_b,

            row_a_spacing=row_a_spacing,
            row_b_spacing=row_b_spacing,
            row_spacing=row_spacing,

            centroid=centroid,

            x_deviation=x_deviation,
            y_deviation=y_position,

            orientation=orientation,

            reason=reason,
        )

    # ==================================================
    # RECONSTRUCT ROW
    # ==================================================

    def _reconstruct_row(
        self,
        row,
        expected_count,
    ):

        """
        Reconstruct the expected longitudinal positions of a pallet row.

        Assumption:
            The most common missing-pole case is at the END of the row.

        Strategy:
            1. Keep all detected poles as anchors.
            2. Estimate the longitudinal spacing.
            3. Detect an internal missing pole if a gap is approximately
               2x/3x/... the expected spacing.
            4. Otherwise, assume missing poles are after the last detected
               pole and extend the row toward the end.
        """

        if not row:
            return []

        detected = [
            np.asarray(cluster.centroid, dtype=np.float32)
            for cluster in row
        ]

        # --------------------------------------------------
        # Already complete
        # --------------------------------------------------

        if len(detected) >= expected_count:
            return detected[:expected_count]

        # --------------------------------------------------
        # Estimate spacing
        # --------------------------------------------------

        detected_gaps = []

        for index in range(len(detected) - 1):
            gap = float(
                np.linalg.norm(
                    detected[index + 1] - detected[index]
                )
            )

            if gap > 1e-6:
                detected_gaps.append(gap)

        spacing = self.expected_longitudinal_spacing

        if detected_gaps:
            # For missing poles, a measured gap can be 2x, 3x, ...
            # the nominal spacing. Reduce those gaps to their likely
            # single-pole spacing before estimating the row spacing.
            normalized_gaps = []

            for gap in detected_gaps:
                multiplier = max(
                    1,
                    int(
                        round(
                            gap
                            / self.expected_longitudinal_spacing
                        )
                    ),
                )

                normalized_gap = (
                    gap / multiplier
                )

                normalized_gaps.append(normalized_gap)

            spacing = float(
                np.median(normalized_gaps)
            )

        # Keep reconstruction tied to the configured pallet geometry.
        if not (
            self.expected_longitudinal_spacing
            - self.longitudinal_spacing_tolerance
            <= spacing
            <=
            self.expected_longitudinal_spacing
            + self.longitudinal_spacing_tolerance
        ):
            spacing = self.expected_longitudinal_spacing

        # --------------------------------------------------
        # Check for internal missing poles
        # --------------------------------------------------

        reconstructed = [detected[0]]

        for index in range(len(detected) - 1):

            first = detected[index]
            second = detected[index + 1]

            gap_vector = second - first
            gap = float(np.linalg.norm(gap_vector))

            if gap <= 1e-6:
                continue

            direction = gap_vector / gap

            # How many nominal pole intervals exist in this gap?
            intervals = max(
                1,
                int(
                    round(
                        gap / spacing
                    )
                ),
            )

            # Only reconstruct internal positions when the gap clearly
            # represents multiple expected pole intervals.
            if intervals > 1:

                for interval in range(
                    1,
                    intervals
                ):
                    point = (
                        first
                        + direction
                        * interval
                        * spacing
                    )

                    reconstructed.append(
                        point.astype(np.float32)
                    )

            reconstructed.append(second)

        # --------------------------------------------------
        # Missing poles at the END
        # --------------------------------------------------

        missing_count = (
            expected_count
            - len(reconstructed)
        )

        if missing_count > 0:

            if len(reconstructed) >= 2:

                first = reconstructed[-2]
                last = reconstructed[-1]

                direction = last - first
                length = np.linalg.norm(direction)

                if length > 1e-6:
                    direction = (
                        direction / length
                    )
                else:
                    direction = np.array(
                        [1.0, 0.0],
                        dtype=np.float32
                    )

            else:
                # With only one detected pole, use the longitudinal
                # direction of the coordinate system.
                direction = np.array(
                    [1.0, 0.0],
                    dtype=np.float32
                )

            for _ in range(missing_count):

                last = np.asarray(
                    reconstructed[-1],
                    dtype=np.float32
                )

                point = (
                    last
                    + direction * spacing
                )

                reconstructed.append(
                    point.astype(np.float32)
                )

        # --------------------------------------------------
        # Logging
        # --------------------------------------------------

        # print(
        #     f"PalletDetector | "
        #     f"row_detected={len(row)} | "
        #     f"row_expected={expected_count} | "
        #     f"missing={max(0, expected_count - len(row))} | "
        #     f"reconstructed={len(reconstructed)}"
        # )

        return reconstructed[:expected_count]

    # ==================================================
    # PALLET CENTROID
    # ==================================================

    def _calculate_pallet_centroid(
        self,
        row_a,
        row_b,
    ) -> np.ndarray:

        """
        Calculate centroid from reconstructed pallet
        geometry.

        This is intentionally different from calculating
        the centroid of detected poles.

        The returned centroid represents the estimated
        center of the complete pallet.
        """

        points = []

        points.extend(row_a)
        points.extend(row_b)

        if not points:
            return np.zeros(
                2,
                dtype=np.float32,
            )

        points = np.asarray(
            points,
            dtype=np.float32,
        )

        centroid = np.mean(
            points,
            axis=0,
        )

        # print(
        #     f"PalletDetector | "
        #     f"reconstructed_points={len(points)} | "
        #     f"pallet_centroid="
        #     f"[{centroid[0]:.3f}, "
        #     f"{centroid[1]:.3f}]"
        # )

        return centroid

    # ==================================================
    # GAPS
    # ==================================================

    def _calculate_gaps(
        self,
        row,
    ):

        if len(row) < 2:
            return []

        gaps = []

        for index in range(
            len(row) - 1
        ):

            first = np.asarray(
                row[index].centroid
            )

            second = np.asarray(
                row[index + 1].centroid
            )

            distance = float(
                np.linalg.norm(
                    second - first
                )
            )

            gaps.append(distance)

        return gaps

    # ==================================================
    # ROW SPACING
    # ==================================================

    def _calculate_row_spacing(
        self,
        row_a,
        row_b,
    ):

        if not row_a or not row_b:
            return 0.0

        mean_a = np.mean(
            [
                cluster.centroid[1]
                for cluster in row_a
            ]
        )

        mean_b = np.mean(
            [
                cluster.centroid[1]
                for cluster in row_b
            ]
        )

        return float(
            abs(mean_a - mean_b)
        )

    # ==================================================
    # ORIENTATION
    # ==================================================

    def _calculate_orientation(
        self,
        row_a,
        row_b,
    ):

        row = (
            row_a
            if len(row_a) >= 2
            else row_b
        )

        if len(row) < 2:
            return 0.0

        first = row[0]
        last = row[-1]

        dy = float(
            last[0] - first[0]
        )

        dx = float(
            last[1] - first[1]
        )

        orientation = np.arctan2(
            dx,
            dy,
        )

        return float(
            orientation
        )

    # ==================================================
    # LONGITUDINAL SPACING SCORE
    # ==================================================

    def _spacing_score(
        self,
        spacing,
    ):

        """
        Score longitudinal spacing while allowing missing poles.

        Examples with expected spacing = 0.46 m:

            0.46 m -> adjacent poles
            0.92 m -> one missing pole
            1.38 m -> two missing poles

        The score is based on the nearest integer multiple of the
        expected spacing.
        """

        if spacing <= 0.0:
            return 0.0

        expected = self.expected_longitudinal_spacing
        tolerance = self.longitudinal_spacing_tolerance

        if expected <= 0.0 or tolerance <= 0.0:
            return 0.0

        multiplier = max(
            1,
            int(round(spacing / expected))
        )

        expected_gap = (
            multiplier * expected
        )

        error = abs(
            spacing - expected_gap
        )

        if error > tolerance:
            return 0.0

        return float(
            1.0 - error / tolerance
        )

    # ==================================================
    # ROW SPACING SCORE
    # ==================================================

    def _row_spacing_score(
        self,
        spacing,
    ):

        if spacing <= 0.0:
            return 0.0

        error = abs(
            spacing
            - self.expected_row_spacing
        )

        tolerance = (
            self.row_spacing_tolerance
        )

        if tolerance <= 0.0:
            return 0.0

        if error > tolerance:
            return 0.0

        return float(
            1.0
            - error / tolerance
        )