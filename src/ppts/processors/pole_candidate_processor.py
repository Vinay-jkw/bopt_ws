#!/usr/bin/env python3
import numpy as np
from models.cluster import ClusterQuality
from models.pole_candidate import PoleCandidate
from processors.base_processor import BaseProcessor
from config.config_models import PoleCandidateConfig

class PoleCandidateProcessor(BaseProcessor):
    """
    Evaluates validated clusters and determines whether they
    are geometrically plausible pallet-pole candidates.

    This processor does NOT detect a pallet.

    It only answers:

        "Can this cluster reasonably represent a pallet pole?"

    Pallet-level reasoning is handled later by PalletDetector.
    """

    def __init__(
        self,
        config: PoleCandidateConfig,
        input_key: str = "clusters",
        output_key: str = "pole_candidates",
    ):
        self.config = config
        self.input_key = input_key
        self.output_key = output_key


    # ======================================================
    # PROCESS
    # ======================================================

    def process(self, context) -> None:

        clusters = getattr(
            context,
            self.input_key,
            [],
        )

        candidates = []

        for cluster in clusters:

            candidate = self._evaluate_cluster(
                cluster
            )

            candidates.append(candidate)

        setattr(
            context,
            self.output_key,
            candidates,
        )

    # ======================================================
    # CLUSTER EVALUATION
    # ======================================================

    def _evaluate_cluster(
        self,
        cluster,
    ) -> PoleCandidate:

        candidate = PoleCandidate(
            cluster=cluster
        )

        if cluster is None:
            return candidate

        # --------------------------------------------------
        # Reject clusters already marked as rejected
        # --------------------------------------------------

        if cluster.quality == ClusterQuality.REJECTED:
            return candidate

        # --------------------------------------------------
        # Point score
        # --------------------------------------------------

        point_score = self._calculate_point_score(
            cluster.point_count
        )

        # --------------------------------------------------
        # Size score
        # --------------------------------------------------

        width_score = self._calculate_range_score(
            cluster.width,
            self.config.min_width,
            self.config.max_width,
        )

        height_score = self._calculate_range_score(
            cluster.height,
            self.config.min_height,
            self.config.max_height,
        )

        size_score = (
            width_score + height_score
        ) / 2.0

        # --------------------------------------------------
        # Aspect ratio
        # --------------------------------------------------

        aspect_ratio = self._calculate_aspect_ratio(
            cluster
        )

        aspect_score = (
            self._calculate_aspect_score(
                aspect_ratio
            )
        )

        # --------------------------------------------------
        # PCA shape
        # --------------------------------------------------

        shape_score = (
            self._calculate_shape_score(
                cluster
            )
        )

        # --------------------------------------------------
        # Final score
        # --------------------------------------------------

        weights = self.config.weights

        score = (
            weights.point * point_score
            + weights.size * size_score
            + weights.aspect * aspect_score
            + weights.shape * shape_score
        )

        # --------------------------------------------------
        # Weak clusters should remain usable but receive
        # reduced confidence.
        # --------------------------------------------------

        quality = self.config.quality
        if cluster.quality == ClusterQuality.WEAK:
            score *= quality.weak_factor

        elif cluster.quality == ClusterQuality.SUSPICIOUS:
            score *= quality.suspicious_factor

        candidate.point_score = point_score
        candidate.size_score = size_score
        candidate.aspect_score = aspect_score
        candidate.shape_score = shape_score

        candidate.score = score

        candidate.accepted = (
            score >= self.config.min_score
        )

        return candidate

    # ======================================================
    # POINT COUNT
    # ======================================================

    def _calculate_point_score(
        self,
        point_count: int,
    ) -> float:

        if point_count < self.config.min_points:
            return 0.0

        # Saturates at 20 points.
        return min(
            point_count /  self.config.saturation_points,
            1.0,
        )

    # ======================================================
    # SIZE
    # ======================================================

    @staticmethod
    def _calculate_range_score(
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:

        if value < minimum:
            return 0.0

        if value > maximum:
            return 0.0

        center = (
            minimum + maximum
        ) / 2.0

        half_range = (
            maximum - minimum
        ) / 2.0

        if half_range <= 0.0:
            return 1.0

        distance = abs(
            value - center
        )

        return max(
            0.0,
            1.0 - (
                distance / half_range
            ),
        )

    # ======================================================
    # ASPECT RATIO
    # ======================================================

    def _calculate_aspect_ratio(
        self,
        cluster,
    ) -> float:

        width = max(
            float(cluster.width),
            1e-6,
        )

        height = max(
            float(cluster.height),
            1e-6,
        )

        # Compactness ratio.
        #
        # 1.0 -> width and height are identical
        # 0.5 -> one dimension is approximately
        #        twice the other
        #
        return min(
            width / height,
            height / width,
        )


    def _calculate_aspect_score(
        self,
        aspect_ratio: float,
    ) -> float:

        # Aspect ratio is already normalized to [0, 1].
        #
        # 1.0 = perfectly compact
        # 0.8 = good
        # 0.5 = elongated
        #
        return float(
            np.clip(
                aspect_ratio,
                0.0,
                1.0,
            )
        )

    # ======================================================
    # PCA SHAPE
    # ======================================================

    @staticmethod
    def _calculate_shape_score(
        cluster,
    ) -> float:

        major = cluster.major_spread
        minor = cluster.minor_spread

        if major <= 0.0:
            return 0.0

        ratio = (
            major
            / max(minor, 1e-6)
        )

        # 4:1 PCA elongation gives full score.
        return min(
            ratio / 4.0,
            1.0,
        )