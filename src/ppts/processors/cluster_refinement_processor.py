#!/usr/bin/env python3

from models.cluster import ClusterQuality
from processors.base_processor import BaseProcessor
from algorithms.base_cluster_algorithm import BaseClusterAlgorithm


class ClusterRefinementProcessor(BaseProcessor):
    """
    Closed-loop refinement of suspicious clusters.

    Processing logic
    ----------------

    VALID:
        Keep the cluster unchanged.

    WEAK:
        Keep the cluster unchanged.

    SUSPICIOUS:
        Refine the cluster using the configured refinement
        algorithm, e.g. K-Means.

        After successful refinement:

            1. Replace the suspicious cluster.
            2. Recalculate features.
            3. Revalidate clusters.
            4. Check for remaining suspicious clusters.
            5. Repeat until no suspicious clusters remain
               or max_iterations is reached.

    Responsibilities
    ----------------
    This processor controls the refinement loop.

    It does NOT:
        - decide whether a cluster is suspicious
        - calculate cluster features
        - implement K-Means

    Those responsibilities belong to:

        ClusterValidationProcessor
        ClusterFeatureProcessor
        BaseClusterAlgorithm
    """

    def __init__(
        self,
        algorithm: BaseClusterAlgorithm,
        feature_processor: BaseProcessor,
        validation_processor: BaseProcessor,
        enabled: bool = True,
        max_iterations: int = 2,
    ):
        if algorithm is None:
            raise ValueError(
                "Refinement algorithm cannot be None"
            )

        if feature_processor is None:
            raise ValueError(
                "Feature processor cannot be None"
            )

        if validation_processor is None:
            raise ValueError(
                "Validation processor cannot be None"
            )

        if max_iterations < 1:
            raise ValueError(
                "max_iterations must be at least 1"
            )

        self.algorithm = algorithm
        self.feature_processor = feature_processor
        self.validation_processor = validation_processor
        self.enabled = enabled
        self.max_iterations = max_iterations

    # ==========================================================
    # Processing
    # ==========================================================

    def process(self, context) -> None:
        """
        Run closed-loop cluster refinement.

        Flow:

            suspicious clusters
                    ↓
                 K-Means
                    ↓
             replace cluster
                    ↓
            feature calculation
                    ↓
               validation
                    ↓
             suspicious again?
                /          \
              YES          NO
               ↓            ↓
             K-Means       DONE
        """

        if not self.enabled:
            return

        roi_clusters = getattr(
            context,
            "roi_clusters",
            None,
        )

        if not roi_clusters:
            return

        # ------------------------------------------------------
        # Refinement iterations
        # ------------------------------------------------------

        for iteration in range(
            self.max_iterations
        ):

            refined_any = False

            # --------------------------------------------------
            # Process each ROI independently
            # --------------------------------------------------

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

                refined_clusters = []

                # ----------------------------------------------
                # Process clusters inside this ROI
                # ----------------------------------------------

                for cluster in clusters:

                    if cluster is None:
                        continue

                    # ------------------------------------------
                    # VALID / WEAK
                    #
                    # Keep unchanged.
                    # ------------------------------------------

                    if (
                        cluster.quality
                        != ClusterQuality.SUSPICIOUS
                    ):
                        refined_clusters.append(
                            cluster
                        )
                        continue

                    # ------------------------------------------
                    # SUSPICIOUS
                    #
                    # Attempt refinement.
                    # ------------------------------------------

                    if cluster.cloud is None:
                        refined_clusters.append(
                            cluster
                        )
                        continue

                    if cluster.cloud.is_empty:
                        refined_clusters.append(
                            cluster
                        )
                        continue

                    refined = self.algorithm.detect(
                        cluster.cloud
                    )

                    # ------------------------------------------
                    # Refinement failed
                    #
                    # Keep original cluster.
                    # ------------------------------------------

                    if not refined:
                        refined_clusters.append(
                            cluster
                        )
                        continue

                    # ------------------------------------------
                    # A refinement is useful only if it
                    # actually produces multiple clusters.
                    #
                    # If K-Means returns one cluster, the
                    # suspicious cluster has not really been
                    # split.
                    # ------------------------------------------

                    if len(refined) < 2:
                        refined_clusters.append(
                            cluster
                        )
                        continue

                    # ------------------------------------------
                    # Successful refinement
                    #
                    # Replace suspicious cluster with the
                    # newly generated clusters.
                    # ------------------------------------------

                    refined_clusters.extend(
                        refined
                    )

                    refined_any = True

                # --------------------------------------------------
                # Replace this ROI's cluster list.
                # --------------------------------------------------

                roi_cluster.clusters = (
                    refined_clusters
                )

            # ------------------------------------------------------
            # No successful refinement
            #
            # Nothing changed, so there is no reason to run
            # feature calculation and validation again.
            # ------------------------------------------------------

            if not refined_any:
                break

            # ------------------------------------------------------
            # IMPORTANT
            #
            # K-Means generated new point sets.
            #
            # Therefore their features must be calculated again.
            # ------------------------------------------------------

            self.feature_processor.process(
                context
            )

            # ------------------------------------------------------
            # Revalidate the newly generated clusters.
            # ------------------------------------------------------

            self.validation_processor.process(
                context
            )

            # ------------------------------------------------------
            # Stop when no suspicious clusters remain.
            # ------------------------------------------------------

            if not self._has_suspicious_clusters(
                context
            ):
                break

        # ------------------------------------------------------
        # Optional final state
        #
        # At this point either:
        #
        #   1. no suspicious clusters remain
        #   2. max_iterations was reached
        #   3. refinement could not produce a valid split
        #
        # The remaining suspicious clusters are intentionally
        # kept. They should never silently disappear.
        # ------------------------------------------------------

    # ==========================================================
    # Helpers
    # ==========================================================

    @staticmethod
    def _has_suspicious_clusters(
        context,
    ) -> bool:
        """
        Return True if at least one suspicious cluster remains.
        """

        roi_clusters = getattr(
            context,
            "roi_clusters",
            None,
        )

        if not roi_clusters:
            return False

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

                if (
                    cluster.quality
                    == ClusterQuality.SUSPICIOUS
                ):
                    return True

        return False