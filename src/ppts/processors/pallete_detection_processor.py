#!/usr/bin/env python3

from processors.base_processor import BaseProcessor

from detectors.pallete_detector import PalletDetector


class PalletDetectionProcessor(BaseProcessor):
    """
    Executes pallet-level detection using the configured
    PalletDetector algorithm.

    Input
    -----
    context.pallet_features

    Output
    ------
    context.pallet_detection

    Responsibilities
    ----------------
    - Read pallet features from the context
    - Execute PalletDetector
    - Store the detection result back into the context

    This processor does NOT perform geometric calculations.
    Those are handled by PalletFeatureProcessor.

    This processor does NOT implement detection logic.
    That is handled by PalletDetector.
    """

    def __init__(
        self,
        detector: PalletDetector,
        input_key: str = "pallet_features",
        output_key: str = "pallet_detection",
    ):
        self.detector = detector
        self.input_key = input_key
        self.output_key = output_key

    # ======================================================
    # PROCESS
    # ======================================================

    def process(self, context) -> None:
        """
        Run pallet detection on the calculated pallet features.
        """

        features = getattr(
            context,
            self.input_key,
            None,
        )

        # --------------------------------------------------
        # No features available
        # --------------------------------------------------

        if features is None:

            result = self.detector.detect(
                None
            )

            setattr(
                context,
                self.output_key,
                result,
            )

            return

        # --------------------------------------------------
        # Run detector
        # --------------------------------------------------

        result = self.detector.detect(
            features
        )

        # --------------------------------------------------
        # Store result
        # --------------------------------------------------

        setattr(
            context,
            self.output_key,
            result
        )