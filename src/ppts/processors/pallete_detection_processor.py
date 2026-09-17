#!/usr/bin/env python3

from processors.base_processor import BaseProcessor
from detectors.pallete_detector import PalletDetector


class PalletDetectionProcessor(BaseProcessor):
    """
    Executes pallet-level detection using the configured
    PalletDetector algorithm.

    Input
    -----
    context.pallet_pole_input

    Output
    ------
    context.pallet_detection

    Responsibilities
    ----------------
    - Read pallet-pole input from the context
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
        input_key: str = "pallet_pole_input",
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
        Run pallet detection on the pallet-pole input.
        """

        pallet_pole_input = getattr(
            context,
            self.input_key,
            None,
        )

        # --------------------------------------------------
        # No input available
        # --------------------------------------------------

        if pallet_pole_input is None:
            print(
                "PalletDetectionProcessor | "
                "pallet_pole_input is None"
            )
            result = self.detector.detect(None)

            setattr(
                context,
                self.output_key,
                result,
            )

            return context

        # --------------------------------------------------
        # Run detector
        # --------------------------------------------------
        print(
            f"PalletDetectionProcessor | "
            f"candidates={len(pallet_pole_input.candidates)} | "
            f"row_a={len(pallet_pole_input.features.row_a)} | "
            f"row_b={len(pallet_pole_input.features.row_b)}"
        )
        result = self.detector.detect(
            pallet_pole_input
        )

        # --------------------------------------------------
        # Store result
        # --------------------------------------------------

        setattr(
            context,
            self.output_key,
            result,
        )

        return context