#!/usr/bin/env python3

from processors.base_processor import BaseProcessor
from detectors.pallete_pole_algorithm import PalletPoleAlgorithm
from models.pallete_pole_input import PalletPoleInput


class PalletPoleProcessor(BaseProcessor):

    def __init__(
        self,
        algorithm: PalletPoleAlgorithm,
        input_key: str = "pole_candidates",
        feature_key: str = "pallet_features",
        output_key: str = "pallet_pole_input",
    ):
        self.algorithm = algorithm
        self.input_key = input_key
        self.feature_key = feature_key
        self.output_key = output_key

    def process(self, context):

        candidates = getattr(
            context,
            self.input_key,
            [],
        )

        features = getattr(
            context,
            self.feature_key,
            None,
        )

        data = PalletPoleInput(
            candidates=candidates,
            features=features,
        )

        result = self.algorithm.detect(data)

        setattr(
            context,
            self.output_key,
            result,
        )

        # Keep the classified candidates available
        # in context as well.
        context.pole_candidates = result.candidates

        # for index, candidate in enumerate(
        #     result.candidates
        # ):
        #     self.get_logger().info(
        #         f"Pole Candidate {index} | "
        #         f"accepted={candidate.accepted} | "
        #         f"pallet_pole={candidate.pallet_pole}"
        #     )

        return context