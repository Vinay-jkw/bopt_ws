#!/usr/bin/env python3

from processors.base_processor import BaseProcessor
from detectors.base_detector_algorithm import BasePathAlgorithm
from models.pallete_perception import PalletDetectionResult
from models.pallete_path import PalletPath


class PalletPathProcessor(BaseProcessor):

    def __init__(
        self,
        algorithm: BasePathAlgorithm,
        input_key: str = "pallet_detection",
        output_key: str = "pallet_path",
    ):
        self.algorithm = algorithm
        self.input_key = input_key
        self.output_key = output_key

    def process(self, context):

        detection = getattr(context, self.input_key)

        path = self.algorithm.generate(detection)

        setattr(context, self.output_key, path)

        return context