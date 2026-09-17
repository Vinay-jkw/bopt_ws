#!/usr/bin/env python3

from dataclasses import dataclass, field

from models.pole_candidate import PoleCandidate
from models.pallete_feature_set import PalletFeatureSet


@dataclass
class PalletPoleInput:
    """
    Input to PalletPoleAlgorithm.

    Contains all pole candidates and the pallet features
    calculated from those candidates.
    """

    candidates: list[PoleCandidate] = field(
        default_factory=list
    )

    features: PalletFeatureSet = field(
        default_factory=PalletFeatureSet
    )