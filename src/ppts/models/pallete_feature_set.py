#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import List

import numpy as np

from models.cluster import Cluster


@dataclass
class PalletFeatureSet:

    candidates: list[Cluster] = field(
        default_factory=list
    )

    # Two longitudinal pallet rows
    row_a: list[Cluster] = field(
        default_factory=list
    )

    row_b: list[Cluster] = field(
        default_factory=list
    )

    # Spacing between poles in each row
    row_a_gaps: list[float] = field(
        default_factory=list
    )

    row_b_gaps: list[float] = field(
        default_factory=list
    )

    # Average spacing
    row_a_spacing: float = 0.0
    row_b_spacing: float = 0.0

    # Distance between pallet rows
    row_spacing: float = 0.0

    # Overall pallet geometry
    centroid: np.ndarray = field(
        default_factory=lambda: np.zeros(
            2,
            dtype=np.float32
        )
    )

    min_x: float = 0.0
    max_x: float = 0.0
    min_y: float = 0.0
    max_y: float = 0.0