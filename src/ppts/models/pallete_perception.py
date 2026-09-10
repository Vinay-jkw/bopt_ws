#!/usr/bin/env python3

from dataclasses import dataclass, field
from typing import List

from models.cluster import Cluster


@dataclass
class PalletDetectionResult:
    """
    Result produced by PalletDetector.
    """

    detected: bool = False

    score: float = 0.0

    detected_poles: int = 0

    expected_poles: int = 8

    row_a_count: int = 0
    row_b_count: int = 0

    row_a: List[Cluster] = field(
        default_factory=list
    )

    row_b: List[Cluster] = field(
        default_factory=list
    )

    row_a_spacing: float = 0.0

    row_b_spacing: float = 0.0

    row_spacing: float = 0.0

    centroid: object = None
    
    x_deviation: float = 0.0

    y_deviation: float = 0.0

    orientation: float = 0.0

    reason: str = ""