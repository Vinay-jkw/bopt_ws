#!/usr/bin/env python3

from dataclasses import dataclass

from models.cluster import Cluster


@dataclass
class PoleCandidate:

    cluster: Cluster

    score: float = 0.0

    aspect_score: float = 0.0
    size_score: float = 0.0
    point_score: float = 0.0
    shape_score: float = 0.0

    accepted: bool = False