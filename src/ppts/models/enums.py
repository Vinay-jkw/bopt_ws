#!/usr/bin/env python3

from enum import Enum


class TransformProviderType(str, Enum):
    """
    Supported transform providers.
    """

    TF = "tf"
    YAML = "yaml"


class ClusterAlgorithm(str, Enum):
    """
    Supported clustering algorithms.
    """

    DBSCAN = "dbscan"
    KMEANS = "kmeans"
    EUCLIDEAN = "euclidean"

class ClusterQuality(Enum):
    """
    Quality/status assigned during cluster validation.
    """
    REJECTED = 0
    UNKNOWN = 1
    VALID = 2
    WEAK = 3
    SUSPICIOUS = 4
    REFINED = 5

class ClusterLabel(Enum):
    """
    Semantic label assigned by higher-level perception modules.
    """

    UNKNOWN = 0
    PALLET_LEG = 1
    WALL = 2
    OBSTACLE = 3
    NOISE = 4

class ClusterType(Enum):

    UNKNOWN = 0
    POLE_CANDIDATE = 1
    NON_POLE = 2