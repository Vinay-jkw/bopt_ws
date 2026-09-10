#!/usr/bin/env python3

from abc import ABC, abstractmethod

from models.point_cloud import PointCloud
from models.cluster import Cluster


class BaseClusterAlgorithm(ABC):
    """
    Base interface for clustering algorithms.

    A clustering algorithm receives one PointCloud and returns
    a list of Cluster objects.
    """

    @abstractmethod
    def detect(
        self,
        cloud: PointCloud,
    ) -> list[Cluster]:
        """
        Detect spatial clusters.

        Parameters
        ----------
        cloud : PointCloud
            Input point cloud.

        Returns
        -------
        list[Cluster]
            Detected clusters.
        """
        raise NotImplementedError