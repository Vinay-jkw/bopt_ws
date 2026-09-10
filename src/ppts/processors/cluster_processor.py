#!/usr/bin/env python3

from algorithms.base_cluster_algorithm import BaseClusterAlgorithm
from models.cluster import Cluster, ROICluster
from processors.base_processor import BaseProcessor


class ClusterProcessor(BaseProcessor):
    """
    Performs primary clustering on every ROI cloud.

    Input
    -----
    context.roi_clouds

    Output
    ------
    context.roi_clusters
    context.clusters
    """

    def __init__(
        self,
        algorithm: BaseClusterAlgorithm,
    ):
        self.algorithm = algorithm

    def process(self, context) -> None:

        roi_clusters: list[ROICluster] = []

        all_clusters: list[Cluster] = []

        for roi_cloud in context.roi_clouds:

            if roi_cloud is None:
                continue

            cloud = roi_cloud.cloud

            if cloud is None or cloud.is_empty:
                continue

            detected = self.algorithm.detect(
                cloud
            )

            roi_cluster = ROICluster(
                roi=roi_cloud.roi,
                cloud=cloud,
                clusters=detected,
            )

            roi_clusters.append(
                roi_cluster
            )

            all_clusters.extend(
                detected
            )

        context.roi_clusters = roi_clusters
        context.clusters = all_clusters