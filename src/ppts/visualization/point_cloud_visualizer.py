#!/usr/bin/env python3

import numpy as np

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from models.point_cloud import PointCloud

from visualization.base_visualizer import BaseVisualizer


class PointCloudVisualizer(BaseVisualizer):
    """
    Converts internal PointCloud objects into ROS PointCloud2 messages.
    """

    def __init__(
        self,
        frame_id: str,
        namespace: str = "pointcloud",
    ):
        super().__init__(
            frame_id=frame_id,
            namespace=namespace,
        )

    def create_cloud_msg(
        self,
        cloud: PointCloud,
    ) -> PointCloud2:
        """
        Convert a PointCloud into a PointCloud2 message.
        """

        if cloud is None:
            return PointCloud2()

        if cloud.is_empty:
            return PointCloud2()

        header = self.create_header(cloud.frame_id)

        xyz = np.asarray(
                cloud.points,
                dtype=np.float32,
            )

        return point_cloud2.create_cloud_xyz32(
            header,
            xyz.tolist(),
        )

    def create_cluster_cloud_msg(
        self,
        cluster,
    ) -> PointCloud2:
        """
        Convert a cluster into PointCloud2.
        """

        return self.create_cloud_msg(
            cluster.cloud
        )