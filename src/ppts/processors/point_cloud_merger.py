#!/usr/bin/env python3

import numpy as np

from processors.base_processor import BaseProcessor
from models.point_cloud import PointCloud


class PointCloudMerger(BaseProcessor):
    """
    Generic Point Cloud Merger.

    Merges multiple PointCloud objects into a single PointCloud.
    """

    def __init__(
        self,
        input_keys,
        output_key="merged_cloud",
        output_frame="load_wheel_base_link",
        output_source="merged",
    ):
        self.input_keys = input_keys
        self.output_key = output_key
        self.output_frame = output_frame
        self.output_source = output_source

    def merge(self, clouds):

        valid_clouds = [
            cloud for cloud in clouds
            if cloud is not None and cloud.points.size > 0
        ]

        if not valid_clouds:
            return PointCloud(
                frame_id=self.output_frame,
                source=self.output_source,
            )

        merged_points = np.vstack(
            [cloud.points for cloud in valid_clouds]
        )

        timestamp = max(
            cloud.timestamp for cloud in valid_clouds
        )

        return PointCloud(
            points=merged_points,
            frame_id=self.output_frame,
            source=self.output_source,
            timestamp=timestamp,
        )

    def process(self, context):

        clouds = [
            getattr(context, key)
            for key in self.input_keys
        ]

        merged_cloud = self.merge(clouds)

        setattr(
            context,
            self.output_key,
            merged_cloud
        )