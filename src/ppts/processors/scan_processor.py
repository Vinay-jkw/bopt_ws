#!/usr/bin/env python3

import numpy as np

from sensor_msgs.msg import LaserScan

from processors.base_processor import BaseProcessor
from models.point_cloud import PointCloud


class ScanProcessor(BaseProcessor):
    """
    Converts a LaserScan into a PointCloud.
    """

    def __init__(
        self,
        input_key: str,
        output_key: str,
        source: str,
    ):
        self.input_key = input_key
        self.output_key = output_key
        self.source = source

    def scan_to_cloud(self, scan: LaserScan) -> PointCloud:

        ranges = np.asarray(scan.ranges, dtype=np.float32)

        angles = (
            scan.angle_min
            + np.arange(len(ranges)) * scan.angle_increment
        )

        valid = (
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges <= scan.range_max)
        )

        ranges = ranges[valid]
        angles = angles[valid]

        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)

        points = np.column_stack(
                (
                    x,
                    y,
                    np.zeros_like(x),
                )
            ).astype(np.float32)

        timestamp = (
            scan.header.stamp.sec
            + scan.header.stamp.nanosec * 1e-9
        )

        return PointCloud(
            points=points,
            frame_id=scan.header.frame_id,
            source=self.source,
            timestamp=timestamp,
        )

    def process(self, context):

        scan = getattr(context, self.input_key)

        cloud = self.scan_to_cloud(scan)

        setattr(context, self.output_key, cloud)