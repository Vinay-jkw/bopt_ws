#!/usr/bin/env python3

from processors.base_processor import BaseProcessor
from models.point_cloud import PointCloud, ROICloud
from models.roi import ROI
import numpy as np

class ROIFilterProcessor(BaseProcessor):
    """
    Filters the merged point cloud into ROI clouds.

    If ROI filtering is disabled, the complete merged cloud is
    forwarded as a single ROICloud so that downstream processors
    receive the same data structure in both modes.
    """

    def __init__(
        self,
        input_key: str,
        output_key: str,
        enabled: bool = True,
    ):
        self.input_key = input_key
        self.output_key = output_key
        self.enabled = enabled

    def process(self, context) -> None:
        """
        Generate ROI clouds or forward the complete cloud.

        When enabled:
            merged_cloud -> ROI filtering -> roi_clouds

        When disabled:
            merged_cloud -> roi_clouds
        """

        cloud = getattr(
            context,
            self.input_key,
            None,
        )

        if cloud is None or cloud.is_empty:
            setattr(
                context,
                self.output_key,
                [],
            )
            return

        # --------------------------------------------------
        # ROI filtering disabled
        # --------------------------------------------------

        if not self.enabled:
            setattr(
                context,
                self.output_key,
                self._create_full_cloud_roi(cloud),
            )
            return

        # --------------------------------------------------
        # ROI filtering enabled
        # --------------------------------------------------

        rois = context.rois

        roi_clouds = self._filter_cloud(
            cloud,
            rois,
        )

        setattr(
            context,
            self.output_key,
            roi_clouds,
        )

    def _filter_cloud(
        self,
        cloud: PointCloud,
        rois: list[ROI],
    ) -> list[ROICloud]:

        roi_clouds: list[ROICloud] = []

        if cloud is None or cloud.is_empty:
            return roi_clouds

        for roi in rois:

            mask = (
                (cloud.points[:, 0] >= roi.xmin)
                & (cloud.points[:, 0] <= roi.xmax)
                & (cloud.points[:, 1] >= roi.ymin)
                & (cloud.points[:, 1] <= roi.ymax)
            )

            roi_points = cloud.points[mask]

            if roi_points.size == 0:
                continue

            roi_clouds.append(
                ROICloud(
                    roi=roi,
                    cloud=cloud.copy_with_points(
                        roi_points
                    ),
                )
            )

        return roi_clouds

    def _create_full_cloud_roi(
        self,
        cloud: PointCloud,
    ) -> list[ROICloud]:
        """
        Wrap the complete cloud in a single ROICloud.

        No point filtering is performed.
        """

        points = cloud.points

        roi = ROI(
            id=0,
            xmin=float(np.min(points[:, 0])),
            xmax=float(np.max(points[:, 0])),
            ymin=float(np.min(points[:, 1])),
            ymax=float(np.max(points[:, 1])),
        )

        return [
            ROICloud(
                roi=roi,
                cloud=cloud,
            )
        ]