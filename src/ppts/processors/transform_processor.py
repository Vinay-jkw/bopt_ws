#!/usr/bin/env python3

import numpy as np

from processors.base_processor import BaseProcessor
from models.point_cloud import PointCloud
from transforms.transform_manager import TransformManager


class TransformProcessor(BaseProcessor):
    """
    Applies a rigid body transformation to a PointCloud.

    The transform is obtained from the TransformManager.
    The processor is completely independent of TF2 and ROS
    messages.
    """

    def __init__(
        self,
        input_key: str,
        output_key: str,
        target_frame: str,
        transform_manager: TransformManager,
    ) -> None:

        self._input_key = input_key
        self._output_key = output_key
        self._target_frame = target_frame
        self._transform_manager = transform_manager

    # ---------------------------------------------------------

    def process(
        self,
        context,
    ) -> None:

        cloud = getattr(
            context,
            self._input_key,
        )

        transformed_cloud = self._transform(cloud)

        setattr(
            context,
            self._output_key,
            transformed_cloud,
        )

    # ---------------------------------------------------------

    def _transform(
        self,
        cloud: PointCloud,
    ) -> PointCloud:

        if cloud is None:
            return None

        if cloud.is_empty:
            return PointCloud(
                points=np.empty((0, 3), dtype=np.float32),
                frame_id=self._target_frame,
                source=cloud.source,
                timestamp=cloud.timestamp,
            )

        # Already in desired frame
        if cloud.frame_id == self._target_frame:
            return cloud

        transform = self._transform_manager.get_transform(
            source_frame=cloud.frame_id,
            target_frame=self._target_frame,
        )

        transformed_points = self._apply_transform(
            cloud.points,
            transform.matrix,
        )

        return PointCloud(
            points=transformed_points,
            frame_id=self._target_frame,
            source=cloud.source,
            timestamp=cloud.timestamp,
        )

    # ---------------------------------------------------------

    @staticmethod
    def _apply_transform(
        points: np.ndarray,
        transform_matrix: np.ndarray,
    ) -> np.ndarray:
        """
        Apply a homogeneous 4×4 rigid body transform.

        Parameters
        ----------
        points : ndarray (N,3)
            Input point cloud.

        transform_matrix : ndarray (4,4)
            Homogeneous transform matrix.

        Returns
        -------
        ndarray (N,3)
            Transformed points.
        """

        num_points = points.shape[0]

        homogeneous_points = np.ones(
            (num_points, 4),
            dtype=np.float32,
        )

        homogeneous_points[:, :3] = points

        transformed = (
            transform_matrix @ homogeneous_points.T
        ).T

        return transformed[:, :3]