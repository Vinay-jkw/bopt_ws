#!/usr/bin/env python3

from rclpy.time import Time

from tf2_ros import Buffer, TransformException

import numpy as np
from scipy.spatial.transform import Rotation

from transforms.base_transform_provider import BaseTransformProvider
from models.rigid_transform import RigidTransform


class TFTransformProvider(BaseTransformProvider):
    """
    TF2 implementation of BaseTransformProvider.

    Converts TF2 transforms into the framework's
    internal rigid transform representation.
    """

    def __init__(
        self,
        tf_buffer: Buffer,
    ) -> None:

        self._tf_buffer = tf_buffer

    # ---------------------------------------------------------

    def get_transform(
        self,
        source_frame: str,
        target_frame: str,
    ) -> RigidTransform:

        try:

            tf = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=Time(),
            )

        except TransformException as ex:

            raise RuntimeError(
                f"Failed TF lookup "
                f"{source_frame} -> {target_frame}\n{ex}"
            )

        return self._build_transform(tf)

    # ---------------------------------------------------------

    @staticmethod
    def _build_transform(tf_msg) -> RigidTransform:

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation

        matrix = np.eye(4, dtype=np.float64)

        matrix[:3, :3] = Rotation.from_quat(
            [q.x, q.y, q.z, q.w]
        ).as_matrix()

        matrix[:3, 3] = [
            t.x,
            t.y,
            t.z,
        ]

        return RigidTransform(matrix)