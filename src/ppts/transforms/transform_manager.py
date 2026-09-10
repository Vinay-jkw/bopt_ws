#!/usr/bin/env python3

from transforms.base_transform_provider import BaseTransformProvider
from models.rigid_transform import RigidTransform


class TransformManager:
    """
    High-level interface for retrieving transforms.

    The manager delegates transform lookup to the configured
    provider (TF2, YAML, Calibration, etc.).

    It acts as the single access point for all transform
    requests and can later provide caching, diagnostics,
    fallback providers, statistics, etc.
    """

    def __init__(
        self,
        provider: BaseTransformProvider,
    ) -> None:

        self._provider = provider

    # ---------------------------------------------------------

    @property
    def provider(self) -> BaseTransformProvider:
        """
        Returns the configured transform provider.
        """

        return self._provider

    # ---------------------------------------------------------

    def get_transform(
        self,
        source_frame: str,
        target_frame: str,
    ) -> RigidTransform:
        """
        Retrieve the transform between two coordinate frames.

        Parameters
        ----------
        source_frame : str
            Source coordinate frame.

        target_frame : str
            Target coordinate frame.

        Returns
        -------
        Transform
            Rigid body transform.
        """

        return self._provider.get_transform(
            source_frame=source_frame,
            target_frame=target_frame,
        )