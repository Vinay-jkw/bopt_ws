#!/usr/bin/env python3

from abc import ABC, abstractmethod

from models.rigid_transform import RigidTransform


class BaseTransformProvider(ABC):
    """
    Abstract interface for all transform providers.
    """

    @abstractmethod
    def get_transform(
        self,
        source_frame: str,
        target_frame: str,
    ) -> RigidTransform:
        """
        Returns transform between two frames.
        """
        pass