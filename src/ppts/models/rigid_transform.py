#!/usr/bin/env python3

from dataclasses import dataclass
import numpy as np


@dataclass(slots=True)
class RigidTransform:
    """
    Represents a rigid body transform in SE(3).

    The transform is stored as a homogeneous 4×4 matrix.
    """

    matrix: np.ndarray

    @property
    def rotation(self) -> np.ndarray:
        return self.matrix[:3, :3]

    @property
    def translation(self) -> np.ndarray:
        return self.matrix[:3, 3]