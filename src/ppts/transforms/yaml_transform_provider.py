#!/usr/bin/env python3

import numpy as np

from models.rigid_transform import RigidTransform
from transforms.base_transform_provider import BaseTransformProvider


class YAMLTransformProvider(BaseTransformProvider):
    """
    Returns transforms from YAML configuration.
    """

    def __init__(self, transforms: dict):

        self.transforms = transforms

    def get_transform(
        self,
        source_frame: str,
        target_frame: str,
    ) -> RigidTransform:

        key = (source_frame, target_frame)

        if key not in self.transforms:

            raise RuntimeError(
                f"No transform available from "
                f"{source_frame} to {target_frame}"
            )

        return self.transforms[key]