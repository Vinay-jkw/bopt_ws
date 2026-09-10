#!/usr/bin/env python3

from dataclasses import dataclass, field
from models.roi import ROI
import numpy as np


@dataclass
class PointCloud:

    points: np.ndarray = field(
        default_factory=lambda: np.empty(
            (0, 2),
            dtype=np.float32,
        )
    )

    frame_id: str = ""

    source: str = ""

    timestamp: float = 0.0

    @property
    def size(self) -> int:
        return len(self.points)

    @property
    def is_empty(self) -> bool:
        return self.points.size == 0

    def copy_with_points(
        self,
        points: np.ndarray,
    ) -> "PointCloud":

        return PointCloud(
            points=points,
            frame_id=self.frame_id,
            source=self.source,
            timestamp=self.timestamp,
        )

@dataclass
class ROICloud:
    roi: ROI
    cloud: PointCloud | None = None