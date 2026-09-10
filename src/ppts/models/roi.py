from dataclasses import dataclass


@dataclass(slots=True)
class ROI:
    id: str
    xmin: float
    xmax: float
    ymin: float
    ymax: float