from dataclasses import dataclass, field
from typing import List


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float
    

@dataclass
class PalletPath:
    waypoints: List[Waypoint] = field(default_factory=list)
    valid: bool = False
    reason: str = ""