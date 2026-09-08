# Dataclass models for the controller configuration. Each class maps directly to
# a section in nmpc_config.yaml and is populated by config_loader.py.

from dataclasses import dataclass
from typing import Optional, Dict, Any


@dataclass
class LookaheadConfig:
    type: str                                  # "fixed" or "dynamic"
    value: Optional[float] = None             # used when type is fixed
    base: float = 0.3                         # base lookahead before velocity scaling
    velocity_factor: Optional[float] = None   # scales lookahead with current speed
    min: Optional[float] = None               # lower clamp on the velocity-scaled distance
    max: Optional[float] = None               # upper clamp on the velocity-scaled distance
    offset: float = 0.5                       # always added after clamping (handles AGV reverse)
    curvature_window: float = 0.2             # path segment (m) used for curvature check inside get_target_point
    curvature_boost: Optional[float] = None   # extra distance added on straight sections
    straight_threshold: float = 0.02          # curvature below this is treated as straight


@dataclass
class SafetyConfig:
    type: str                              # "sqrt" or "sigmoid"
    mu: float                             # friction coefficient for speed limit formula
    sigmoid_k: Optional[float] = None    # steepness of the sigmoid speed gate
    sigmoid_scale: float = 1.0
    exp_decay_k: Optional[float] = None  # exponential decay rate for high-curvature sections
    curvature_window: float = 1.0        # path segment (m) used for curvature in safety calculation
    cte_gain: Optional[float] = None          # global CTE speed gate — applies everywhere (None = disabled)
    approach_cte_gain: Optional[float] = None # CTE penalty inside slow-down zone only — no effect during normal travel


@dataclass
class FeaturesConfig:
    use_runway: bool = False             # extend path beyond goal to stabilise final approach
    runway_length: float = 0.3
    use_picking: bool = False            # enable slow-down ramp near the goal
    pallet_gate: bool = False            # ignore pallet detection within 0.5 m of goal
    stop_on_pallet_loaded: bool = False  # stop immediately when CAN signals load complete
    sensor: Optional[str] = None        # "ds_field" or "can_odot"


@dataclass
class Config:
    profile: str
    path_file: str
    goal_tolerance: float

    max_velocity: float
    min_velocity: float
    slow_down_distance: float

    parking: bool = False                        # when True, pallet_loaded triggers charging approach instead of immediate stop
    stopping_velocity: Optional[float] = None   # target speed at the end of the slow-down ramp
    picking_velocity: Optional[float] = None    # overrides stopping_velocity for pick profiles

    lookahead: LookaheadConfig = None
    safety: SafetyConfig = None
    features: FeaturesConfig = None

    lookup_table: str = None
