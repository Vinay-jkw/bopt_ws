# Loads the YAML config file and merges the common defaults with the
# named profile so the controller receives a single flat Config object.

import yaml
from .models import Config, LookaheadConfig, SafetyConfig, FeaturesConfig


def load_config(path, profile):
    with open(path, "r") as f:
        data = yaml.safe_load(f)["nmpc_controller"]

    # Profile values override common values where both define the same key.
    merged = {**data["common"], **data["profiles"][profile]}

    return Config(
        profile=profile,
        path_file=merged["path_file"],
        goal_tolerance=merged["goal_tolerance"],

        max_velocity=merged["max_velocity"],
        min_velocity=merged["min_velocity"],
        slow_down_distance=merged["slow_down_distance"],

        stopping_velocity=merged.get("stopping_velocity"),
        picking_velocity=merged.get("picking_velocity"),

        lookup_table=merged["lookup_table"],

        lookahead=LookaheadConfig(**merged.get("lookahead", {})),
        safety=SafetyConfig(**merged.get("safety", {})),
        features=FeaturesConfig(**merged.get("features", {})),
    )