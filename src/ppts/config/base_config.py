#!/usr/bin/env python3

from pathlib import Path
from typing import Any, Optional

import yaml
from rclpy.node import Node


class BaseConfig:
    """
    Base configuration loader for PPTS.

    Configuration precedence:
        1. YAML configuration
        2. ROS parameter server
        3. Caller-provided default

    Nested YAML values are accessed with dot notation.
    """

    def __init__(
        self,
        node: Node,
        config_file: Optional[str] = "/home/ankit/Desktop/Ankit/Test/APDS/src/ppts/config/ppts.yaml",
    ):
        self.node = node

        # Keep the configuration file next to the config package by default.
        self.config_file = Path(config_file) if config_file else (
            Path(__file__).resolve().with_name("ppts.yaml")
        )

        self.config = self._load_yaml()

    # ------------------------------------------------------------------
    # YAML
    # ------------------------------------------------------------------

    def _load_yaml(self) -> dict:
        if not self.config_file.exists():
            
            return {}
        print(f"Config Found : {self.config_file}")
        with self.config_file.open("r", encoding="utf-8") as file:
            yaml_data = yaml.safe_load(file) or {}

        if not isinstance(yaml_data, dict):
            raise ValueError(
                f"Invalid YAML root in '{self.config_file}': expected mapping"
            )

        parameters = (
            yaml_data.get("ppts", {})
            .get("ros__parameters", {})
        )

        if not isinstance(parameters, dict):
            raise ValueError(
                f"Invalid 'ppts.ros__parameters' in '{self.config_file}'"
            )

        return parameters

    def _get_from_yaml(self, name: str) -> Any:
        current = self.config

        for key in name.split("."):
            if not isinstance(current, dict) or key not in current:
                return None
            current = current[key]

        return current

    # ------------------------------------------------------------------
    # ROS parameters
    # ------------------------------------------------------------------

    def _get_from_ros(self, name: str, default: Any) -> Any:
        self.node.declare_parameter(name, default)
        return self.node.get_parameter(name).value

    # ------------------------------------------------------------------
    # Public getters
    # ------------------------------------------------------------------

    def get(self, name: str, default: Any = None) -> Any:
        """
        Get a configuration value.

        YAML has priority over ROS parameters. A YAML value of None is
        treated as missing and therefore falls back to ROS/default.
        """
        value = self._get_from_yaml(name)

        if value is not None:
            return value

        return self._get_from_ros(name, default)

    def get_bool(self, name: str, default: bool) -> bool:
        value = self.get(name, default)
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            normalized = value.strip().lower()
            if normalized in {"true", "1", "yes", "on"}:
                return True
            if normalized in {"false", "0", "no", "off"}:
                return False
        raise ValueError(f"'{name}' must be a boolean")

    def get_int(self, name: str, default: int) -> int:
        value = self.get(name, default)
        try:
            return int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"'{name}' must be an integer") from exc

    def get_float(self, name: str, default: float) -> float:
        value = self.get(name, default)
        try:
            return float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"'{name}' must be a number") from exc

    def get_str(self, name: str, default: str) -> str:
        value = self.get(name, default)
        if value is None:
            return default
        return str(value)