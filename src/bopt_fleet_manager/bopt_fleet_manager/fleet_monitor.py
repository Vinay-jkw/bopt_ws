#!/usr/bin/env python3
"""Read-only fleet pose and separation monitor for BOPT Gazebo robots.

This node intentionally does not publish commands or change controller parameters.
It reports pose freshness, pairwise distance, and advisory threshold states only.
"""
from dataclasses import dataclass
import math
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


@dataclass
class RobotSnapshot:
    x: float
    y: float
    received_at: float


def pair_advisory(distance: float, previous_state: str,
                  stop_distance: float, release_distance: float) -> str:
    """Return an advisory state with hysteresis; this does not stop robots."""
    if previous_state == "NEAR":
        return "CLEAR" if distance >= release_distance else "NEAR"
    return "NEAR" if distance <= stop_distance else "CLEAR"


class FleetMonitor(Node):
    def __init__(self):
        super().__init__("bopt_fleet_monitor")

        self.declare_parameter("robots", ["robot001", "robot002", "robot003"])
        self.declare_parameter("stop_distance", 4.0)
        self.declare_parameter("release_distance", 4.5)
        self.declare_parameter("pose_timeout_sec", 1.5)
        self.declare_parameter("report_period_sec", 1.0)

        self.robots = list(self.get_parameter("robots").value)
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.release_distance = float(self.get_parameter("release_distance").value)
        self.pose_timeout = float(self.get_parameter("pose_timeout_sec").value)
        report_period = float(self.get_parameter("report_period_sec").value)

        if not self.robots:
            raise ValueError("'robots' must contain at least one namespace")
        if self.stop_distance <= 0 or self.release_distance < self.stop_distance:
            raise ValueError("Require 0 < stop_distance <= release_distance")
        if self.pose_timeout <= 0 or report_period <= 0:
            raise ValueError("pose_timeout_sec and report_period_sec must be > 0")

        self.snapshots: Dict[str, RobotSnapshot] = {}
        self.robot_states: Dict[str, str] = {}
        self.current_tasks: Dict[str, str] = {}
        self.pair_states: Dict[Tuple[str, str], str] = {}

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        for robot in self.robots:
            ns = "/" + robot.strip("/")
            self.create_subscription(
                PoseStamped, f"{ns}/current_pose",
                lambda msg, name=robot: self._pose_callback(name, msg), qos
            )
            self.create_subscription(
                String, f"{ns}/state",
                lambda msg, name=robot: self._string_callback(self.robot_states, name, msg), 10
            )
            self.create_subscription(
                String, f"{ns}/byd/current_task",
                lambda msg, name=robot: self._string_callback(self.current_tasks, name, msg), 10
            )

        self.timer = self.create_timer(report_period, self._report)
        self.get_logger().info(
            "READ-ONLY MODE: no command publishers and no parameter changes. "
            f"Robots={self.robots}; advisory stop/release={self.stop_distance:.2f}/"
            f"{self.release_distance:.2f} m; pose timeout={self.pose_timeout:.2f} s."
        )

    def _pose_callback(self, robot: str, msg: PoseStamped) -> None:
        self.snapshots[robot] = RobotSnapshot(
            x=float(msg.pose.position.x),
            y=float(msg.pose.position.y),
            received_at=time.monotonic(),
        )

    @staticmethod
    def _string_callback(store: dict, robot: str, msg: String) -> None:
        store[robot] = msg.data.strip()

    def _report(self) -> None:
        now = time.monotonic()
        lines = ["--- BOPT FLEET MONITOR (READ ONLY) ---"]
        valid = {}
        for robot in self.robots:
            snap = self.snapshots.get(robot)
            age = (now - snap.received_at) if snap else math.inf
            if snap and age <= self.pose_timeout:
                valid[robot] = snap
                pose_text = f"x={snap.x:7.2f} y={snap.y:7.2f} age={age:4.1f}s"
            else:
                pose_text = "POSE STALE/MISSING"
            state = self.robot_states.get(robot, "unknown")
            task = self.current_tasks.get(robot, "unknown")
            lines.append(f"{robot:10s} {pose_text:29s} state={state!r} task={task!r}")

        for i, a in enumerate(self.robots):
            for b in self.robots[i + 1:]:
                key = (a, b)
                if a not in valid or b not in valid:
                    self.pair_states[key] = "UNKNOWN"
                    lines.append(f"PAIR {a} <-> {b}: UNKNOWN (missing/stale pose)")
                    continue
                pa, pb = valid[a], valid[b]
                distance = math.hypot(pa.x - pb.x, pa.y - pb.y)
                old = self.pair_states.get(key, "CLEAR")
                state = pair_advisory(
                    distance, old, self.stop_distance, self.release_distance
                )
                self.pair_states[key] = state
                if state == "NEAR":
                    label = "ADVISORY NEAR — operator/controller action required"
                else:
                    label = "clear"
                lines.append(f"PAIR {a} <-> {b}: {distance:.2f} m — {label}")

        lines.append("NOTE: Advisory only; this node does not stop or resume robots.")
        self.get_logger().info("\n" + "\n".join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = FleetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
