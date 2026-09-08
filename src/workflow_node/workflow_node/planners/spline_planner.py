import math
import traceback
from typing import List, Tuple

import numpy as np
from rclpy.logging import get_logger
from scipy.interpolate import CubicSpline

from workflow_node.utils.math_utils import quaternion_to_euler

_logger = get_logger('spline_planner')


def generate_spline_path(
    start_pose,
    graph_path: list,
    scale: float = 1.0,
    runway_length: float = 0.0,
    spacing: float = 0.01,
) -> list:
    """Generate a smooth cubic-spline path through graph waypoints.

    Parameters:
        start_pose: Starting pose [x, y, qz, qw] (inserted at front of path).
        graph_path: Remaining waypoints each as [x, y, qz, qw, ...].
        scale: Coordinate scaling factor.
        runway_length: Straight-line extension appended past the final point.
        spacing: Desired arc-length between sampled points (metres).

    Returns:
        List of (x, y) tuples. Returns [start_pose[:2]] as fallback on failure.
    """
    try:
        graph_path.insert(0, start_pose)

        points = [(wp[0] * scale, wp[1] * scale) for wp in graph_path]
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        if len(points) < 2:
            _logger.warning(
                f"Not enough points to generate a spline (got {len(points)})."
            )
            return points

        distances = [0.0]
        for i in range(1, len(points)):
            dist = math.hypot(
                points[i][0] - points[i - 1][0],
                points[i][1] - points[i - 1][1],
            )
            distances.append(distances[-1] + dist)
        total_dist = distances[-1]

        if total_dist < 1e-6:
            _logger.warning(
                "Path has near-zero total length — "
                "returning raw points without spline interpolation."
            )
            return points

        last_wp = graph_path[-1]
        qz, qw = last_wp[2], last_wp[3]
        final_yaw = quaternion_to_euler(qw, 0, 0, qz)[2] + math.pi

        final_slope_x = math.cos(final_yaw)
        final_slope_y = math.sin(final_yaw)

        try:
            cs_x = CubicSpline(
                distances, xs, bc_type=((1, 0), (1, final_slope_x))
            )
            cs_y = CubicSpline(
                distances, ys, bc_type=((1, 0), (1, final_slope_y))
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            _logger.error(
                f"CubicSpline construction failed: {error}. "
                "Falling back to raw points."
            )
            traceback.print_exc()
            return points

        sample_distances = np.arange(0, total_dist, spacing)
        if len(sample_distances) == 0 or sample_distances[-1] < total_dist:
            sample_distances = np.append(sample_distances, total_dist)

        smoothed_coords = []
        for d in sample_distances:
            try:
                smoothed_coords.append((float(cs_x(d)), float(cs_y(d))))
            except Exception as error:
                _logger.warning(
                    f"Spline evaluation failed at d={d}: {error} — skipping point."
                )

        if not smoothed_coords:
            _logger.error("No smoothed points generated — falling back to raw points.")
            return points

        runway_steps = 20
        step_length = runway_length / runway_steps if runway_steps > 0 else 0.0
        if runway_length > 0:
            final_point = smoothed_coords[-1]
            for i in range(1, runway_steps + 1):
                x_new = final_point[0] + i * step_length * math.cos(final_yaw)
                y_new = final_point[1] + i * step_length * math.sin(final_yaw)
                smoothed_coords.append((float(x_new), float(y_new)))

        _logger.info(
            f"Spline generated: {len(smoothed_coords)} points, "
            f"total_dist={total_dist:.3f}m, spacing={spacing}m."
        )
        return smoothed_coords

    except Exception as error:
        _logger.error(f"Unexpected failure in generate_spline_path: {error}")
        traceback.print_exc()
        return [(start_pose[0], start_pose[1])]
