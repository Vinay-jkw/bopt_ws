import json
import math
import traceback
from typing import List, Optional

import networkx as nx
from rclpy.logging import get_logger

from workflow_node.utils.math_utils import euler_to_quaternion, quaternion_to_euler

_logger = get_logger('graph_planner')


def filter_points_within_radius(
    location_to_compare: list,
    points: list,
    radius: float,
) -> list:
    """Return only those points whose XY distance from location exceeds radius."""
    x, y = location_to_compare[0], location_to_compare[1]
    filtered_points = []
    for point in points:
        try:
            distance = math.sqrt(
                (point[0] - x) ** 2 + (point[1] - y) ** 2
            )
            if distance > radius:
                filtered_points.append(point)
        except (TypeError, IndexError) as error:
            _logger.warning(f"Skipping malformed point {point}: {error}")
    return filtered_points


def find_path_and_angles_between_points(
    G: nx.Graph,
    point1,
    point2,
    path_publisher=None,
    publish_path=False,
) -> list:
    """Find shortest graph path between two XY coordinates, returning poses.

    Each returned pose is [x, y, qz, qw].  Orientation is the direction of
    travel along each graph edge (+ π so the robot faces forward).

    If publish_path is truthy and path_publisher is provided, the node list is
    published on the /path topic.  When publish_path is a string it is appended
    as a terminal node label.

    Returns an empty list if no path can be found (instead of raising).
    """

    def euclidean_distance(coord1, coord2):
        return math.sqrt(
            (coord1[0] - coord2[0]) ** 2 + (coord1[1] - coord2[1]) ** 2
        )

    def closest_node(G, point, k=10):
        for i, node in enumerate(
            sorted(
                G.nodes,
                key=lambda n: euclidean_distance(G.nodes[n]['position'], point)
            )
        ):
            if i < k:
                yield node

    try:
        node1 = tuple(closest_node(G, point1))
        node2 = tuple(closest_node(G, point2))

        if not node1 or not node2:
            _logger.error(
                f"Could not find candidate nodes near "
                f"point1={point1} or point2={point2}."
            )
            return []

        selected_n1, selected_n2 = node1[0], node2[0]
        found_path = False

        for n1 in node1:
            for n2 in node2:
                _logger.debug(f"Trying nodes: {n1} → {n2}")
                try:
                    nx.shortest_path_length(G, source=n1, target=n2)
                    selected_n1, selected_n2 = n1, n2
                    found_path = True
                    _logger.debug(f"Found viable node pair: {n1} → {n2}")
                    break
                except nx.NetworkXNoPath:
                    _logger.debug(f"No path between nodes: {n1} → {n2}")
                    continue
                except nx.NodeNotFound as error:
                    _logger.warning(f"Node not found during path search: {error}")
                    continue
            if found_path:
                _logger.debug(f"Breaking outer loop with node: {n1}")
                break
            else:
                _logger.debug(f"No valid path found for node: {n1}")

        if not found_path:
            _logger.error(
                f"No graph path found between point1={point1} and point2={point2} "
                "after exhausting all candidate node pairs."
            )
            return []

    except Exception as error:
        _logger.error(f"Node search failed: {error}")
        traceback.print_exc()
        return []

    try:
        path = nx.shortest_path(G, source=selected_n1, target=selected_n2)
    except (nx.NetworkXNoPath, nx.NodeNotFound) as error:
        _logger.error(
            f"shortest_path failed for ({selected_n1}, {selected_n2}): {error}"
        )
        return []

    # Publish path for visualisation (best-effort, never abort on failure)
    if path_publisher is not None and publish_path:
        try:
            from std_msgs.msg import String
            if isinstance(publish_path, str):
                path_publisher.publish(
                    String(data=json.dumps({"path": path + [publish_path]}))
                )
            else:
                path_publisher.publish(
                    String(data=json.dumps({"path": path}))
                )
        except Exception as error:
            _logger.warning(f"Failed to publish path: {error}")

    try:
        path_coords = [G.nodes[node]['position'] for node in path]

        path_coords_with_quaternions = []
        for i in range(len(path_coords) - 1):
            start = path_coords[i]
            end = path_coords[i + 1]
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            angle_rad = math.atan2(dy, dx) + math.pi
            quaternion = euler_to_quaternion(0, 0, angle_rad)
            _logger.debug(f"angle: {math.degrees(angle_rad):.2f}°, quaternion: {quaternion}")
            path_coords_with_quaternions.append((start, quaternion[-2:]))

        if path_coords_with_quaternions:
            last_quaternion = path_coords_with_quaternions[-1][1]
            path_coords_with_quaternions.append((path_coords[-1], last_quaternion))

        result = []
        for coord, quaternion in path_coords_with_quaternions:
            result.append(list(coord) + list(quaternion))

        return result

    except Exception as error:
        _logger.error(f"Failed to build path poses for path {path}: {error}")
        traceback.print_exc()
        return []
