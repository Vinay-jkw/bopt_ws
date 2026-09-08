import os
import traceback

import networkx as nx
import numpy as np
from rclpy.logging import get_logger

_logger = get_logger('graph_loader')


def load_graphml(graphml_path: str) -> nx.Graph:
    """Load a graphml file and parse node position strings into float tuples.

    Raises:
        FileNotFoundError: if the graphml file does not exist.
        Exception: if the file cannot be parsed as a valid graphml graph.
    """
    if not os.path.isfile(graphml_path):
        raise FileNotFoundError(
            f"GraphML map file not found: '{graphml_path}'. "
            "Check the 'graphml_file' path in the YAML config."
        )

    _logger.info(f"Loading graph from: {graphml_path}")
    try:
        G = nx.read_graphml(graphml_path)
    except Exception as error:
        _logger.error(f"Failed to parse graphml file '{graphml_path}': {error}")
        traceback.print_exc()
        raise

    parse_errors = 0
    for node, data in G.nodes(data=True):
        pos_str = data.get('position', '')
        if pos_str:
            try:
                G.nodes[node]['position'] = tuple(map(float, pos_str.split(',')))
            except (ValueError, TypeError) as error:
                parse_errors += 1
                _logger.warning(
                    f"Could not parse position '{pos_str}' for node '{node}': {error}"
                )

    if parse_errors:
        _logger.warning(
            f"{parse_errors} node(s) had unparseable position data and were skipped."
        )

    _logger.info(
        f"Graph loaded: {G.number_of_nodes()} nodes, {G.number_of_edges()} edges."
    )
    return G


def extract_waypoints(G: nx.Graph) -> tuple:
    """Return (waypoints_array, waypoints_positions_array) from a loaded graph.

    Only nodes that have a valid 'position' attribute (tuple of floats) are
    included in the positions array.
    """
    try:
        waypoints = np.array(list(G.nodes))
        waypoints_with_pos = [
            node for node in G.nodes
            if isinstance(G.nodes[node].get('position'), tuple)
        ]
        if len(waypoints_with_pos) < len(waypoints):
            missing = len(waypoints) - len(waypoints_with_pos)
            _logger.warning(
                f"{missing} node(s) have no 'position' attribute "
                "and are excluded from the waypoints array."
            )

        waypoints = np.array(waypoints_with_pos)
        waypoints_positions = np.array(
            [G.nodes[node]['position'] for node in waypoints_with_pos]
        )
        _logger.info(f"Extracted {len(waypoints)} positioned waypoints.")
        return waypoints, waypoints_positions

    except Exception as error:
        _logger.error(f"Failed to extract waypoints from graph: {error}")
        traceback.print_exc()
        raise
