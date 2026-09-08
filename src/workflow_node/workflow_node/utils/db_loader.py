import os
import pickle
import sqlite3
import traceback
from typing import Optional

import numpy as np
import pandas as pd
from rclpy.logging import get_logger

_logger = get_logger('db_loader')


def fetch_location_data_from_db(db_path: str) -> tuple:
    """Load dock / station pose tables from the workflow node SQLite database.

    Returns:
        (left_easy_dock_dict, right_easy_dock_dict,
         dock_location_dict, dock_station_end_line_dict)

    Raises:
        FileNotFoundError: if the database file does not exist.
        sqlite3.Error: on any database query failure.
    """
    if not os.path.isfile(db_path):
        raise FileNotFoundError(
            f"Workflow node database not found: '{db_path}'. "
            "Check the 'wn_db_file' path in the YAML config."
        )

    _logger.info(f"Connecting to database: {db_path}")
    conn = None
    try:
        conn = sqlite3.connect(db_path)

        queries = {
            'left_easy_dock': 'SELECT * FROM left_easy_dock;',
            'right_easy_dock': 'SELECT * FROM right_easy_dock;',
            'dock_location': 'SELECT * FROM dock_location;',
            'dock_station_end_line': 'SELECT * FROM dock_station_end_line;',
        }

        dataframes = {}
        for table_name, query in queries.items():
            try:
                dataframes[table_name] = pd.read_sql(query, conn)
            except Exception as error:
                raise sqlite3.Error(
                    f"Failed to query table '{table_name}' from '{db_path}': {error}"
                ) from error

        _logger.info(f"Tables loaded: {list(dataframes.keys())}")
        _logger.debug(f"Raw dataframes: {dataframes}")

    except sqlite3.Error as error:
        _logger.error(f"Database access failed for '{db_path}': {error}")
        traceback.print_exc()
        raise
    finally:
        if conn is not None:
            conn.close()

    def create_location_dict(df, name_col, coord_cols):
        return (
            df.set_index(name_col)[coord_cols]
            .apply(lambda row: row.tolist(), axis=1)
            .to_dict()
        )

    try:
        left_easy_dock_dict = create_location_dict(
            dataframes['left_easy_dock'],
            name_col='easy_dock_name',
            coord_cols=['x', 'y', 'z', 'W'],
        )
        right_easy_dock_dict = create_location_dict(
            dataframes['right_easy_dock'],
            name_col='easy_dock_name',
            coord_cols=['x', 'y', 'z', 'W'],
        )
        dock_location_dict = create_location_dict(
            dataframes['dock_location'],
            name_col='dock_name',
            coord_cols=['x_m', 'y_m', 'pose_z', 'pose_w'],
        )
        dock_station_end_line_dict = create_location_dict(
            dataframes['dock_station_end_line'],
            name_col='station_name',
            coord_cols=['station_x', 'station_y', 'pose_z', 'pose_w'],
        )
    except (KeyError, ValueError) as error:
        _logger.error(f"Failed to build location dicts from '{db_path}': {error}")
        traceback.print_exc()
        raise

    _logger.info(
        f"Location data loaded successfully. "
        f"Dock locations: {list(dock_location_dict.keys())}"
    )
    return (
        left_easy_dock_dict,
        right_easy_dock_dict,
        dock_location_dict,
        dock_station_end_line_dict,
    )


def get_locations_and_waypoints(db_path: str) -> tuple:
    """Legacy loader for the old Vehicles.db waypoint tables (kept for reference)."""
    if not os.path.isfile(db_path):
        raise FileNotFoundError(
            f"Waypoints database not found: '{db_path}'"
        )

    connection = None
    try:
        connection = sqlite3.connect(db_path)
        cur = connection.cursor()

        station_locations = {}
        flow_matrix = {}
        flow_matrix_park = {}
        rev_flow_matrix = {}
        easy_station_locations = {}
        easy_station_location_rev = {}
        doc_station_locations = {}
        pick_flow = {}
        rev_pick_flow = {}

        for rows in cur.execute("SELECT * FROM location"):
            station_locations[rows[0]] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]
        i = 0
        for rows in cur.execute("SELECT * FROM waypoints"):
            flow_matrix[i] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]
            i += 1

        l = 0
        for rows in cur.execute("SELECT * FROM waypoints_pick"):
            pick_flow[l] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]
            l += 1

        m = 14
        for rows in cur.execute("SELECT * FROM rev_waypoints_pick"):
            rev_pick_flow[m] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]
            m -= 1

        k = 70
        for rows in cur.execute("SELECT * FROM waypoints_park"):
            flow_matrix_park[k] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]
            k -= 1

        j = 73
        for rows in cur.execute("SELECT * FROM waypoints_rev"):
            rev_flow_matrix[j] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]
            j -= 1

        _logger.debug(f"pick_flow: {pick_flow}")

        for rows in cur.execute("SELECT * FROM easy_station_loc"):
            easy_station_locations[rows[0]] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]

        for rows in cur.execute("SELECT * FROM easy_station_loc_rev"):
            easy_station_location_rev[rows[0]] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]

        for rows in cur.execute("SELECT * FROM doc_station_loc"):
            doc_station_locations[rows[0]] = [
                float(rows[1]), float(rows[2]), float(rows[3]), float(rows[4])
            ]

        _logger.debug(
            f"station_locations={station_locations}, easy_station_locations={easy_station_locations}, "
            f"flow_matrix={flow_matrix}, doc_station_locations={doc_station_locations}"
        )

    except sqlite3.Error as error:
        _logger.error(f"Failed to load legacy waypoints from '{db_path}': {error}")
        traceback.print_exc()
        raise
    finally:
        if connection is not None:
            connection.close()

    return (
        station_locations,
        easy_station_locations,
        flow_matrix,
        doc_station_locations,
        rev_flow_matrix,
        easy_station_location_rev,
        flow_matrix_park,
        pick_flow,
        rev_pick_flow,
    )


def find_nearest_key(lookup_table, position, initial_tolerance=0.01,
                     max_tolerance=0.1, tolerance_increment=0.01):
    """Find the nearest entry in a lookup table with velocity direction check."""
    tolerance = initial_tolerance

    while tolerance <= max_tolerance:
        _logger.debug(f"find_nearest_key: trying tolerance={tolerance:.4f}")
        potential_keys = []

        for key in lookup_table.keys():
            position_difference = np.sqrt(
                (key[0] - position[0]) ** 2 + (key[1] - position[1]) ** 2
            )
            vel = lookup_table[key][1]

            if position_difference <= tolerance and (
                (position[0] > 0 and vel > 0) or (position[0] <= 0 and vel <= 0)
            ):
                potential_keys.append(key)

        if potential_keys:
            nearest_key = min(
                potential_keys, key=lambda k: lookup_table[k][2]
            )
            return nearest_key

        tolerance += tolerance_increment

    return None


def get_precise_control_key(ws_path: str, robot_state: str, tf_pose) -> tuple:
    """Load lookup table for the given fork state and return steering/vel/dist."""
    _logger.debug(f"get_precise_control_key: going to -> {tf_pose}")

    if robot_state == 'up':
        table_path = os.path.join(
            ws_path,
            'src/nmpc_controller/nmpc_controller/data/mpc_lookup_table_1.542.pkl',
        )
    elif robot_state == 'down':
        table_path = os.path.join(
            ws_path,
            'src/nmpc_controller/nmpc_controller/data/mpc_lookup_table_1.544.pkl',
        )
    else:
        _logger.warning(f"Unknown robot_state '{robot_state}' — returning zeros")
        return 0.0, 0.0, 0.0

    if not os.path.isfile(table_path):
        _logger.error(f"Lookup table not found: '{table_path}'")
        return 0.0, 0.0, 0.0

    try:
        with open(table_path, 'rb') as f:
            loaded_lookup_table = pickle.load(f)
    except (FileNotFoundError, pickle.UnpicklingError, EOFError) as error:
        _logger.error(f"Failed to load lookup table '{table_path}': {error}")
        traceback.print_exc()
        return 0.0, 0.0, 0.0

    position = tf_pose[:2]
    nearest_key = find_nearest_key(loaded_lookup_table, position)

    if nearest_key is None:
        _logger.warning("No solution found within given tolerances.")
        return 0.0, 0.0, 0.0

    steering_angle = np.rad2deg(loaded_lookup_table[nearest_key][0])
    vel = loaded_lookup_table[nearest_key][1]
    dist = loaded_lookup_table[nearest_key][2]

    return steering_angle, vel, dist
