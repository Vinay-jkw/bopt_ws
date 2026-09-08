import math
import traceback

from rclpy.logging import get_logger
from rsplan import planner

from workflow_node.utils.math_utils import quaternion_to_euler

_logger = get_logger('rs_path_planner')


def _deg(rad: float) -> float:
    """Radians -> degrees, normalized to (-180, 180] for readable logs."""
    d = math.degrees(rad)
    return (d + 180.0) % 360.0 - 180.0


def _log_segment_diagnostics(current_pose, end_pose_tuple, tr, path) -> None:
    """Log WHY the planner produced the path it did (cusp diagnosis).

    Prints the start/goal geometry, the Dubins-normalized angles that decide the
    path family, the chosen segment 'word' with per-segment drive direction, and
    flags any reverse cusp (the sharp triangle).
    """
    try:
        sx, sy, syaw = current_pose
        ex, ey, eyaw = end_pose_tuple

        dx, dy = ex - sx, ey - sy
        dist = math.hypot(dx, dy)
        bearing = math.atan2(dy, dx)              # direction from start to goal
        alpha = _deg(syaw - bearing)              # start heading vs bearing
        beta = _deg(eyaw - bearing)               # goal heading vs bearing

        segs = [(s.type, int(s.direction), round(s.length, 3)) for s in path.segments]
        directions = [int(s.direction) for s in path.segments]
        n_reversals = sum(
            1 for i in range(1, len(directions)) if directions[i] != directions[i - 1]
        )
        has_reverse = any(d == -1 for d in directions)

        _logger.info(
            "RS-DIAG | start=(%.3f, %.3f, %.1fdeg) goal=(%.3f, %.3f, %.1fdeg) | "
            "dist=%.3fm turn_radius=%.3fm dist/R=%.2f | "
            "alpha=%.1fdeg beta=%.1fdeg | "
            "word=%s total_len=%.3fm | reversals(cusps)=%d has_reverse=%s%s"
            % (
                sx, sy, _deg(syaw), ex, ey, _deg(eyaw),
                dist, tr, (dist / tr if tr else float('inf')),
                alpha, beta,
                segs, getattr(path, 'total_length', float('nan')),
                n_reversals, has_reverse,
                "  <<< CUSP / TRIANGLE HERE" if (n_reversals or has_reverse) else "",
            )
        )
    except Exception as error:  # diagnostics must never break planning
        _logger.warning("RS-DIAG logging failed: %s" % error)


# def generate_rs_path(
#     start_pose: list,
#     graph_path: list,
#     turn_radius: float,
#     rev_drive: bool,
#     SCALE: float = 1.0,
# ) -> list:
#     """Generate a Reeds-Shepp path through a sequence of graph waypoints.

#     Parameters:
#         start_pose: [x, y, qz, qw]
#         graph_path: list of waypoints, each [x, y, qz, qw]
#         turn_radius: minimum turning radius (metres)
#         rev_drive: if True the final waypoint uses a runway; currently both
#                    branches produce the same output (preserved from original)
#         SCALE: coordinate scaling factor

#     Returns:
#         List of (x, y) tuples. Returns [] on failure.
#     """
#     try:
#         rs_path_waypoints = []
#         _logger.debug(f"generate_rs_path: graph_path={graph_path}")
#         for waypoint in graph_path:
#             try:
#                 yaw = quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2]
#             except (IndexError, TypeError, ValueError) as error:
#                 _logger.warning(
#                     f"Failed to compute yaw for waypoint {waypoint}: {error} — skipping."
#                 )
#                 continue

#             if waypoint == graph_path[-1] and rev_drive:
#                 rs_path_waypoints.append([
#                     waypoint[0] * SCALE, waypoint[1] * SCALE, yaw, turn_radius, 0.0 * SCALE
#                 ])
#             elif waypoint == graph_path[-1] and not rev_drive:
#                 rs_path_waypoints.append([
#                     waypoint[0] * SCALE, waypoint[1] * SCALE, yaw, turn_radius, 0.0 * SCALE
#                 ])
#             else:
#                 rs_path_waypoints.append([
#                     waypoint[0] * SCALE, waypoint[1] * SCALE, yaw, turn_radius, 0.0
#                 ])

#         if not rs_path_waypoints:
#             _logger.error("No valid RS waypoints built — returning empty path.")
#             return []

#         step_size = 0.01 * SCALE

#         try:
#             start_yaw = quaternion_to_euler(start_pose[3], 0, 0, start_pose[2])[2]
#         except (IndexError, TypeError, ValueError) as error:
#             _logger.error(f"Failed to compute yaw from start_pose {start_pose}: {error}")
#             traceback.print_exc()
#             return []

#         waypoints = []
#         current_pose = (start_pose[0], start_pose[1], start_yaw)

#         for end_pose in rs_path_waypoints:
#             end_x, end_y, end_yaw, tr, runway_length = end_pose
#             end_pose_tuple = (end_x, end_y, end_yaw)
#             try:
#                 path = planner.path(current_pose, end_pose_tuple, tr, runway_length, step_size)
#                 _log_segment_diagnostics(current_pose, end_pose_tuple, tr, path)
#                 waypoints.extend([(wp.x, wp.y) for wp in path.waypoints()])
#             except Exception as error:
#                 _logger.error(
#                     f"planner.path failed from {current_pose} to {end_pose_tuple} "
#                     f"(tr={tr}, runway={runway_length}): {error}"
#                 )
#                 traceback.print_exc()
#             current_pose = end_pose_tuple

#         if not waypoints:
#             _logger.warning("RS path produced no waypoints.")

#         return waypoints

#     except Exception as error:
#         _logger.error(f"Unexpected failure in generate_rs_path: {error}")
#         traceback.print_exc()
#         return []



def generate_rs_path(start_pose, graph_path, turn_radius, rev_drive, SCALE=1):

        rs_path_waypoints = []
        print('gp:::',graph_path)
        for waypoint in graph_path:
            if waypoint == graph_path[-1] and rev_drive == True:
                rs_path_waypoints.append([waypoint[0]*SCALE, waypoint[1]*SCALE, quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2], turn_radius, 0.0*SCALE])
            elif waypoint == graph_path[-1] and rev_drive == False:
                rs_path_waypoints.append([waypoint[0]*SCALE, waypoint[1]*SCALE, quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2], turn_radius, 0.0*SCALE])
            else:
                rs_path_waypoints.append([waypoint[0]*SCALE, waypoint[1]*SCALE, quaternion_to_euler(waypoint[3], 0, 0, waypoint[2])[2], turn_radius, 0.0])


        step_size = 0.01 * SCALE

        waypoints = []
        current_pose = (*start_pose[:2], quaternion_to_euler(start_pose[3],0,0, start_pose[2])[2])

        for end_pose in rs_path_waypoints:
            end_x, end_y, end_yaw, turn_radius, runway_length = end_pose
            end_pose_tuple = (end_x, end_y, end_yaw)
            path = planner.path(current_pose, end_pose_tuple, turn_radius, runway_length, step_size)
            waypoints.extend([(wp.x, wp.y) for wp in path.waypoints()])
            current_pose = end_pose_tuple

        return waypoints