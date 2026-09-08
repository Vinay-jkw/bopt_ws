import os
import pickle
import subprocess
import traceback

import numpy as np
from std_msgs.msg import String

from workflow_node.constants import FIELD_NORMAL, FIELD_PICKDROP, PROFILE_FAST, PROFILE_SLOW
from workflow_node.planners import docking_planner, graph_planner, rs_path_planner, spline_planner
from workflow_node.states.current_location import get_current_pose
from workflow_node.utils.math_utils import euler_to_quaternion


class FastDriveMixin:
    """Mixin providing spline path calculation and the fast-drive navigation loop.

    Requires the host class to expose:
        self.node             — WorkflowHandler instance
        self._logger          — ROS logger
        self.is_near_location — proximity check method
    """

    def _calculate_spline_path(
        self,
        point1,
        point2,
        point1_filter_radius: float = None,
        point2_filter_radius: float = None,
        skip_easy_dock: bool = False,
        publish_path=True,
    ) -> list:
        """Build a smooth spline path from point1 to point2 via the graph."""
        node = self.node
        cfg = node.cfg
        if point1_filter_radius is None:
            point1_filter_radius = cfg.thresholds.point1_filter_radius
        if point2_filter_radius is None:
            point2_filter_radius = cfg.thresholds.point2_filter_radius

        path_coords = graph_planner.find_path_and_angles_between_points(
            node.G_loaded, point1, point2,
            path_publisher=node.path_publisher,
            publish_path=publish_path,
        )
        last_point = path_coords[-1] if path_coords else get_current_pose()
        path_coords = graph_planner.filter_points_within_radius(
            point1, path_coords, point1_filter_radius
        )
        path_coords = graph_planner.filter_points_within_radius(
            point2, path_coords, point2_filter_radius
        )

        if path_coords:
            waypoints = [get_current_pose()] + path_coords
            end_easy_dock = self._resolve_easy_dock(path_coords[-1], skip_easy_dock, last_point)
        else:
            current_pose = get_current_pose()
            waypoints = [current_pose]
            end_easy_dock = self._resolve_easy_dock(current_pose, skip_easy_dock, last_point)

        waypoints.append(end_easy_dock)

        return spline_planner.generate_spline_path(
            waypoints[0], waypoints[1:],
            spacing=self.node.cfg.planner.spline_spacing,
        )

    def _resolve_easy_dock(self, reference_pose, skip_easy_dock: bool, fallback):
        """Return the appropriate easy dock pose or the fallback waypoint."""
        if skip_easy_dock:
            return fallback
        node = self.node
        try:
            return docking_planner.find_left_or_right_dock(
                node.left_easy_dock, node.right_easy_dock,
                node.dock_location, reference_pose, is_last_pose=True,
            )
        except Exception as error:
            self._logger.warning(
                f"find_left_or_right_dock failed: {error} — using left dock."
            )
            traceback.print_exc()
            return node.left_easy_dock

    def _run_fast_drive_loop(self, movement: str) -> None:
        """Execute the rerouting / spline fast-drive loop until arrival at dock."""
        node = self.node
        state = node.state
        cfg = node.cfg
        rs_path_file = cfg.paths.rs_path_file
        nmpc_cmd = list(cfg.subprocesses.nmpc_controller)

        while True:
            if (
                state.nmpc_fast_process is not None
                and state.nmpc_fast_process.poll() is None
            ):
                if state.reroutor_startup_conflict is not None:
                    node.reroute_handler.handle_conflict_action(
                        state.reroutor_startup_conflict
                    )
                    state.reroutor_startup_conflict = None
                state.nmpc_fast_process.wait()
                if state.moving_to_destination:
                    break

            self._logger.debug("Waiting for fast drive to complete...")

            if state.reroute_locations:
                action = self._handle_reroute_segment(state, cfg, rs_path_file, nmpc_cmd)
            else:
                action = self._handle_main_drive_segment(
                    movement, state, node, cfg, rs_path_file, nmpc_cmd
                )

            if action == 'break':
                break

    def _handle_reroute_segment(self, state, cfg, rs_path_file, nmpc_cmd) -> str:
        """Handle one reroute step. Returns 'break', 'skip', or 'ok'."""
        try:
            point1 = get_current_pose()
        except RuntimeError as error:
            self._logger.error(f"Cannot get pose during reroute: {error}")
            traceback.print_exc()
            return 'break'

        state.current_reroute_item = point2 = state.reroute_locations.popleft()

        try:
            if isinstance(point2[0], (float, int)):
                spline_path = self._calculate_spline_path(
                    point1, point2, skip_easy_dock=True
                )
            else:
                path_coords_with_quaternions = [
                    p + q for p, q in zip(
                        point2,
                        map(
                            lambda x: euler_to_quaternion(0, 0, x)[-2:],
                            3 / 2 * np.pi - np.arctan2(
                                *map(np.gradient, zip(*point2))
                            ),
                        ),
                    )
                ]
                spline_path = spline_planner.generate_spline_path(
                    point1, path_coords_with_quaternions,
                    spacing=cfg.planner.spline_spacing,
                )
        except Exception as error:
            self._logger.error(f"Spline computation failed for reroute: {error}")
            traceback.print_exc()
            return 'skip'

        try:
            with open(rs_path_file, 'wb') as f:
                pickle.dump(spline_path, f)
            state.nmpc_fast_process = subprocess.Popen(
                nmpc_cmd + ["--profile", PROFILE_FAST, "--path_file", rs_path_file],
                preexec_fn=os.setsid,
            )
        except (OSError, pickle.PicklingError, FileNotFoundError) as error:
            self._logger.error(f"Failed to launch nmpc during reroute: {error}")
            traceback.print_exc()
            return 'skip'

        return 'ok'

    def _handle_main_drive_segment(
        self, movement, state, node, cfg, rs_path_file, nmpc_cmd
    ) -> str:
        """Handle one main-drive step. Returns 'break' or 'ok'."""
        try:
            point1 = get_current_pose()
        except RuntimeError as error:
            self._logger.error(f"Cannot get pose for main drive: {error}")
            traceback.print_exc()
            return 'break'

        point2 = node.dock_location
        graph_planner.find_path_and_angles_between_points(
            node.G_loaded, point1, point2,
            path_publisher=node.path_publisher,
            publish_path=movement,
        )

        if (
            node.location_id == cfg.locations.parking_location_id
            and self.is_near_location(
                node.parking_location,
                dist_threshold=cfg.thresholds.parking_near_radius,
            )
        ):
            self._launch_parking_approach(state, node, cfg, rs_path_file, nmpc_cmd)
        else:
            self._launch_fast_drive_to_dock(
                movement, state, node, cfg, rs_path_file, nmpc_cmd, point1, point2
            )

        state.current_reroute_item = None
        return 'ok'

    def _launch_parking_approach(self, state, node, cfg, rs_path_file, nmpc_cmd) -> None:
        try:
            node.publisher_.publish(String(data=FIELD_PICKDROP))
            rs_path = rs_path_planner.generate_rs_path(
                get_current_pose(),
                [node.parking_dock_location],
                turn_radius=cfg.planner.turn_radius,
                rev_drive=False,
            )
            with open(rs_path_file, 'wb') as f:
                pickle.dump(rs_path, f)
            state.nmpc_fast_process = subprocess.Popen(
                nmpc_cmd + ["--profile", PROFILE_SLOW, "--path_file", rs_path_file],
                preexec_fn=os.setsid,
            )
        except (OSError, pickle.PicklingError, FileNotFoundError) as error:
            self._logger.error(f"Parking approach launch failed: {error}")
            traceback.print_exc()
        except RuntimeError as error:
            self._logger.error(f"Cannot get pose for parking approach: {error}")
            traceback.print_exc()
        state.moving_to_destination = False

    def _launch_fast_drive_to_dock(
        self, movement, state, node, cfg, rs_path_file, nmpc_cmd, point1, point2
    ) -> None:
        try:
            node.publisher_.publish(String(data=FIELD_NORMAL))
            spline_path = self._calculate_spline_path(
                point1, point2, publish_path=movement
            )
            with open(rs_path_file, 'wb') as f:
                pickle.dump(spline_path, f)
            state.nmpc_fast_process = subprocess.Popen(
                nmpc_cmd + ["--profile", PROFILE_FAST, "--path_file", rs_path_file],
                preexec_fn=os.setsid,
            )
        except (OSError, pickle.PicklingError, FileNotFoundError) as error:
            self._logger.error(f"Fast drive launch failed: {error}")
            traceback.print_exc()
        except Exception as error:
            self._logger.error(f"Spline/nmpc failed for main drive: {error}")
            traceback.print_exc()
        state.moving_to_destination = True
