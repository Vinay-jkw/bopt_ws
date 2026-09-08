import math
import pickle
import subprocess
import traceback

import numpy as np
from rsplan import planner

from workflow_node.constants import PROFILE_PARK, PROFILE_PP, PROFILE_SLOW
from workflow_node.states.current_location import get_current_pose
from workflow_node.utils.math_utils import quaternion_to_angle_rad
from workflow_node.utils.process_utils import run_command_with_retry



class RsManeuverMixin:
    """Mixin providing RS-path precision maneuvers for ActionHandler.

    Requires the host class to expose:
        self.node    — WorkflowHandler instance
        self._logger — ROS logger
    """

    def _build_rs_waypoints(self, rs_path_waypoints: list, step_size: float) -> list:
        """Trace an RS path through a list of (x, y, yaw, tr, runway) waypoints."""
        waypoints = []
        if not rs_path_waypoints:
            self._logger.warning("_build_rs_waypoints called with empty waypoint list.")
            return waypoints

        current_pose = tuple(rs_path_waypoints[0][:3])
        for end_pose in rs_path_waypoints[1:]:
            try:
                end_x, end_y, end_yaw, turn_radius, runway_length = end_pose
                end_pose_tuple = (end_x, end_y, end_yaw)
                path = planner.path(
                    current_pose, end_pose_tuple, turn_radius, runway_length, step_size
                )
                waypoints.extend([(wp.x, wp.y) for wp in path.waypoints()])
                current_pose = end_pose_tuple
            except Exception as error:
                self._logger.error(
                    f"RS segment failed from {current_pose} to {end_pose}: {error}"
                )
                traceback.print_exc()
        return waypoints

    def parking_control(self, lateral_offset: float) -> None:
        """Back-up then forward-insert RS maneuver for parking stations."""
        self._logger.info("parking_control: starting")
        try:
            SCALE = self.node.cfg.planner.scale
            step_size = self.node.cfg.planner.step_size
            LATERAL_OFFSET = lateral_offset
            BACKUP_DISTANCE = self.node.cfg.planner.backup_distance_slow
            PALLET_LENGTH = self.node.cfg.planner.pallet_length
            rs_path_file = self.node.cfg.paths.rs_path_file
            nmpc_cmd = list(self.node.cfg.subprocesses.nmpc_controller)

            current_pose = get_current_pose()
            rx, ry = current_pose[0], current_pose[1]
            robotO = quaternion_to_angle_rad(current_pose[2], current_pose[3])

            rs_path_waypoints = [
                (rx * SCALE, ry * SCALE, robotO, 0.9 * SCALE, 0 * SCALE),
                (
                    (rx - math.cos(robotO) * BACKUP_DISTANCE) * SCALE,
                    (ry - math.sin(robotO) * BACKUP_DISTANCE) * SCALE,
                    robotO, 0.9 * SCALE, 1 * SCALE,
                ),
            ]
            waypoints = self._build_rs_waypoints(rs_path_waypoints, step_size)

            try:
                with open(rs_path_file, 'wb') as f:
                    pickle.dump(waypoints, f)
            except (OSError, pickle.PicklingError) as error:
                self._logger.error(f"Failed to write RS path file '{rs_path_file}': {error}")
                traceback.print_exc()
                return

            try:
                subprocess.run(
                    nmpc_cmd + ["--profile", PROFILE_SLOW, "--path_file", rs_path_file],
                    capture_output=True, text=True,
                )
            except Exception as error:
                self._logger.error(f"nmpc_controller failed (parking_control phase 1): {error}")
                traceback.print_exc()
                return

            current_pose = get_current_pose()
            rx, ry = current_pose[0], current_pose[1]
            robotO = quaternion_to_angle_rad(current_pose[2], current_pose[3])

            rs_path_waypoints = [
                (rx * SCALE, ry * SCALE, robotO, 0.3 * SCALE, 0 * SCALE),
                (
                    rx + BACKUP_DISTANCE * math.cos(robotO) - LATERAL_OFFSET * math.sin(robotO),
                    ry + BACKUP_DISTANCE * math.sin(robotO) + LATERAL_OFFSET * math.cos(robotO),
                    robotO, 0.3 * SCALE, 0 * SCALE,
                ),
                (
                    rx + BACKUP_DISTANCE * math.cos(robotO)
                    - LATERAL_OFFSET * math.sin(robotO)
                    + PALLET_LENGTH * math.cos(robotO),
                    ry + BACKUP_DISTANCE * math.sin(robotO)
                    + LATERAL_OFFSET * math.cos(robotO)
                    + PALLET_LENGTH * math.sin(robotO),
                    robotO, 0.3 * SCALE, -1 * SCALE,
                ),
            ]
            waypoints = self._build_rs_waypoints(rs_path_waypoints, step_size)

            try:
                with open(rs_path_file, 'wb') as f:
                    pickle.dump(waypoints, f)
            except (OSError, pickle.PicklingError) as error:
                self._logger.error(f"Failed to write RS path file '{rs_path_file}': {error}")
                traceback.print_exc()
                return

            try:
                subprocess.run(
                    nmpc_cmd + ["--profile", PROFILE_PARK, "--path_file", rs_path_file],
                    capture_output=True, text=True,
                )
            except Exception as error:
                self._logger.error(f"nmpc_controller failed (parking_control phase 2): {error}")
                traceback.print_exc()

        except Exception as error:
            self._logger.error(f"Unexpected failure in parking_control: {error}")
            traceback.print_exc()

    # def pp_control(self, lateral_offset: float) -> None:
    #     """Pallet-pickup precise positioning maneuver using RS paths."""
    #     self._logger.info("pp_control: starting")
    #     try:
    #         SCALE = self.node.cfg.planner.scale
    #         step_size = self.node.cfg.planner.step_size
    #         LATERAL_OFFSET = lateral_offset
    #         BACKUP_DISTANCE = self.node.cfg.planner.backup_distance_pp
    #         PALLET_LENGTH = self.node.cfg.planner.pallet_length
    #         rs_path_file = self.node.cfg.paths.rs_path_file
    #         nmpc_cmd = list(self.node.cfg.subprocesses.nmpc_controller)

    #         current_pose = get_current_pose()
    #         rx, ry = current_pose[0], current_pose[1]
    #         robotO = quaternion_to_angle_rad(current_pose[2], current_pose[3])

    #         rs_path_waypoints = [
    #             (rx * SCALE, ry * SCALE, robotO, 0.9 * SCALE, 0 * SCALE),
    #             (
    #                 (rx - math.cos(robotO) * BACKUP_DISTANCE) * SCALE,
    #                 (ry - math.sin(robotO) * BACKUP_DISTANCE) * SCALE,
    #                 robotO, 0.9 * SCALE, 1 * SCALE,
    #             ),
    #         ]
    #         waypoints = self._build_rs_waypoints(rs_path_waypoints, step_size)

    #         try:
    #             with open(rs_path_file, 'wb') as f:
    #                 pickle.dump(waypoints, f)
    #         except (OSError, pickle.PicklingError) as error:
    #             self._logger.error(f"Failed to write RS path file '{rs_path_file}': {error}")
    #             traceback.print_exc()
    #             return

    #         try:
    #             result = subprocess.run(
    #                 nmpc_cmd + ["--profile", PROFILE_SLOW, "--path_file", rs_path_file]
    #             )
    #             self._logger.debug(f"pp_control phase 1 stdout: {result.stdout}")
    #             self._logger.debug(f"pp_control phase 1 stderr: {result.stderr}")
    #         except Exception as error:
    #             self._logger.error(f"nmpc_controller failed (pp_control phase 1): {error}")
    #             traceback.print_exc()
    #             return

    #         current_pose = get_current_pose()
    #         rx, ry = current_pose[0], current_pose[1]
    #         robotO = quaternion_to_angle_rad(current_pose[2], current_pose[3])

    #         rs_path_waypoints = [
    #             (rx * SCALE, ry * SCALE, robotO, 0.3 * SCALE, 0 * SCALE),
    #             (
    #                 rx + BACKUP_DISTANCE * math.cos(robotO) - LATERAL_OFFSET * math.sin(robotO),
    #                 ry + BACKUP_DISTANCE * math.sin(robotO) + LATERAL_OFFSET * math.cos(robotO),
    #                 robotO, 0.3 * SCALE, 0 * SCALE,
    #             ),
    #             (
    #                 rx + BACKUP_DISTANCE * math.cos(robotO)
    #                 - LATERAL_OFFSET * math.sin(robotO)
    #                 + PALLET_LENGTH * math.cos(robotO),
    #                 ry + BACKUP_DISTANCE * math.sin(robotO)
    #                 + LATERAL_OFFSET * math.cos(robotO)
    #                 + PALLET_LENGTH * math.sin(robotO),
    #                 robotO, 0.3 * SCALE, -1 * SCALE,
    #             ),
    #         ]
    #         waypoints = self._build_rs_waypoints(rs_path_waypoints, step_size)

    #         try:
    #             with open(rs_path_file, 'wb') as f:
    #                 pickle.dump(waypoints, f)
    #         except (OSError, pickle.PicklingError) as error:
    #             self._logger.error(f"Failed to write RS path file '{rs_path_file}': {error}")
    #             traceback.print_exc()
    #             return

    #         try:
    #             result3 = subprocess.run(
    #                 nmpc_cmd + ["--profile", PROFILE_PP, "--path_file", rs_path_file]
    #             )
    #             self._logger.debug(f"pp_control phase 2 stdout: {result3.stdout}")
    #             self._logger.debug(f"pp_control phase 2 stderr: {result3.stderr}")
    #         except Exception as error:
    #             self._logger.error(f"nmpc_controller failed (pp_control phase 2): {error}")
    #             traceback.print_exc()

    #     except Exception as error:
    #         self._logger.error(f"Unexpected failure in pp_control: {error}")
    #         traceback.print_exc()
    
    
    def pp_control(self, lateral_offset: float, dy: float, angular_offset: float, stquat) -> None:
        """Pallet-pickup precise positioning maneuver with lateral + angular correction."""
        self._logger.info("pp_control: starting")
        rs_path_file = self.node.cfg.paths.rs_path_file
        step_size = self.node.cfg.planner.step_size
        # dy = dy - self.node.cfg.planner.load_wheel_base_link_to_lidar - self.node.cfg.planner.pallet_half # for PPTS
        LATERAL_OFFSET = lateral_offset + dy * np.tan(angular_offset)
        BACKUP_DISTANCE = self.node.cfg.planner.backup_distance_pp
        PALLET_LENGTH = self.node.cfg.planner.pallet_length

        # --- Phase 1: straight back-up (linear interpolation) ---
        try:
            current_pose = get_current_pose()
        except RuntimeError as error:
            self._logger.error(f"pp_control phase 1: cannot get pose: {error}")
            traceback.print_exc()
            return

        rx, ry = current_pose[0], current_pose[1]
        robotO = quaternion_to_angle_rad(current_pose[2], current_pose[3])

        start = (rx, ry)
        end = (rx - math.cos(robotO) * BACKUP_DISTANCE,
               ry - math.sin(robotO) * BACKUP_DISTANCE)
        waypoints = [
            (start[0] + t / 100 * (end[0] - start[0]),
             start[1] + t / 100 * (end[1] - start[1]))
            for t in range(101)
        ]

        self._logger.info(f"pp_control: generated waypoints for phase 1: {len(waypoints)}")
        
        try:
            with open(rs_path_file, 'wb') as f:
                pickle.dump(waypoints, f)
        except (OSError, pickle.PicklingError) as error:
            self._logger.error(f"Failed to write RS path (pp phase 1): {error}")
            traceback.print_exc()
            return

        nmpc_cmd = list(self.node.cfg.subprocesses.nmpc_controller)
        try:
            self._logger.info(f"pp_control: executing nmpc_pp_slow with path file: {rs_path_file}")
            result = subprocess.run(
                nmpc_cmd + ["--profile", PROFILE_SLOW, "--path_file", rs_path_file],
                capture_output=True, text=True,
            )
            
            # result = subprocess.run(
            #     list(self.node.cfg.subprocesses.nmpc_pp_slow) + ["--path_file", rs_path_file],
            #                         nmpc_cmd + ["--profile", PROFILE_SLOW, "--path_file", rs_path_file],

            #     capture_output=True, text=True,
            # )
            
            print(result.stdout)
            print(result.stderr)
            
        except Exception as error:
            self._logger.error(f"nmpc_slow failed: {error}")
            traceback.print_exc()
            return
            


        # --- Phase 2: lateral-corrected forward insertion with angle ---
        try:
            current_pose = get_current_pose()
        except RuntimeError as error:
            self._logger.error(f"pp_control phase 2: cannot get pose: {error}")
            traceback.print_exc()
            return

        rx, ry = current_pose[0], current_pose[1]
        robotO = quaternion_to_angle_rad(current_pose[2], current_pose[3])

        self._logger.info(f"pp_control: dx_shift={LATERAL_OFFSET - lateral_offset:.4f}, dx_new={LATERAL_OFFSET:.4f}")
        self._logger.info(f"pp_control: angle_shift={np.degrees(robotO + angular_offset):.2f}, deg, angle_eol={np.degrees(robotO):.2f}deg")

        rs_path_waypoints = [
            (rx, ry, robotO, 0.3, 0.0),
            (
                rx + BACKUP_DISTANCE * math.cos(robotO) - LATERAL_OFFSET * math.sin(robotO),
                ry + BACKUP_DISTANCE * math.sin(robotO) + LATERAL_OFFSET * math.cos(robotO),
                robotO - angular_offset, 0.3, 0.0,
            ),
            (
                rx + BACKUP_DISTANCE * math.cos(robotO)
                - LATERAL_OFFSET * math.sin(robotO - angular_offset)
                + PALLET_LENGTH * math.cos(robotO - angular_offset),
                ry + BACKUP_DISTANCE * math.sin(robotO)
                + LATERAL_OFFSET * math.cos(robotO - angular_offset)
                + PALLET_LENGTH * math.sin(robotO - angular_offset),
                robotO - angular_offset, 0.3, 1.0,
            ),
        ]
        self._logger.info(f"pp_control: rs_path_waypoints={rs_path_waypoints}")

        waypoints = []
        current_rs_pose = tuple(rs_path_waypoints[0][:3])
        for end_pose in rs_path_waypoints[1:]:
            end_x, end_y, end_yaw, turn_radius, runway_length = end_pose
            end_pose_tuple = (end_x, end_y, end_yaw)
            try:
                path = planner.path(current_rs_pose, end_pose_tuple, turn_radius, runway_length, step_size)
                waypoints.extend([(wp.x, wp.y) for wp in path.waypoints()])
                current_rs_pose = end_pose_tuple
            except Exception as error:
                self._logger.error(f"RS segment failed in pp_control phase 2: {error}")
                traceback.print_exc()
                return

        try:
            with open(rs_path_file, 'wb') as f:
                pickle.dump(waypoints, f)
        except (OSError, pickle.PicklingError) as error:
            self._logger.error(f"Failed to write RS path (pp phase 2): {error}")
            traceback.print_exc()
            return

        nmpc_cmd = list(self.node.cfg.subprocesses.nmpc_controller)
        try:
            self._logger.info(f"pp_control: executing nmpc_pp_slow with path file: {rs_path_file}")
            result = subprocess.run(
                nmpc_cmd + ["--profile", PROFILE_PP, "--path_file", rs_path_file],
                capture_output=True, text=True,
            )           
            print(result.stdout)
            print(result.stderr)
            
        except Exception as error:
            self._logger.error(f"nmpc_pp failed: {error}")
            traceback.print_exc()
            return
        
        # pose_cmd = list(self.node.cfg.subprocesses.pose_correction) + [
        #     "--ros-args",
        #     "-p", f"target_z:={self.node.dock_station_end_line[2]}",
        #     "-p", f"target_w:={self.node.dock_station_end_line[3]}",
        # ]
        # try:
        #     run_command_with_retry(
        #         pose_cmd,
        #         timeout=self.node.cfg.pose_correction.timeout,
        #         max_retries=self.node.cfg.pose_correction.max_retries,
        #         delay_between_retries=self.node.cfg.pose_correction.delay_between_retries,
        #     )
        # except Exception as error:
        #     self._logger.error(f"pose_correction failed after retries: {error}")
        #     traceback.print_exc()
        #     return 
