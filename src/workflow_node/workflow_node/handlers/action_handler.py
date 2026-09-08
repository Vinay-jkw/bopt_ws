import pickle
import re
import subprocess
import time
import traceback
from threading import Thread

import numpy as np

from workflow_node.constants import (
    ACTION_DROP,
    ACTION_HOLD,
    ACTION_PARKING,
    ACTION_PICKUP,
    ACTION_WAIT,
    PROFILE_DD,
    PROFILE_DP,
    PROFILE_PP,
    PROFILE_SLOW,
)
from workflow_node.handlers.rs_maneuvers import RsManeuverMixin
from workflow_node.states.current_location import get_current_pose
from workflow_node.states.odot_state import get_current_odot_state
from workflow_node.states.pap_status import get_pap_status
from workflow_node.utils.process_utils import run_command_with_retry


class ActionHandler(RsManeuverMixin):
    """Handles pickup, drop, parking, wait, and hold action operations.

    RS path precision maneuvers (pp_control, parking_control, _build_rs_waypoints)
    are inherited from RsManeuverMixin.
    """

    def __init__(self, node) -> None:
        self.node = node
        self._logger = node.get_logger()

    # ------------------------------------------------------------------
    # Fork control
    # ------------------------------------------------------------------

    def fork_up(self) -> None:
        command = (
            "ros2 service call /byd/send_command "
            "example_interfaces/srv/Command \"{command: 'up'}\""
        )
        try:
            subprocess.call(command, shell=True)
        except Exception as error:
            self._logger.error(f"fork_up subprocess failed: {error}")
            traceback.print_exc()

    def fork_down(self) -> None:
        command = (
            "ros2 service call /byd/send_command "
            "example_interfaces/srv/Command \"{command: 'down'}\""
        )
        try:
            subprocess.call(command, shell=True)
        except Exception as error:
            self._logger.error(f"fork_down subprocess failed: {error}")
            traceback.print_exc()

    # ------------------------------------------------------------------
    # Pallet detection
    # ------------------------------------------------------------------

    def apds(self) -> tuple:
        """Run lidar_clustering once and parse the pallet detection result."""
        try:
            result = subprocess.run(
                list(self.node.cfg.subprocesses.ppts),
                capture_output=True,
                text=True,
            )
            # self._logger.info(f"lidar_clustering result: {result}")
        except FileNotFoundError as error:
            self._logger.error(f"lidar_clustering executable not found: {error}")
            traceback.print_exc()
            return None, None, None, None
        except Exception as error:
            self._logger.error(f"lidar_clustering subprocess failed: {error}")
            traceback.print_exc()
            return None, None, None, None
        # Angle Offset can be "X.XX degrees" or "No Data" — both are valid outputs
        stdout = result.stdout or ""
        stderr = result.stderr or ""
        self._logger.info(f"lidar_clustering result: {stdout.strip()}")
        self._logger.info(f"lidar_clustering error: {stderr.strip()}")

        if not stderr:
            # twin_lidar_node reports through stderr — nothing there means the
            # node produced no output at all, not a parse-able "No Data" result.
            self._logger.error(
                "APDS: lidar_clustering produced no stderr output — cannot "
                "determine pallet state."
            )
            return None, None, None, None

        # twin_lidar_node emits its result through the ROS logger, which writes to
        # stderr (stdout is empty), so we parse result.stderr. Two valid shapes:
        #   pallet present: "Pallet Present: Yes, Middle Offset: dx=-0.18, dy=0.12, Angle Offset: 19.07 degrees"
        #   no pallet:      "Pallet Present: No,  Middle Offset: No Data,         Angle Offset: No Data"
        # When no pallet is seen BOTH Middle Offset and Angle Offset collapse to
        # "No Data", so both must be optional in the pattern.
        pattern = (
            r'Pallet Present:\s*(?P<present>\w+),\s*'
            r'Middle Offset:\s*(?:dx=(?P<dx>[-\d.]+),\s*dy=(?P<dy>[-\d.]+)|No Data),\s*'
            r'Angle Offset:\s*(?:(?P<angle>[-\d.]+)\s*degrees|No Data)'
        )
        try:
            match = re.search(pattern, stderr)
        except Exception as error:
            self._logger.error(f"APDS: regex search on lidar_clustering output failed: {error}")
            traceback.print_exc()
            return None, None, None, None

        if not match:
            # The node ran but its output matched neither expected shape — a real
            # format/parse problem, not simply an empty dock. Log the raw output
            # so the mismatch can be diagnosed.
            self._logger.error(
                "APDS: lidar_clustering output did not match the expected "
                "'Pallet Present: ..., Middle Offset: ..., Angle Offset: ...' "
                f"format — cannot determine pallet state. Raw output: {stderr!r}"
            )
            return None, None, None, None

        present = match.group('present')

        # No pallet detected: 'present' is not "Yes" and the offsets came back as
        # "No Data" (so the dx/dy capture groups are None). This is a normal
        # negative result, not an error.
        if present.lower() != 'yes' or match.group('dx') is None:
            self._logger.info(
                f"APDS: no pallet detected (Pallet Present='{present}', "
                "offsets reported as 'No Data')."
            )
            return None, None, None, None

        # Pallet present: dx/dy are guaranteed numeric; angle may still be "No Data".
        try:
            dx = float(match.group('dx'))
            dy = float(match.group('dy'))
            angle_str = match.group('angle')
            angle_rad = np.radians(float(angle_str)) if angle_str is not None else 0.0
        except (ValueError, AttributeError) as error:
            self._logger.error(
                "APDS: pallet detected but its offset values were unparseable "
                f"(dx={match.group('dx')!r}, dy={match.group('dy')!r}, "
                f"angle={match.group('angle')!r}): {error}"
            )
            traceback.print_exc()
            return None, None, None, None

        self._logger.info(
            f"APDS: pallet detected — dx={dx:.3f}m, dy={dy:.3f}m, "
            f"angle={np.degrees(angle_rad):.2f}deg ({angle_rad:.4f}rad)."
        )
        return present, dx, dy, angle_rad

    def ppts(self) -> tuple:
        """Run PPTS/lidar clustering once and parse the pallet detection result."""

        try:
            result = subprocess.run(
                list(self.node.cfg.subprocesses.ppts),
                capture_output=True,
                text=True,
                timeout=30,
            )

        except FileNotFoundError as error:
            self._logger.error(
                f"ppts executable not found: {error}"
            )
            traceback.print_exc()
            return None, None, None, None

        except subprocess.TimeoutExpired as error:
            self._logger.error(
                f"ppts subprocess timed out: {error}"
            )
            return None, None, None, None

        except Exception as error:
            self._logger.error(
                f"ppts subprocess failed: {error}"
            )
            traceback.print_exc()
            return None, None, None, None

        stdout = result.stdout or ""
        stderr = result.stderr or ""

        self._logger.info(
            f"ppts stdout:\n{stdout.strip()}"
        )

        self._logger.info(
            f"ppts stderr:\n{stderr.strip()}"
        )

        # ROS logging normally appears in stderr.
        output = f"{stdout}\n{stderr}"

        # ==========================================================
        # Parse current PPTS detection result
        # ==========================================================

        pattern = (
            r'Pallet Detection\s*\|\s*'
            r'detected=(?P<detected>\w+)\s*\|\s*'
            r'score=(?P<score>[-\d.]+)\s*\|\s*'
            r'detected_poles=(?P<detected_poles>\d+)\s*\|\s*'
            r'expected_poles=(?P<expected_poles>\d+)\s*\|\s*'
            r'row_a_count=(?P<row_a_count>\d+)\s*\|\s*'
            r'row_b_count=(?P<row_b_count>\d+)\s*\|\s*'
            r'x_deviation=(?P<x_deviation>[-\d.]+)\s*\|\s*'
            r'y_deviation=(?P<y_deviation>[-\d.]+)\s*\|\s*'
            r'orientation=(?P<orientation>[-\d.]+)'
        )

        match = re.search(pattern, output)
        reason_pattern = (
            r'Pallet Detection Result\s*\|\s*'
            r'reason=(?P<reason>.*)'
        )

        reason_match = re.search(reason_pattern, output)

        reason = None

        if reason_match:
            reason = reason_match.group("reason").strip()
        if not match:
            self._logger.error(
                "PPTS: PPTS output did not contain a valid "
                "'Pallet Detection' result."
            )

            self._logger.error(
                f"Raw output:\n{output}"
            )

            return None, None, None, None

        detected = match.group("detected").lower() == "true"

        score = float(match.group("score"))
        detected_poles = int(match.group("detected_poles"))
        expected_poles = int(match.group("expected_poles"))

        x_deviation = float(match.group("x_deviation"))
        y_deviation = float(match.group("y_deviation"))
        orientation = float(match.group("orientation"))

        # ==========================================================
        # No pallet
        # ==========================================================

        if not detected:
            self._logger.info(
                f"PPTS: no pallet detected — "
                f"poles={detected_poles}/{expected_poles}, "
                f"score={score:.3f}, "
                f"reason={reason}"
            )

            return None, None, None, None

        # ==========================================================
        # Pallet detected
        # ==========================================================

        self._logger.info(
            f"PPTS: pallet detected — "
            f"dx={x_deviation:.3f}m, "
            f"dy={y_deviation:.3f}m, "
            f"angle={orientation:.3f}rad"
        )

        return (
            "Yes",
            x_deviation,
            y_deviation,
            orientation,
        )
    # ------------------------------------------------------------------
    # Main action dispatcher
    # ------------------------------------------------------------------

    def action_operation(self, action: str) -> bool:
        node = self.node
        state = node.state
        cfg = node.cfg
        mvmt = node.movement_handler

        try:
            node.thread1 = Thread(target=node.pickdrop_field)
            node.thread1.start()
        except Exception as error:
            self._logger.error(f"Failed to start pickdrop_field thread: {error}")
            traceback.print_exc()

        try:
            if action == ACTION_PICKUP:
                self._execute_pickup(node, state, cfg, mvmt)

            elif action == ACTION_DROP:
                self._execute_drop(node, state, cfg, mvmt)

            elif action == ACTION_PARKING:
                self._logger.info("action_operation: PARKING")
                mvmt.drive_rs_path(
                    node.dock_station_end_line,
                    PROFILE_PP,
                    adjust=False,
                    MAX_SPEED=cfg.thresholds.parking_max_speed,
                    parking=True,
                )

            elif ACTION_WAIT in action:
                self._execute_wait(action, node, state, cfg, mvmt)

            elif ACTION_HOLD in action:
                self._execute_hold(node, state, mvmt)

        except Exception as error:
            self._logger.error(
                f"Unexpected failure in action_operation (action='{action}'): {error}"
            )
            traceback.print_exc()
            return False

        if action != ACTION_PARKING:
            try:
                node.thread1 = Thread(target=node.normal_field)
                node.thread1.start()
            except Exception as error:
                self._logger.error(f"Failed to start normal_field thread: {error}")
                traceback.print_exc()

        return True

    # ------------------------------------------------------------------
    # Action-specific implementations
    # ------------------------------------------------------------------

    def _execute_pickup(self, node, state, cfg, mvmt) -> bool:
        self._logger.info("action_operation: PICKUP")

        if state.operation_state < '3':
            mvmt.drive_rs_path(node.dock_station_end_line, PROFILE_DP)
            pose_cmd = list(cfg.subprocesses.pose_correction) + [
                "--ros-args",
                "-p", f"target_z:={node.dock_location[2]}",
                "-p", f"target_w:={node.dock_location[3]}",
            ]
            try:
                subprocess.run(pose_cmd)
            except Exception as error:
                self._logger.error(f"pose_correction subprocess failed: {error}")
                traceback.print_exc()

            present, dx, dy, dang = self.ppts()

            if dx is None:
                node.mqtt_node.publish2topic("machine/error/detected", "E002")
                node.mqtt_node.publish2topic("machine/task/status", "Pallet_Not_Present")
                state.error_status = "Pallet_Not_Present"
                mvmt.drive_rs_path(node.dock_location, PROFILE_SLOW, adjust=False)
                try:
                    node.thread1 = Thread(target=node.pickdrop_field)
                    node.thread1.start()
                except Exception as error:
                    self._logger.error(f"Failed to restart pickdrop_field thread: {error}")
                    traceback.print_exc()
                return True

            if abs(dx) >= cfg.thresholds.apds_dx_threshold:
                self._logger.info(f"In PP {dx}")
                self.pp_control(dx, dy, dang, node.dock_station_end_line[2:])
                # self.pp_control(dx)
            else:
                mvmt.drive_rs_path(node.dock_station_end_line, PROFILE_PP, adjust=False)

            try:
                odot_state = get_current_odot_state()
                self._logger.info(f"odot_state: {odot_state}")
            except RuntimeError as error:
                self._logger.error(f"Could not read odot state: {error}")
                traceback.print_exc()
                return False

            # if '0' != odot_state[0] or '0' != odot_state[-1]:
            if '1' != odot_state[1]:
                node.mqtt_node.publish2topic("machine/error/detected", "E012")
                node.mqtt_node.publish2topic("machine/task/status", "Unable_To_Pickup")
                state.error_status = "Unable_To_Pickup"
                mvmt.drive_rs_path(node.dock_location, PROFILE_SLOW, adjust=False)
                return False

            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=3')

        if state.operation_state < '4':
            self.fork_up()
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=4')

        if state.operation_state < '5':
            mvmt.drive_rs_path(node.dock_location, PROFILE_SLOW, adjust=False)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=5')

        return True

    def _execute_drop(self, node, state, cfg, mvmt) -> bool:
        try:
            pap = get_pap_status()
        except RuntimeError as error:
            self._logger.error(f"Could not read PAP status for drop: {error}")
            traceback.print_exc()
            pap = False
        self._logger.info(f"action_operation: DROP, pap={pap}")

        if state.operation_state < '3':
            if pap:
                node.mqtt_node.publish2topic("machine/error/detected", "E001")
                node.mqtt_node.publish2topic("machine/task/status", "Pallet_Already_Present")
                state.error_status = "Pallet_Already_Present"
                try:
                    node.thread1 = Thread(target=node.pickdrop_field)
                    node.thread1.start()
                except Exception as error:
                    self._logger.error(f"Failed to restart pickdrop_field thread: {error}")
                    traceback.print_exc()
                return True
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=3')

        if state.operation_state < '4':
            self._logger.info(f"drop: dock_station_end_line={node.dock_station_end_line}")
            mvmt.drive_rs_path(node.dock_station_end_line, PROFILE_DD)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=4')

        if state.operation_state < '5':
            self.fork_down()
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=5')

        rs_path_file = cfg.paths.rs_path_file
        try:
            from workflow_node.planners import rs_path_planner
            rs_path = rs_path_planner.generate_rs_path(
                get_current_pose(),
                [node.dock_location],
                turn_radius=cfg.planner.turn_radius,
                rev_drive=False,
            )
            with open(rs_path_file, 'wb') as f:
                pickle.dump(rs_path, f)
        except (OSError, pickle.PicklingError) as error:
            self._logger.error(f"Failed to write RS path for drop return: {error}")
            traceback.print_exc()
        except Exception as error:
            self._logger.error(f"RS path generation failed for drop return: {error}")
            traceback.print_exc()
        else:
            nmpc_cmd = list(cfg.subprocesses.nmpc_controller)
            try:
                subprocess.run(
                    nmpc_cmd + ["--profile", PROFILE_SLOW, "--path_file", rs_path_file],
                    capture_output=True, text=True,
                )
            except Exception as error:
                self._logger.error(f"nmpc_controller failed on drop return: {error}")
                traceback.print_exc()

        pose_cmd = list(cfg.subprocesses.pose_correction) + [
            "--ros-args",
            "-p", f"target_z:={node.dock_location[2]}",
            "-p", f"target_w:={node.dock_location[3]}",
        ]
        if state.operation_state < '6':
            try:
                run_command_with_retry(
                    pose_cmd,
                    timeout=cfg.pose_correction.timeout,
                    max_retries=cfg.pose_correction.max_retries,
                    delay_between_retries=cfg.pose_correction.delay_between_retries,
                )
            except Exception as error:
                self._logger.error(f"pose_correction retry failed: {error}")
                traceback.print_exc()
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=6')

        return True

    def _execute_wait(self, action, node, state, cfg, mvmt) -> bool:
        try:
            pap = get_pap_status()
        except RuntimeError as error:
            self._logger.error(f"Could not read PAP status for wait: {error}")
            traceback.print_exc()
            pap = False
        self._logger.info(f"action_operation: WAIT, pap={pap}")

        if state.operation_state < '3':
            if pap:
                node.mqtt_node.publish2topic("machine/task/status", "Pallet_Already_Present")
                state.error_status = "Pallet_Already_Present"
                node.thread1.start()
                return True
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=3')

        if state.operation_state < '4':
            mvmt.drive_rs_path(node.dock_station_end_line, PROFILE_DD)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=4')

        if state.operation_state < '5':
            try:
                duration = int(action.split('_')[1])
            except (IndexError, ValueError) as error:
                self._logger.warning(
                    f"Could not parse wait duration from '{action}': {error} — defaulting to 0s."
                )
                duration = 0
            self._logger.info(f"Waiting {duration}s...")
            time.sleep(duration)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=5')

        if state.operation_state < '6':
            mvmt.drive_rs_path(node.dock_location, PROFILE_SLOW)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=6')

        return True

    def _execute_hold(self, node, state, mvmt) -> bool:
        self._logger.info("action_operation: HOLD")

        if state.operation_state < '3':
            published = False
            while True:
                try:
                    pap = get_pap_status()
                except RuntimeError as error:
                    self._logger.error(f"Could not read PAP status in hold loop: {error}")
                    traceback.print_exc()
                    break
                if not pap:
                    break
                if not published:
                    node.mqtt_node.publish2topic("machine/error/detected", "E010")
                    published = True
                time.sleep(5)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=3')

        if state.operation_state < '4':
            mvmt.drive_rs_path(node.dock_station_end_line, PROFILE_DD)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=4')

        if state.operation_state < '5':
            self.fork_down()
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=5')

        if state.operation_state < '6':
            mvmt.drive_rs_path(node.dock_location, PROFILE_SLOW)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=6')

        return True
