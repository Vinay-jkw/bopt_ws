import math
import pickle
import subprocess
import traceback

from std_msgs.msg import String

from workflow_node.constants import PROFILE_SLOW
from workflow_node.handlers.fast_drive import FastDriveMixin
from workflow_node.planners import rs_path_planner
from workflow_node.states.current_location import get_current_pose
from workflow_node.utils.math_utils import euclidean_distance
from workflow_node.utils.process_utils import run_command_with_retry


class MovementHandler(FastDriveMixin):
    """Handles all navigation movement operations: spline traversal, RS drives,
    rerouting loop, and dock approach.

    Fast-drive loop and spline path logic are inherited from FastDriveMixin.
    Receives the WorkflowHandler node as owner for ROS publisher access and
    shared RuntimeState.
    """

    def __init__(self, node) -> None:
        self.node = node
        self._logger = node.get_logger()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def is_movement_needed(self) -> bool:
        try:
            gap_from_dock = euclidean_distance(
                get_current_pose(), self.node.dock_location
            )
            self._logger.info(f"Gap from dock: {gap_from_dock:.3f}m")
            return gap_from_dock > self.node.cfg.thresholds.movement_needed_gap
        except RuntimeError as error:
            self._logger.error(f"Could not get current pose for movement check: {error}")
            traceback.print_exc()
            return True
        except Exception as error:
            self._logger.error(f"is_movement_needed failed: {error}")
            traceback.print_exc()
            return True

    def is_near_location(self, location: list, dist_threshold: float = None) -> bool:
        if dist_threshold is None:
            dist_threshold = self.node.cfg.thresholds.near_location_radius
        try:
            current_pose = get_current_pose()
            dist = math.hypot(
                current_pose[0] - location[0], current_pose[1] - location[1]
            )
            self._logger.debug(f"is_near_location: distance={dist:.3f}m, threshold={dist_threshold}m")
            return dist <= dist_threshold
        except RuntimeError as error:
            self._logger.error(f"Could not get current pose for is_near_location: {error}")
            traceback.print_exc()
            return False
        except Exception as error:
            self._logger.error(f"is_near_location failed: {error}")
            traceback.print_exc()
            return False

    def drive_rs_path(
        self,
        location: list,
        drive_type: str,
        adjust: bool = True,
        MAX_SPEED: float = None,
        parking: bool = False,
    ) -> bool:
        """Generate an RS path to location, write pickle, launch nmpc_controller."""
        try:
            current_pose = get_current_pose()
        except RuntimeError as error:
            self._logger.error(f"drive_rs_path cannot get current pose: {error}")
            traceback.print_exc()
            return False

        rs_path = rs_path_planner.generate_rs_path(
            current_pose,
            [location],
            turn_radius=self.node.cfg.planner.turn_radius,
            rev_drive=False,
        )

        rs_path_file = self.node.cfg.paths.rs_path_file
        try:
            with open(rs_path_file, 'wb') as f:
                pickle.dump(rs_path, f)
        except (OSError, pickle.PicklingError) as error:
            self._logger.error(f"Failed to write RS path file '{rs_path_file}': {error}")
            traceback.print_exc()
            return False

        nmpc_cmd = list(self.node.cfg.subprocesses.nmpc_controller)

        try:
            if MAX_SPEED is not None and parking:
                self._logger.info(f"drive_rs_path: profile={drive_type}, max_speed={MAX_SPEED}, parking=True")
                self.node.result1 = subprocess.run(
                    nmpc_cmd + [
                        "--profile", drive_type,
                        "--path_file", rs_path_file,
                        "--max_velocity", str(MAX_SPEED),
                        "--parking", "True",
                    ]
                )
            else:
                self._logger.info(f"drive_rs_path: profile={drive_type}")
                self.node.result1 = subprocess.run(
                    nmpc_cmd + ["--profile", drive_type, "--path_file", rs_path_file]
                )
            self._logger.debug(f"nmpc stdout: {self.node.result1.stdout}")
            self._logger.debug(f"nmpc stderr: {self.node.result1.stderr}")
        except FileNotFoundError as error:
            self._logger.error(f"nmpc_controller executable not found: {error}")
            traceback.print_exc()
            return False
        except Exception as error:
            self._logger.error(f"nmpc_controller subprocess failed (drive_type={drive_type}): {error}")
            traceback.print_exc()
            return False

        if adjust:
            pose_cmd = list(self.node.cfg.subprocesses.pose_correction) + [
                "--ros-args",
                "-p", f"target_z:={location[2]}",
                "-p", f"target_w:={location[3]}",
            ]
            try:
                run_command_with_retry(
                    pose_cmd,
                    timeout=self.node.cfg.pose_correction.timeout,
                    max_retries=self.node.cfg.pose_correction.max_retries,
                    delay_between_retries=self.node.cfg.pose_correction.delay_between_retries,
                )
            except Exception as error:
                self._logger.error(f"pose_correction failed after retries: {error}")
                traceback.print_exc()

        return True

    def movement_operation(self, movement: str) -> bool:
        """Main movement loop: spline fast-drive → easy dock → RS slow to dock."""
        node = self.node
        state = node.state

        try:
            node.switch_off_contactor_and_reset_charging_state()
        except Exception as error:
            self._logger.error(f"switch_off_contactor failed: {error}")
            traceback.print_exc()

        try:
            node.safety_turnoff_publisher.publish(String(data='TurnOn'))
        except Exception as error:
            self._logger.error(f"safety_turnoff publish failed: {error}")
            traceback.print_exc()

        if state.operation_state < '1':
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=0')
            self._run_fast_drive_loop(movement)
            self._logger.info("Fast drive completed.")
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=1')

        if state.operation_state < '2':
            self.drive_rs_path(node.dock_location, PROFILE_SLOW, adjust=True)
            node.mqtt_node.publish2topic('machine/task/status', 'operation_state=2')

        return True
