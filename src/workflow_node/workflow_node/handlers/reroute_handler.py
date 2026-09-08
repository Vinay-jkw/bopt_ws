import json
import os
import signal
import traceback
from collections import deque

from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

from workflow_node.constants import (
    CONFLICT_HALT,
    CONFLICT_RESUME,
    CONFLICT_REROUTE,
    PARAM_SHOULD_HALT,
    ROBOT_CLIENT_SET_PARAMETERS,
)

_SERVICE_WAIT_TIMEOUT_S = 10.0


class RerouteHandler:
    """Handles incoming /conflict_action messages: reroute, halt, resume.

    Owns no ROS subscriptions.  WorkflowHandler creates the subscription and
    forwards messages here via handle_conflict_action().
    """

    def __init__(self, node) -> None:
        self.node = node
        self._logger = node.get_logger()

    def handle_conflict_action(self, msg) -> None:
        """Process a /conflict_action String message.

        Preserves exact original logic: reroute kills the running nmpc process
        and repopulates the reroute queue; halt/resume set the parameter on the
        robot_client node via service call.
        """
        try:
            conflict_action = json.loads(msg.data)
            state = self.node.state

            if state.nmpc_fast_process is None:
                state.reroutor_startup_conflict = msg
                return

            action_type = conflict_action['type']

            if action_type == CONFLICT_REROUTE:
                self._logger.info(f"Reroute command received: {conflict_action}")
                state.moving_to_destination = False
                state.reroute_locations = deque(conflict_action['data'])
                if state.nmpc_fast_process.poll() is None:
                    os.killpg(
                        os.getpgid(state.nmpc_fast_process.pid), signal.SIGINT
                    )
                state.nmpc_fast_process.kill()

            elif (
                action_type == CONFLICT_HALT
                and state.nmpc_fast_process.poll() is None
            ):
                self._logger.info(f"Halt command received: {conflict_action}")
                self._set_halt_parameter(True)

            elif (
                action_type == CONFLICT_RESUME
                and state.nmpc_fast_process.poll() is None
            ):
                self._logger.info(f"Resume command received: {conflict_action}")
                self._set_halt_parameter(False)

        except Exception:
            self._logger.error("Unhandled exception in handle_conflict_action:")
            traceback.print_exc()

    def _set_halt_parameter(self, value: bool) -> None:
        """Call /robot_client/set_parameters to toggle the should_halt flag."""
        param_cli = self.node.create_client(
            SetParameters, ROBOT_CLIENT_SET_PARAMETERS
        )
        try:
            elapsed = 0.0
            while not param_cli.wait_for_service(timeout_sec=1.0):
                elapsed += 1.0
                self._logger.info('Waiting for parameter service...')
                if elapsed >= _SERVICE_WAIT_TIMEOUT_S:
                    self._logger.error(
                        f"Timed out waiting for {ROBOT_CLIENT_SET_PARAMETERS} "
                        f"after {elapsed:.0f}s — halt/resume parameter not set."
                    )
                    return

            req = SetParameters.Request()
            req.parameters = [
                Parameter(PARAM_SHOULD_HALT, value=value).to_parameter_msg()
            ]
            param_cli.call_async(req)
        except Exception as error:
            self._logger.error(
                f"_set_halt_parameter({value}) failed: {error}"
            )
            traceback.print_exc()
