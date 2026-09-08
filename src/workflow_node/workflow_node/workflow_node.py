"""
workflow_node entry point.

Parses CLI arguments, loads config, instantiates WorkflowHandler, runs the
ROS2 executor, executes the workflow, and performs clean shutdown.

Run with:
    ros2 run workflow_node workflow_node <task_id> <movement> <action> [operation_state]

Optional:
    --config /path/to/config.yaml   (default: package share config)
"""

import json
import os
import signal
import sys
import threading
import time
import traceback

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from std_msgs.msg import Float64, String

from workflow_node.config.config_loader import default_config_path, load_config
from workflow_node.handlers.workflow_handler import WorkflowHandler
from workflow_node.states import runtime_state as rs_module
from workflow_node.utils.process_utils import kill_child_processes

# Module-level logger — available before rclpy.init() since rcutils logging
# is initialized independently of node creation.
_logger = get_logger('workflow_node')


# ---------------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------------

def signal_handler(sig, frame) -> None:
    """SIGINT handler: stop robot motion, clear path, shut down."""
    rs_module.cancel_status = True
    rs_module.stop_event.set()
    _logger.info('SIGINT received — stopping robot and shutting down.')

    try:
        node = rclpy.create_node('stop_robot_node')
    except Exception as error:
        _logger.error(f"Could not create stop_robot_node: {error}")
        traceback.print_exc()
        sys.exit(1)

    try:
        path_publisher = node.create_publisher(String, '/path', 10)
        holded_nodes_publisher = node.create_publisher(String, '/holded_nodes', 10)
        path_publisher.publish(String(data=json.dumps({'path': []})))
        holded_nodes_publisher.publish(
            String(data=json.dumps({"holded_nodes": []}))
        )
    except Exception as error:
        _logger.error(f"Failed to publish stop paths: {error}")
        traceback.print_exc()

    try:
        kill_child_processes(os.getpid())
    except Exception as error:
        _logger.warning(f"kill_child_processes failed: {error}")
        traceback.print_exc()

    _logger.info("=======================CANCELLED===========================")

    try:
        cmd_vel_publisher = node.create_publisher(Twist, '/cmd_vel', 10)
        state_publisher = node.create_publisher(String, '/state', 10)
        velocity_publisher = node.create_publisher(Float64, '/velocity', 10)
        steering_angle_publisher = node.create_publisher(Float64, '/steering_angle', 10)

        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0

        state_msg = String()
        state_msg.data = 'auto'

        zero_msg = Float64()
        zero_msg.data = 0.0

        for _ in range(101):
            cmd_vel_publisher.publish(stop_msg)
            state_publisher.publish(state_msg)
            velocity_publisher.publish(zero_msg)
            steering_angle_publisher.publish(zero_msg)
    except Exception as error:
        _logger.error(f"Failed to publish robot stop commands: {error}")
        traceback.print_exc()

    time.sleep(1)

    try:
        node.destroy_node()
    except Exception:
        pass

    try:
        rclpy.shutdown()
    except Exception:
        pass

    sys.exit(0)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None) -> None:
    try:
        rclpy.init(args=args)
    except Exception as error:
        _logger.error(f"rclpy.init failed: {error}")
        traceback.print_exc()
        sys.exit(1)

    signal.signal(signal.SIGINT, signal_handler)
    _logger.info(f"workflow_node started (PID={os.getpid()})")

    if len(sys.argv) < 4:
        _logger.error(
            "Usage: ros2 run workflow_node workflow_node "
            "<task_id> <movement> <action> [operation_state] [--config PATH]"
        )
        rclpy.shutdown()
        return

    _logger.debug(f"argv: {sys.argv}")

    task_id = sys.argv[1]
    movement = sys.argv[2]
    action = sys.argv[3]
    operation_state = str(sys.argv[4]) if len(sys.argv) > 4 and not sys.argv[4].startswith('--') else '0'
    # --action-only: run ONLY the action sequence (pickup/drop in-station), no
    # drive to the station. Used by the VDA connector, which has already driven
    # the robot to the dock via the master's order nodes.
    action_only = '--action-only' in sys.argv

    try:
        config_path = default_config_path()
    except Exception as error:
        _logger.error(f"Could not resolve default config path: {error}")
        traceback.print_exc()
        rclpy.shutdown()
        sys.exit(1)

    for i, arg in enumerate(sys.argv):
        if arg == '--config' and i + 1 < len(sys.argv):
            config_path = sys.argv[i + 1]
            break

    try:
        cfg = load_config(config_path)
    except Exception as error:
        _logger.error(f"Failed to load config from '{config_path}': {error}")
        traceback.print_exc()
        rclpy.shutdown()
        sys.exit(1)

    try:
        workflow_handler = WorkflowHandler(movement, action, cfg)
    except Exception as error:
        _logger.error(f"WorkflowHandler init failed: {error}")
        traceback.print_exc()
        rclpy.shutdown()
        sys.exit(1)

    workflow_handler.state.task_id = task_id
    workflow_handler.state.operation_state = operation_state

    executor = MultiThreadedExecutor()
    executor.add_node(workflow_handler)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    try:
        if action_only:
            # Skip movement; the connector already drove us to the dock. Run the
            # in-station action sequence (end-line -> detect -> fork -> back to dock).
            _logger.info(f"action-only mode: running action '{action}' at '{movement}'")
            workflow_handler.action_handler.action_operation(action)
            if workflow_handler.state.error_status is None:
                workflow_handler.task_request_pub.publish(
                    String(data=f"Task Completed:{workflow_handler.state.task_id}")
                )
        else:
            workflow_handler.operation()
    except Exception as error:
        _logger.error(f"workflow operation failed: {error}")
        traceback.print_exc()

    # ---------------------------------------------------------------------------
    # Clean shutdown sequence
    # ---------------------------------------------------------------------------
    rs_module.cancel_status = True
    rs_module.stop_event.set()
    workflow_handler.state.sequence_complete = True

    try:
        workflow_handler.path_publisher.publish(
            String(data=json.dumps({'path': []}))
        )
        workflow_handler.holded_nodes_publisher.publish(
            String(data=json.dumps({"holded_nodes": []}))
        )
    except Exception as error:
        _logger.warning(f"Failed to publish final empty path/nodes: {error}")
        traceback.print_exc()

    _logger.info("Cleanup and shutdown...")

    try:
        workflow_handler.mqtt_node.disconnect()
        workflow_handler.mqtt_node.loop_stop()
    except Exception as error:
        _logger.warning(f"MQTT cleanup failed: {error}")
        traceback.print_exc()

    try:
        executor.shutdown()
        thread.join()
    except Exception as error:
        _logger.warning(f"Executor shutdown failed: {error}")
        traceback.print_exc()

    try:
        workflow_handler.destroy_node()
    except Exception as error:
        _logger.warning(f"destroy_node failed: {error}")
        traceback.print_exc()

    try:
        rclpy.shutdown()
    except Exception as error:
        _logger.warning(f"rclpy.shutdown failed: {error}")
        traceback.print_exc()

    _logger.info("Workflow Node completed.")

    # Non-zero exit if the action reported an error, so a caller (the VDA
    # connector running --action-only) can surface it as a FAILED actionState.
    if action_only and workflow_handler.state.error_status is not None:
        _logger.error(f"action error: {workflow_handler.state.error_status}")
        sys.exit(2)


if __name__ == '__main__':
    main()
