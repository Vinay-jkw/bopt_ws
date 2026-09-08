import os
import re
import signal
import subprocess
import time
import traceback
from typing import Optional

import psutil
from rclpy.logging import get_logger

_logger = get_logger('process_utils')


def kill_child_processes(parent_pid: int, sig: int = signal.SIGTERM) -> None:
    """Terminate all child processes of parent_pid, then SIGKILL survivors."""
    try:
        parent = psutil.Process(parent_pid)
    except psutil.NoSuchProcess:
        return
    try:
        children = parent.children(recursive=True)
        for child in children:
            try:
                child.terminate()
            except psutil.NoSuchProcess:
                pass
        gone, still_alive = psutil.wait_procs(children, timeout=5, callback=None)
        for p in still_alive:
            try:
                p.kill()
                _logger.warning(f"Force-killed lingering child PID {p.pid}")
            except psutil.NoSuchProcess:
                pass
    except Exception as error:
        _logger.warning(
            f"Error during child process cleanup for PID {parent_pid}: {error}"
        )
        traceback.print_exc()


def kill_process_tree(pid: int, including_parent: bool = True) -> None:
    """Kill an entire process tree rooted at pid."""
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        for child in children:
            try:
                child.kill()
            except psutil.NoSuchProcess:
                pass
        psutil.wait_procs(children)
        if including_parent:
            try:
                parent.kill()
                parent.wait()
            except psutil.NoSuchProcess:
                pass
    except psutil.NoSuchProcess:
        pass
    except Exception as error:
        _logger.warning(f"Error killing process tree for PID {pid}: {error}")
        traceback.print_exc()


def run_command_with_retry(
    command: list,
    timeout: int = 45,
    max_retries: int = 5,
    delay_between_retries: int = 5,
) -> subprocess.CompletedProcess:
    """Run a command with timeout and retries; raise on exhaustion.

    Each attempt that times out kills the process before retrying.
    """
    _logger.info(f"Running command: {' '.join(command)}")
    last_error = None

    for attempt in range(max_retries):
        process = None
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
        except (FileNotFoundError, PermissionError, OSError) as error:
            _logger.error(
                f"Failed to launch command "
                f"(attempt {attempt + 1}/{max_retries}): {error}"
            )
            traceback.print_exc()
            last_error = error
            time.sleep(delay_between_retries)
            continue

        try:
            output, error = process.communicate(timeout=timeout)
            if process.returncode != 0:
                _logger.warning(
                    f"Command exited with code {process.returncode}. "
                    f"stderr: {error.strip()}"
                )
            return subprocess.CompletedProcess(
                process.args, process.returncode, output, error
            )
        except subprocess.TimeoutExpired:
            _logger.warning(
                f"Command timed out after {timeout}s "
                f"(attempt {attempt + 1}/{max_retries}). "
                f"Killing process and retrying in {delay_between_retries}s..."
            )
            last_error = subprocess.TimeoutExpired(command, timeout)
        finally:
            if process is not None and process.poll() is None:
                try:
                    process.kill()
                    process.wait()
                except Exception:
                    pass
            time.sleep(delay_between_retries)

    raise RuntimeError(
        f"Command '{' '.join(command)}' failed after {max_retries} retries. "
        f"Last error: {last_error}"
    )


def get_pds_results() -> tuple:
    """Run pallet_detection once and parse stdout for distance and offset."""
    _logger.info("Running pallet_detection...")
    try:
        result_pds = subprocess.run(
            ["ros2", "run", "pallet_detection", "pallet_detection"],
            capture_output=True,
            text=True,
        )
        if result_pds.returncode != 0:
            _logger.warning(
                f"pallet_detection exited with code {result_pds.returncode}. "
                f"stderr: {result_pds.stderr.strip()}"
            )
    except Exception as error:
        _logger.error(f"Failed to run pallet_detection: {error}")
        traceback.print_exc()
        return None, None, None

    output_string = result_pds.stdout

    distance_to_midpoint_match = re.search(
        r"Distance to midpoint: (\d+\.\d+)", output_string
    )
    distance_to_midpoint = (
        float(distance_to_midpoint_match.group(1))
        if distance_to_midpoint_match
        else None
    )

    tensor_match = re.search(r"tensor\((\d+\.\d+)\)", output_string)
    tensor_value = float(tensor_match.group(1)) if tensor_match else None

    fetched_m_offset_match = re.search(
        r"fetched_m_offset (-?\d+\.\d+)", output_string
    )
    fetched_m_offset = (
        float(fetched_m_offset_match.group(1))
        if fetched_m_offset_match
        else None
    )

    _logger.info(
        f"PDS results: distance={distance_to_midpoint}, "
        f"tensor={tensor_value}, m_offset={fetched_m_offset}"
    )
    return distance_to_midpoint, tensor_value, fetched_m_offset


def get_pds_tag_results() -> tuple:
    """Run pallet_detection_tag once and parse stdout for offsets."""
    _logger.info("Running pallet_detection_tag...")
    try:
        result_pds = subprocess.run(
            ["ros2", "run", "pallet_detection", "pallet_detection_tag"],
            capture_output=True,
            text=True,
        )
        if result_pds.returncode != 0:
            _logger.warning(
                f"pallet_detection_tag exited with code {result_pds.returncode}. "
                f"stderr: {result_pds.stderr.strip()}"
            )
    except Exception as error:
        _logger.error(f"Failed to run pallet_detection_tag: {error}")
        traceback.print_exc()
        return None, None

    output_string = result_pds.stdout

    fetched_m_offset_match = re.search(
        r"fetched_m_offset (-?\d+\.\d+)", output_string
    )
    fetched_m_offset = (
        float(fetched_m_offset_match.group(1))
        if fetched_m_offset_match
        else None
    )

    fetched_angular_offset_match = re.search(
        r"fetched_angular_offset (-?\d+\.\d+)", output_string
    )
    fetched_angular_offset = (
        float(fetched_angular_offset_match.group(1))
        if fetched_angular_offset_match
        else None
    )

    _logger.info(
        f"PDS tag results: m_offset={fetched_m_offset}, "
        f"angular_offset={fetched_angular_offset}"
    )
    return fetched_m_offset, fetched_angular_offset
