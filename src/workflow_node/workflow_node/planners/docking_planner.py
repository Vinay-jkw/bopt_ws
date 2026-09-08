import math
import traceback

from rclpy.logging import get_logger

from workflow_node.utils.math_utils import euler_to_quaternion, quaternion_to_euler

_logger = get_logger('docking_planner')


def find_left_or_right_dock(
    left_dock_pose: list,
    right_dock_pose: list,
    dock_pose: list,
    pose_to_compare: list,
    is_last_pose: bool,
) -> list:
    """Choose the correct easy-dock entry pose (left or right side).

    The choice is made by computing the approach heading from dock_pose to
    pose_to_compare and comparing it against each dock's stored orientation.

    Returns either left_dock_pose or right_dock_pose.
    Raises ValueError if the dock side cannot be determined.
    """

    def orientation_difference_deg(q1, q2):
        angle1 = math.degrees(quaternion_to_euler(q1[1], 0, 0, q1[0])[2])
        angle2 = math.degrees(quaternion_to_euler(q2[1], 0, 0, q2[0])[2])
        _logger.debug(f"orientation_difference_deg: {angle1:.2f}° vs {angle2:.2f}°")
        diff = abs(angle1 - angle2) % 360
        return min(diff, 360 - diff)

    try:
        start = dock_pose
        end = pose_to_compare
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        angle_rad = math.atan2(dy, dx)
        if not is_last_pose:
            angle_rad += math.pi
        quaternion = euler_to_quaternion(0, 0, angle_rad)

        euler_angle_deg = math.degrees(angle_rad) % 360

        last_orientation = quaternion[2:]
        left_orientation = left_dock_pose[2:]
        right_orientation = right_dock_pose[2:]

        try:
            left_dock_euler_deg = math.degrees(
                quaternion_to_euler(left_orientation[1], 0, 0, left_orientation[0])[2]
            )
            right_dock_euler_deg = math.degrees(
                quaternion_to_euler(right_orientation[1], 0, 0, right_orientation[0])[2]
            )
            last_orientation_deg = math.degrees(
                quaternion_to_euler(last_orientation[1], 0, 0, last_orientation[0])[2]
            )
        except (IndexError, TypeError, ValueError) as error:
            _logger.error(f"Failed to extract orientation angles: {error}")
            traceback.print_exc()
            raise ValueError("Cannot determine dock side — malformed orientation data.") from error

        try:
            left_diff = orientation_difference_deg(last_orientation, left_orientation)
            right_diff = orientation_difference_deg(last_orientation, right_orientation)
        except (IndexError, TypeError, ValueError) as error:
            _logger.error(f"Orientation difference computation failed: {error}")
            traceback.print_exc()
            raise ValueError("Cannot determine dock side — orientation comparison failed.") from error

        _logger.debug(f"Dock vector: dx={dx:.3f}, dy={dy:.3f}")
        _logger.debug(f"Approach angle: {angle_rad:.4f} rad ({euler_angle_deg:.2f}°)")
        _logger.debug(
            f"last_orientation={last_orientation} ({last_orientation_deg:.2f}°), "
            f"left={left_orientation} ({left_dock_euler_deg:.2f}°), "
            f"right={right_orientation} ({right_dock_euler_deg:.2f}°)"
        )
        _logger.debug(f"left_diff={left_diff:.2f}°, right_diff={right_diff:.2f}°")

        if left_diff < right_diff:
            _logger.info("Choosing left dock")
            return left_dock_pose
        else:
            _logger.info("Choosing right dock")
            return right_dock_pose

    except ValueError:
        raise
    except Exception as error:
        _logger.error(f"Unexpected failure in find_left_or_right_dock: {error}")
        traceback.print_exc()
        raise
