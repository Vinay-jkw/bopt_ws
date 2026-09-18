"""Small quaternion / angle helpers.

Kept dependency-free (numpy only) so the nodes do not need tf_transformations,
which is not installed on the AMR images.

All quaternions are ROS ordered: (x, y, z, w).
"""

import math

import numpy as np


def q_from_msg(q):
    """geometry_msgs/Quaternion -> np.array([x, y, z, w])."""
    return np.array([q.x, q.y, q.z, q.w], dtype=float)


def q_multiply(a, b):
    """Hamilton product a (x) b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=float)


def q_conjugate(q):
    """Inverse rotation (quaternion is assumed unit norm)."""
    x, y, z, w = q
    return np.array([-x, -y, -z, w], dtype=float)


def q_to_matrix(q):
    """Rotation matrix R such that v_parent = R @ v_child."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=float)


def yaw_from_quaternion(q):
    """Z-axis rotation of a (x, y, z, w) quaternion, in radians."""
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_to_pi(angle):
    """Fold an angle into [-pi, pi) so wrap-around never shows up as a jump."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi
