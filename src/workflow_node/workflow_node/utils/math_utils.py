import math

import numba
import numpy as np


@numba.njit
def rotated_rectangle_mask(x, y, cx, cy, w, h, theta_rad):
    """Return boolean mask: True for points inside or on a rotated rectangle.

    w = (left, right)  — lateral half-extents behind / ahead of robot centre
    h = (front, back)  — longitudinal half-extents
    """
    left, right = w
    top, bottom = h

    cos_t = np.cos(-theta_rad)
    sin_t = np.sin(-theta_rad)

    dx = x - cx
    dy = y - cy

    x_r = dx * cos_t - dy * sin_t
    y_r = dx * sin_t + dy * cos_t

    inside = (-left < x_r) & (x_r < right) & (-bottom < y_r) & (y_r < top)

    on_edge = (
        (np.isclose(x_r, -left) & (-bottom <= y_r) & (y_r <= top)) |
        (np.isclose(x_r, right) & (-bottom <= y_r) & (y_r <= top)) |
        (np.isclose(y_r, -bottom) & (-left <= x_r) & (x_r <= right)) |
        (np.isclose(y_r, top) & (-left <= x_r) & (x_r <= right))
    )

    return inside | on_edge


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> list:
    qx = (np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
          - np.cos(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2))
    qy = (np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2)
          + np.sin(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2))
    qz = (np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2)
          - np.sin(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2))
    qw = (np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2)
          + np.sin(roll / 2) * np.sin(pitch / 2) * np.sin(yaw / 2))
    return [qx, qy, qz, qw]


def quaternion_to_euler(w: float, x: float, y: float, z: float) -> tuple:
    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = 2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


def quaternion_to_angle_rad(z: float, w: float) -> float:
    return 2 * math.atan2(z, w)


def quaternion_to_angle_deg(z: float, w: float) -> float:
    return math.degrees(2 * math.atan2(z, w))


def angle_to_quaternion(angle_rad: float) -> tuple:
    """Return (z, w) quaternion components for a 2-D yaw rotation."""
    return math.sin(angle_rad / 2), math.cos(angle_rad / 2)


def euclidean_distance(pose1, pose2) -> float:
    return math.sqrt((pose1[0] - pose2[0]) ** 2 + (pose1[1] - pose2[1]) ** 2)


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(value, maximum))
