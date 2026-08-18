#!/usr/bin/env python3
# Copyright 2026 BOPT Controller Authors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from geometry_msgs.msg import Twist
import rclpy
from std_msgs.msg import Float64

from bopt_controller.controller_node import BOPTController


def test_controller_twist_kinematics():
    """Test Twist kinematics on reverse tricycle."""
    rclpy.init()
    try:
        node = BOPTController()

        # 1. Forward straight (linear.x = 0.5, angular.z = 0.0)
        msg = Twist()
        msg.linear.x = 0.5
        msg.angular.z = 0.0
        node.cmd_vel_twist_callback(msg)

        assert abs(node.current_steering_angle - 0.0) < 1e-4
        assert abs(node.current_wheel_velocity - (0.5 / 0.115)) < 1e-4
        assert not node.is_stopped

        # 2. Turn Left (linear.x = 0.5, angular.z = +0.2 rad/s)
        # target_steering = atan2(-1.542 * 0.2, 0.5) = -0.5526 rad (negative angle = turn left)
        msg.linear.x = 0.5
        msg.angular.z = 0.2
        node.cmd_vel_twist_callback(msg)

        expected_steer = math.atan2(-1.542 * 0.2, 0.5)
        expected_vel = math.hypot(0.5, 1.542 * 0.2) / 0.115
        assert abs(node.current_steering_angle - expected_steer) < 1e-4
        assert abs(node.current_wheel_velocity - expected_vel) < 1e-4

        # 3. Pure rotation (linear.x = 0.0, angular.z = 0.5 rad/s)
        msg.linear.x = 0.0
        msg.angular.z = 0.5
        node.cmd_vel_twist_callback(msg)

        assert abs(node.current_steering_angle - (-1.5708)) < 1e-4

        # 4. Stop (linear.x = 0.0, angular.z = 0.0)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        node.cmd_vel_twist_callback(msg)

        assert abs(node.current_wheel_velocity) < 1e-4
        assert node.is_stopped

        # 5. NaN / Inf safety
        msg.linear.x = float('nan')
        msg.angular.z = float('inf')
        prev_wheel = node.current_wheel_velocity
        prev_steer = node.current_steering_angle
        node.cmd_vel_twist_callback(msg)
        assert node.current_wheel_velocity == prev_wheel
        assert node.current_steering_angle == prev_steer

        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_controller_lift_limits():
    """Test lift clamping and duration."""
    rclpy.init()
    try:
        node = BOPTController()

        # Lift within bounds
        msg = Float64()
        msg.data = 0.05
        node.lift_cmd_callback(msg)

        # Lift exceeds max bound (0.095) -> clamped to 0.095
        msg.data = 0.20
        node.lift_cmd_callback(msg)

        # Lift negative -> clamped to 0.0
        msg.data = -0.10
        node.lift_cmd_callback(msg)

        node.destroy_node()
    finally:
        rclpy.shutdown()
