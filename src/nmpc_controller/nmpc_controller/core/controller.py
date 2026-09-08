# Core NMPC path-following controller. Loads a pre-computed lookup table, subscribes
# to the robot pose and sensor topics, and publishes velocity and steering commands
# at 20 Hz to drive the AGV along a pre-recorded path.

import math
import pickle
import traceback
import numpy as np
from datetime import datetime
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
import os

from nmpc_controller.utils.visualization import Visualizer


class NMPCController(Node):
    def __init__(self, cfg):
        super().__init__("nmpc_controller")
        self.cfg = cfg

        # The LUT maps (x, y) target points in the robot body frame to
        # optimal (steering, velocity) pairs solved offline by the NMPC.
        pkg_path = get_package_share_directory("nmpc_controller")
        lut_path = os.path.join(pkg_path, self.cfg.lookup_table)

        with open(lut_path, "rb") as f:
            self.lookup_table = pickle.load(f)

        self.get_logger().info(f"[{cfg.profile}][__init__] {self._now()} LUT loaded: {lut_path} ({len(self.lookup_table)} entries)")
        self._keys = list(self.lookup_table.keys())
        # KDTree built on the first two dimensions of each key for fast nearest-key lookup
        self._tree = KDTree(np.array([k[:2] for k in self._keys]))

        self.current_pose = None
        self.current_velocity = 0.0
        self.path = []
        self.path_index = 0
        self.goal_reached = False
        self.finished = False

        self.pallet_detected = False
        self.pallet_loaded = False
        self.rem_dist = None  # distance to goal, updated each tick for sensor gating
        self.wheel_velocity = 0.0  # actual measured velocity from /byd/wheel_velocity

        # Charging approach state — only used when cfg.parking is True
        self.charger_connected = False
        self._charging_active = False
        self._charge_start_time = None

        self.vel_pub = self.create_publisher(Float64, "/velocity", 10)
        self.steering_angle_publisher = self.create_publisher(Float64, "/steering_angle", 10)
        self.state_pub = self.create_publisher(String, "/state", 10)
        self.safety_turnoff_pub = self.create_publisher(String, "/safety_turnoff", 10)
        self.viz = Visualizer(self)

        # Pose and CAN odometry are published with BEST_EFFORT so we match that QoS
        # to avoid the subscriber silently receiving nothing due to a QoS mismatch.
        best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.create_subscription(PoseStamped, "/current_pose", self.pose_cb, best_effort)
        self.create_subscription(Float64, "/byd/wheel_velocity", self.wheel_vel_cb, best_effort)
        self.create_subscription(Bool, "/pallet_detected", self.pallet_cb, 1)
        self.create_subscription(Bool, "/ds_field_status", self.ds_field_cb, 1)
        self.create_subscription(String, "/byd/can_odot_data", self.can_cb, best_effort)
        self.create_subscription(String, "/charging_state", self.charging_cb, best_effort)

        self.timer = self.create_timer(0.05, self.follow_path)

        self.load_path()

    # =========================

    def load_path(self):
        with open(self.cfg.path_file, "rb") as f:
            self.path = pickle.load(f)

        # final_point is saved before the runway is added so the goal-tolerance
        # check targets the real end of the path, not the extended runway point.
        self.final_point = self.path[-1]

        if self.cfg.features.use_runway:
            self.path = self.add_runway(self.path)

        self.viz.publish_path(self.path)

    def add_runway(self, path):
        # Appends one extra point beyond the last path point in the same heading
        # direction. This prevents the lookahead from collapsing to the final point
        # too early and causing erratic steering on the final approach.
        if len(path) < 2:
            return path
        p1, p2 = path[-2], path[-1]
        dx, dy = p2[0]-p1[0], p2[1]-p1[1]
        norm = np.hypot(dx, dy)
        dx, dy = dx/norm, dy/norm
        length = self.cfg.features.runway_length
        return path + [(p2[0]+length*dx, p2[1]+length*dy)]

    # =========================

    def pose_cb(self, msg):
        self.current_pose = msg

    def wheel_vel_cb(self, msg):
        self.wheel_velocity = msg.data

    def pallet_cb(self, msg):
        # When pallet_gate is active, ignore detection signals within 0.5 m of the
        # goal so a pallet sitting at the dock does not abort the final approach.
        if self.cfg.features.pallet_gate and self.rem_dist is not None and self.rem_dist <= 0.5:
            return
        self.pallet_detected = msg.data

    def ds_field_cb(self, msg):
        # ds_field sensor is only used on profiles that explicitly request it.
        # The same 0.5 m gate prevents a false stop at the dock.
        if self.cfg.features.sensor != "ds_field":
            return
        if self.rem_dist is None or self.rem_dist > 0.5:
            self.pallet_detected = msg.data

    def can_cb(self, msg):
        # CAN odometry string encodes load status; specific byte patterns indicate
        # that a pallet has been picked up and the robot should stop.
        if self.cfg.features.sensor != "can_odot":
            return

        data = msg.data.split(" ")[1]
        # if data[0] in ["0", "1"] or data[-1] == "0":
        self.get_logger().info(f"data[0] is {data[0]} and data[-1] is {data[-1]}")
        if data[0] in ["1"]:
            self.get_logger().info(f"[{self.cfg.profile}][can_cb] {self._now()} Pallet loaded signal received from CAN odometry")
            self.pallet_loaded = True

    def charging_cb(self, msg):
        if msg.data == "ON":
            self.charger_connected = True
            self.get_logger().info(f"[{self.cfg.profile}][charging_cb] {self._now()} Charger connected signal received")

    # =========================

    def _exit(self, reason):
        # Single exit point for all terminal conditions. Guards with self.finished
        # so it is safe to call multiple times — only the first call acts.
        if self.finished:
            return
        self.finished = True
        self.get_logger().info(f"[{self.cfg.profile}][_exit] {self._now()} {reason} — sending zero velocity and shutting down")
        self.stop()
        self.timer.cancel()
        # Raise SystemExit instead of calling rclpy.shutdown() here. Calling shutdown()
        # from inside a callback deadlocks because shutdown() waits for the executor to
        # finish, but the executor is waiting for this callback to return.
        # SystemExit is a BaseException (not Exception) so the executor does not catch it
        # and it unwinds directly to main.py where the finally block handles cleanup.
        raise SystemExit(0)

    def _charging_approach(self):
        # Called every tick while waiting for the charger to connect after pallet_loaded.
        # Creeps forward at 0.02 m/s until charger reports ON or the 4-second timeout fires.
        elapsed = self.get_clock().now().nanoseconds / 1e9 - self._charge_start_time
        if self.charger_connected:
            self.get_logger().info(f"[{self.cfg.profile}][_charging_approach] {self._now()} Charger connected — stopping and restoring safety")
            self.safety_turnoff_pub.publish(String(data="TurnOn"))
            self._exit("Charging approach complete — charger connected")
        elif elapsed > 4.0:
            self.get_logger().info(f"[{self.cfg.profile}][_charging_approach] {self._now()} Charging timeout (4 s) — stopping and restoring safety")
            self.safety_turnoff_pub.publish(String(data="TurnOn"))
            self._exit("Charging approach complete — timeout")
        else:
            self.get_logger().debug(f"[{self.cfg.profile}][_charging_approach] {self._now()} creeping, elapsed={elapsed:.1f}s")
            self.send_command(0.02, 0.0)

    def follow_path(self):
        # If _exit has already been called, do nothing — timer may fire once more
        # before cancel takes effect.
        if self.finished:
            return

        # Charging approach is active: keep creeping until charger connects or timeout.
        if self._charging_active:
            self._charging_approach()
            return

        if self.cfg.features.stop_on_pallet_loaded and self.pallet_loaded:
            if self.cfg.parking:
                # In parking mode, don't stop immediately — creep forward to dock the charger.
                self._charging_active = True
                self._charge_start_time = self.get_clock().now().nanoseconds / 1e9
                self.safety_turnoff_pub.publish(String(data="TurnOff"))
                self.get_logger().info(f"[{self.cfg.profile}][follow_path] {self._now()} Close to Charger in parking mode — starting charging approach, safety off")
            else:
                self._exit("Pallet loaded signal received")
            return

        if self.goal_reached:
            self._exit("Goal already reached")
            return

        if self.pallet_detected:
            self._exit("Pallet detected in path")
            return

        if not self.current_pose:
            return

        try:
            self._follow_path_impl()
        except Exception as e:
            self.get_logger().error(f"[{self.cfg.profile}][follow_path] {self._now()} error: {e}\n{traceback.format_exc()}")
            self._exit("Aborting due to error in follow_path")

    def _follow_path_impl(self):
        x = self.current_pose.pose.position.x
        y = self.current_pose.pose.position.y
        yaw = self.get_yaw(self.current_pose)

        curvature = self.calculate_curvature(self.cfg.safety.curvature_window)
        target = self.get_target_point(x, y)

        # Transform the target into the robot body frame so the LUT can be queried.
        ttp = self.transform(target, x, y, yaw)

        s, v, _ = self.lookup_table[self.find_key(ttp)]

        velocity = (self.cfg.max_velocity / 0.2) * v
        steering_angle = math.degrees(s)

        # Low-pass filter to prevent step changes in velocity command.
        self.current_velocity += (velocity - self.current_velocity) * 0.1

        # The LUT returns negative velocity for reverse motion (normal AGV direction).
        # All safety math works on magnitudes; the sign is reapplied at the end.
        sign = -1 if velocity < 0 else 1
        abs_cv = abs(self.current_velocity)

        # Physics-based speed limit derived from path curvature.
        if self.cfg.safety.type == "sqrt":
            safe_v = 0.15 * np.sqrt(self.cfg.safety.mu * (9.81 / (curvature + 1e-6)))
        else:
            k = self.cfg.safety.sigmoid_k
            mu = self.cfg.safety.mu
            scale = self.cfg.safety.sigmoid_scale
            safe_v = scale * abs_cv * (1 / (1 + np.exp(-k * np.sqrt(mu * (9.81 / (curvature + 1e-6))))))

        safe_v1 = safe_v   # snapshot: sigmoid/sqrt output only

        if self.cfg.safety.type != "sqrt" and self.cfg.safety.exp_decay_k is not None:
            k = self.cfg.safety.exp_decay_k
            curv = abs(self.calculate_curvature(self.cfg.safety.curvature_window))
            curv_threshold = self.cfg.lookahead.straight_threshold
            if curv < curv_threshold:
                safe_v2 = self.cfg.max_velocity
            else:
                safe_v2 = max(self.cfg.min_velocity, abs(self.cfg.max_velocity * np.exp(-k * (curv - curv_threshold))))
            safe_v = min(safe_v, abs(safe_v2))

        safe_v_curv = safe_v   # snapshot: after exp_decay applied on top of sigmoid

        cte = self.calculate_cte(x, y)
        safe_v_cte = None
        if self.cfg.safety.cte_gain is not None and cte > 0:
            safe_v_cte = max(self.cfg.min_velocity, self.cfg.max_velocity * np.exp(-self.cfg.safety.cte_gain * cte))
            safe_v = min(safe_v, safe_v_cte)

        self.current_velocity = sign * max(self.cfg.min_velocity, min(abs_cv, safe_v))

        dist = math.hypot(self.final_point[0]-x, self.final_point[1]-y)
        self.rem_dist = dist

        # Ramp velocity down as the robot approaches the goal.
        # Uses a ceiling that slides from max_velocity at zone entry down to
        # stopping_velocity at the goal, so entry into the slow-down zone never
        # causes a sudden snap — it only prevents speed from rising above the ceiling.
        if dist <= self.cfg.slow_down_distance:
            factor = dist / self.cfg.slow_down_distance
            base = self.cfg.picking_velocity or self.cfg.stopping_velocity
            ramp_ceil = base + factor * (self.cfg.max_velocity - base)
            # Inside the slow-down zone only: multiply the ceiling by a CTE penalty.
            # Uses approach_cte_gain (separate from global cte_gain) so it never
            # causes slow-fast oscillation during normal travel outside this zone.
            if self.cfg.safety.approach_cte_gain is not None and cte > 0:
                ramp_ceil = ramp_ceil * np.exp(-self.cfg.safety.approach_cte_gain * cte)
            ramp_ceil = max(base, ramp_ceil)
            self.current_velocity = sign * min(abs(self.current_velocity), ramp_ceil)

        if dist <= self.cfg.goal_tolerance:
            self.goal_reached = True
            self._exit(f"Goal reached — dtg={dist:.3f} within tolerance={self.cfg.goal_tolerance:.3f}")
            return

        self.viz.publish_target_marker(target)
        cte_str = f"{safe_v_cte:.3f}" if safe_v_cte is not None else "off"
        self.get_logger().info(
            f"[{self.cfg.profile}] {self._now()} "
            f"| v={self.current_velocity:.3f} wheel={self.wheel_velocity:.3f} steer={steering_angle:.1f}° "
            f"| dtg={dist:.3f} idx={self.path_index} "
            f"| curv={curvature:.4f} sv1={safe_v1:.3f} sv2={safe_v_curv:.3f} "
            f"| cte={cte:.3f} safe_cte={cte_str}"
            f"| final point=({self.final_point[0]:.2f}, {self.final_point[1]:.2f})"
            f"| current pose=({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°)"
        )
        self.send_command(self.current_velocity, steering_angle)

    # =========================
    
    def get_target_point(self, robot_x, robot_y):
        la = self.cfg.lookahead

        if la.type == "fixed":
            lookahead_distance = la.value
        else:
            lookahead_distance = la.base + la.velocity_factor * abs(self.wheel_velocity)
            lookahead_distance = max(la.min, min(lookahead_distance, la.max))
            if self.wheel_velocity < 0:
                lookahead_distance += la.offset
            if self.wheel_velocity >= 0:   # FORWARD (fork side)
                lookahead_distance += la.offset

        for i in range(self.path_index, len(self.path)):
            point = self.path[i]
            distance = math.hypot(point[0] - robot_x, point[1] - robot_y)
            if distance > lookahead_distance:
                self.path_index = i
                break

        curvature = self.calculate_curvature(la.curvature_window)

        if curvature < la.straight_threshold:
            lookahead_distance += la.curvature_boost

        self.get_logger().debug(f"[{self.cfg.profile}][get_target_point] {self._now()} curvature={curvature:.5f} lookahead={lookahead_distance:.2f}")

        for i in range(self.path_index, len(self.path)):
            point = self.path[i]
            distance = math.hypot(point[0] - robot_x, point[1] - robot_y)
            if distance > lookahead_distance:
                self.path_index = i
                return point
        return self.path[-1]

    

    # =========================
    
    def calculate_cte(self, robot_x, robot_y):
        # Cross-track error: minimum distance from the robot to the nearest path point.
        # Searches 1.5 m behind path_index (150 points at 1 cm spacing) — enough to
        # catch any lag between the robot position and the advancing path_index.
        start = max(0, self.path_index - 150)
        end = min(len(self.path), self.path_index + 10)
        min_dist = float("inf")
        for pt in self.path[start:end]:
            d = math.hypot(pt[0] - robot_x, pt[1] - robot_y)
            if d < min_dist:
                min_dist = d
        return min_dist

    def calculate_curvature_finite_diff(self, path):
        if len(path) < 3:
            return 0
        # path = path[int((len(path)*0.66)):] #looking at last third of path

        try:
            path = np.array(path)
            x = path[:, 0]
            y = path[:, 1]

            dx = np.gradient(x)
            dy = np.gradient(y)
            ddx = np.gradient(dx)
            ddy = np.gradient(dy)

            denominator = (dx ** 2 + dy ** 2) ** 1.5 + 1e-6  # adding epsilon to avoid division by zero
            curvature = np.abs(dx * ddy - dy * ddx) / denominator
            return np.max(curvature)
        except Exception as e:
            self.get_logger().error(f"[{self.cfg.profile}][calculate_curvature] {self._now()} error: {e}")
            return 0
    
    def calculate_curvature(self, lookahead):
        """
        lookahead is in meters
        :param lookahead:
        :return:
        """
        segmented_path = self.path[self.path_index:self.path_index + int(lookahead * 100)]

        if len(segmented_path) < 3:
            return 0.01

        # Divide all elements in path by 40
        scaled_segmented_path = np.array(segmented_path)
        scaled_segmented_path.tolist()

        return self.calculate_curvature_finite_diff(scaled_segmented_path)
    
    
    
    

    def transform(self, p, x, y, yaw):
        dx, dy = p[0]-x, p[1]-y
        return [
            dx*np.cos(yaw)+dy*np.sin(yaw),
            -dx*np.sin(yaw)+dy*np.cos(yaw)
        ]

    def find_key(self, pos):
        _, idx = self._tree.query(pos)
        return self._keys[int(idx)]

    # ===== RESTORED ORIGINAL FUNCTIONS =====

    def send_command(self, velocity, steering_angle):
        self.vel_pub.publish(Float64(data=velocity))
        self.steering_angle_publisher.publish(Float64(data=steering_angle))

        state_msg = String()
        state_msg.data = "Custom"
        self.state_pub.publish(state_msg)

    def send_stop_command(self, velocity, steering_angle):
        self.vel_pub.publish(Float64(data=velocity))
        self.steering_angle_publisher.publish(Float64(data=steering_angle))

        state_msg = String()
        state_msg.data = "Auto"
        self.state_pub.publish(state_msg)

    def stop(self):
        self.get_logger().info(f"[{self.cfg.profile}][stop] {self._now()} Sending zero velocity")
        self.send_stop_command(0.0, 0.0)

    @staticmethod
    def _now():
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def get_yaw(self, pose):
        q = pose.pose.orientation
        return np.arctan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z)
        )

    def destroy_node(self):
        # Send zero velocity before the ROS context is torn down so the robot
        # does not coast on the last commanded speed after the node exits.
        # print() is used here because the rosout publisher may already be invalid
        # by the time destroy_node is called (e.g. when an external handler shuts
        # down the context first), so get_logger() would produce "Failed to publish" noise.
        try:
            print("[nmpc_controller] Node shutting down — sending final zero velocity")
            self.vel_pub.publish(Float64(data=0.0))
            self.steering_angle_publisher.publish(Float64(data=0.0))
            state_msg = String()
            state_msg.data = "Auto"
            self.state_pub.publish(state_msg)
            print("[nmpc_controller] Zero velocity sent")
        except Exception:
            pass
        super().destroy_node()

