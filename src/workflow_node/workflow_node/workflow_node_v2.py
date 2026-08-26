import math

import rclpy
from rclpy.node import Node

import tf2_ros

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String


class WorkflowHandler(Node):

    # =========================================================
    # STATES
    # =========================================================

    IDLE = "IDLE"
    LOCALIZATION_CHECK = "LOCALIZATION_CHECK"
    NAVIGATING = "NAVIGATING"
    GOAL_REACHED = "GOAL_REACHED"
    RECOVERY = "RECOVERY"
    SAFETY_STOP = "SAFETY_STOP"

    def __init__(self):

        super().__init__("workflow_handler")

        # =====================================================
        # WORKFLOW STATE
        # =====================================================

        self.current_state = self.IDLE
        self.previous_state = None

        # =====================================================
        # GOAL
        # =====================================================

        self.goal = None

        # =====================================================
        # CURRENT ROBOT POSE
        # =====================================================

        self.current_pose = None
        self.localization_valid = False

        # =====================================================
        # GOAL TOLERANCES
        # =====================================================

        self.position_tolerance = 0.15       # meters
        self.yaw_tolerance = math.radians(10.0)

        # =====================================================
        # SAFETY
        # =====================================================

        self.safety_triggered = False

        # =====================================================
        # NAVIGATION STATUS
        # =====================================================

        # Temporary until NMPC integration.
        # Later this will be replaced by actual NMPC status.
        self.nmpc_active = False

        # =====================================================
        # TF
        # =====================================================

        self.tf_buffer = tf2_ros.Buffer()

        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.map_frame = "map"
        self.base_frame = "base_footprint"

        # =====================================================
        # WORKFLOW STATE PUBLISHER
        # =====================================================
        
        self.state_pub = self.create_publisher(
            String,
            "/workflow/state",
            10
        )

        # =====================================================
        # GOAL SUBSCRIBER
        # =====================================================

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_callback,
            10
        )
        self.path_pub = self.create_publisher(
            Path,
            "/reference_path",
            10
        )
        self.current_pose_pub = self.create_publisher(
            PoseStamped,
            '/current_pose',
            10
        )

        
        # =====================================================
        # WORKFLOW LOOP
        # =====================================================

        self.workflow_timer = self.create_timer(
            0.1,
            self.workflow_step
        )

        # =====================================================
        # STARTUP
        # =====================================================

        self.publish_state()

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "       BOPT WORKFLOW NODE STARTED"
        )

        self.get_logger().info(
            "       Initial State: IDLE"
        )

        self.get_logger().info(
            "       Localization: map -> base_footprint"
        )

        self.get_logger().info(
            "========================================"
        )

    # =========================================================
    # GOAL CALLBACK
    # =========================================================

    def goal_callback(self, msg):

        # -----------------------------------------------------
        # Only accept goals while idle
        # -----------------------------------------------------

        if self.current_state != self.IDLE:

            self.get_logger().warn(
                f"[GOAL] Ignoring goal. "
                f"Workflow is currently {self.current_state}"
            )

            return

        # -----------------------------------------------------
        # Check goal frame
        # -----------------------------------------------------

        goal_frame = msg.header.frame_id

        if goal_frame != self.map_frame:

            self.get_logger().error(
                f"[GOAL] Unsupported goal frame: "
                f"{goal_frame}. Expected '{self.map_frame}'."
            )

            return

        # -----------------------------------------------------
        # Store goal
        # -----------------------------------------------------

        self.goal = {
            "frame_id": goal_frame,

            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
            "z": msg.pose.position.z,

            "qx": msg.pose.orientation.x,
            "qy": msg.pose.orientation.y,
            "qz": msg.pose.orientation.z,
            "qw": msg.pose.orientation.w
        }

        self.get_logger().info(
            f"[GOAL] New goal received | "
            f"x={self.goal['x']:.3f} | "
            f"y={self.goal['y']:.3f}"
        )

        self.get_logger().info(
            f"[GOAL] Orientation | "
            f"qx={self.goal['qx']:.3f} | "
            f"qy={self.goal['qy']:.3f} | "
            f"qz={self.goal['qz']:.3f} | "
            f"qw={self.goal['qw']:.3f}"
        )

    # =========================================================
    # MAIN WORKFLOW LOOP
    # =========================================================

    def workflow_step(self):

        # -----------------------------------------------------
        # SAFETY HAS HIGHEST PRIORITY
        # -----------------------------------------------------

        if self.safety_triggered:

            if self.current_state != self.SAFETY_STOP:

                self.change_state(
                    self.SAFETY_STOP
                )

            return

        # -----------------------------------------------------
        # STATE MACHINE
        # -----------------------------------------------------

        if self.current_state == self.IDLE:

            self.handle_idle()

        elif self.current_state == self.LOCALIZATION_CHECK:

            self.handle_localization_check()

        elif self.current_state == self.NAVIGATING:

            self.handle_navigation()

        elif self.current_state == self.GOAL_REACHED:

            self.handle_goal_reached()

        elif self.current_state == self.RECOVERY:

            self.handle_recovery()

        elif self.current_state == self.SAFETY_STOP:

            self.handle_safety_stop()

        else:

            self.get_logger().error(
                f"[WORKFLOW] Unknown state: "
                f"{self.current_state}"
            )

            self.change_state(
                self.SAFETY_STOP
            )

    # =========================================================
    # STATE TRANSITION
    # =========================================================

    def change_state(self, new_state):

        if new_state == self.current_state:
            return

        self.previous_state = self.current_state
        self.current_state = new_state

        self.get_logger().info(
            f"[WORKFLOW] "
            f"{self.previous_state} -> "
            f"{self.current_state}"
        )

        self.publish_state()

    # =========================================================
    # PUBLISH STATE
    # =========================================================

    def publish_state(self):

        msg = String()
        msg.data = self.current_state

        self.state_pub.publish(msg)

    # =========================================================
    # IDLE
    # =========================================================

    def handle_idle(self):

        if self.goal is None:
            return

        self.get_logger().info(
            "[IDLE] Goal available"
        )

        self.change_state(
            self.LOCALIZATION_CHECK
        )

    # =========================================================
    # LOCALIZATION CHECK
    # =========================================================

    def handle_localization_check(self):

        if self.goal is None:

            self.get_logger().warn(
                "[LOCALIZATION] Goal disappeared"
            )

            self.change_state(
                self.IDLE
            )

            return

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            # -------------------------------------------------
            # Position
            # -------------------------------------------------

            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z

            # -------------------------------------------------
            # Orientation
            # -------------------------------------------------

            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # -------------------------------------------------
            # Save current pose
            # -------------------------------------------------

            self.current_pose = {
                "x": x,
                "y": y,
                "z": z,

                "qx": qx,
                "qy": qy,
                "qz": qz,
                "qw": qw
            }

            self.localization_valid = True

            self.get_logger().info(
                f"[LOCALIZATION] Valid | "
                f"x={x:.3f} | "
                f"y={y:.3f}"
            )

            # -------------------------------------------------
            # Localization is good
            # -------------------------------------------------

            self.change_state(
                self.NAVIGATING
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            self.localization_valid = False

            self.get_logger().warn(
                "[LOCALIZATION] Waiting for "
                "map -> base_footprint transform..."
            )

    # =========================================================
    # NAVIGATING
    # =========================================================
    def generate_reference_path(self):

        if self.current_pose is None:
            return

        if self.goal is None:
            return

        start_x = self.current_pose["x"]
        start_y = self.current_pose["y"]

        goal_x = self.goal["x"]
        goal_y = self.goal["y"]

        # Distance between robot and goal
        distance = math.hypot(
            goal_x - start_x,
            goal_y - start_y
        )

        # Number of points
        point_spacing = 0.1  # meters
        num_points = max(
            2,
            int(distance / point_spacing) + 1
        )

        path_msg = Path()

        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(num_points):

            ratio = i / (num_points - 1)

            x = start_x + ratio * (goal_x - start_x)
            y = start_y + ratio * (goal_y - start_y)

            pose = PoseStamped()

            pose.header = path_msg.header

            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # For now, orientation is not generated.
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        self.get_logger().info(
            f"[PATH] Generated reference path | "
            f"points={len(path_msg.poses)} | "
            f"distance={distance:.2f} m"
        )

    def handle_navigation(self):

        if self.goal is None:

            self.get_logger().warn(
                "[NAVIGATION] No goal available"
            )

            self.stop_nmpc()

            self.change_state(
                self.IDLE
            )

            return

        # -----------------------------------------------------
        # Update current pose continuously
        # -----------------------------------------------------

        if not self.update_current_pose():

            self.localization_valid = False

            self.get_logger().warn(
                "[NAVIGATION] Localization lost"
            )

            self.stop_nmpc()

            self.change_state(
                self.RECOVERY
            )

            return
        # -----------------------------------------------------
        # Generate reference path
        # -----------------------------------------------------

        self.generate_reference_path()

        # -----------------------------------------------------
        # Check whether goal has been reached
        # -----------------------------------------------------

        if self.goal_reached():

            self.get_logger().info(
                "[NAVIGATION] Goal tolerance reached"
            )

            self.stop_nmpc()

            self.change_state(
                self.GOAL_REACHED
            )

            return

        # -----------------------------------------------------
        # NMPC
        # -----------------------------------------------------

        self.run_nmpc()

    # =========================================================
    # UPDATE CURRENT POSE
    # =========================================================

    def update_current_pose(self):

        try:

            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )

            self.current_pose = {

                "x": transform.transform.translation.x,
                "y": transform.transform.translation.y,
                "z": transform.transform.translation.z,

                "qx": transform.transform.rotation.x,
                "qy": transform.transform.rotation.y,
                "qz": transform.transform.rotation.z,
                "qw": transform.transform.rotation.w
            }

            # -------------------------------------------------
            # Publish current robot pose
            # -------------------------------------------------

            pose_msg = PoseStamped()

            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = self.map_frame

            pose_msg.pose.position.x = self.current_pose["x"]
            pose_msg.pose.position.y = self.current_pose["y"]
            pose_msg.pose.position.z = self.current_pose["z"]

            pose_msg.pose.orientation.x = self.current_pose["qx"]
            pose_msg.pose.orientation.y = self.current_pose["qy"]
            pose_msg.pose.orientation.z = self.current_pose["qz"]
            pose_msg.pose.orientation.w = self.current_pose["qw"]

            self.current_pose_pub.publish(pose_msg)

            self.localization_valid = True

            return True

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):

            return False

    # =========================================================
    # GOAL REACHED CHECK
    # =========================================================

    def goal_reached(self):

        if self.current_pose is None:
            return False

        if self.goal is None:
            return False

        # -----------------------------------------------------
        # Position error
        # -----------------------------------------------------

        dx = (
            self.goal["x"]
            - self.current_pose["x"]
        )

        dy = (
            self.goal["y"]
            - self.current_pose["y"]
        )

        distance = math.hypot(
            dx,
            dy
        )

        # -----------------------------------------------------
        # Orientation error
        # -----------------------------------------------------

        current_yaw = self.quaternion_to_yaw(
            self.current_pose
        )

        goal_yaw = self.quaternion_to_yaw(
            self.goal
        )

        yaw_error = self.normalize_angle(
            goal_yaw - current_yaw
        )

        self.get_logger().debug(
            f"[GOAL CHECK] "
            f"distance={distance:.3f} m | "
            f"yaw_error={math.degrees(yaw_error):.2f} deg"
        )

        # -----------------------------------------------------
        # Check both position and orientation
        # -----------------------------------------------------

        return (
            distance <= self.position_tolerance
            and
            abs(yaw_error) <= self.yaw_tolerance
        )

    # =========================================================
    # QUATERNION → YAW
    # =========================================================

    def quaternion_to_yaw(self, pose):

        qx = pose["qx"]
        qy = pose["qy"]
        qz = pose["qz"]
        qw = pose["qw"]

        siny_cosp = (
            2.0 * (qw * qz + qx * qy)
        )

        cosy_cosp = (
            1.0
            - 2.0 * (qy * qy + qz * qz)
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    # =========================================================
    # NORMALIZE ANGLE
    # =========================================================

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    # =========================================================
    # NMPC INTERFACE
    # =========================================================

    def run_nmpc(self):

        """
        Temporary NMPC interface.

        This is where we will connect your existing
        NMPC package.

        NMPC will eventually receive:

            current_pose
            goal_pose

        and generate:

            velocity
            steering

        or whatever interface your NMPC package uses.
        """

        if not self.nmpc_active:

            self.get_logger().info(
                "[NMPC] Starting navigation controller"
            )

            self.nmpc_active = True

        # -----------------------------------------------------
        # TODO:
        #
        # Send:
        #     self.current_pose
        #     self.goal
        #
        # to actual NMPC controller.
        # -----------------------------------------------------

    # =========================================================
    # STOP NMPC
    # =========================================================

    def stop_nmpc(self):

        if self.nmpc_active:

            self.get_logger().info(
                "[NMPC] Stopping navigation controller"
            )

        self.nmpc_active = False

        # TODO:
        # Send stop command to NMPC/controller.

    # =========================================================
    # GOAL REACHED
    # =========================================================

    def handle_goal_reached(self):

        self.stop_nmpc()

        self.get_logger().info(
            "[WORKFLOW] Goal reached successfully"
        )

        # Clear current task

        self.goal = None
        self.current_pose = None

        self.localization_valid = False

        self.change_state(
            self.IDLE
        )

    # =========================================================
    # RECOVERY
    # =========================================================

    def handle_recovery(self):

        self.stop_nmpc()

        self.get_logger().warn(
            "[RECOVERY] Attempting recovery"
        )

        # For now:
        # go back and check localization.

        if self.goal is not None:

            self.change_state(
                self.LOCALIZATION_CHECK
            )

        else:

            self.change_state(
                self.IDLE
            )

    # =========================================================
    # SAFETY STOP
    # =========================================================

    def handle_safety_stop(self):

        self.stop_nmpc()

        self.get_logger().error(
            "[SAFETY] SAFETY STOP ACTIVE"
        )

    # =========================================================
    # SAFETY API
    # =========================================================

    def trigger_safety_stop(self):

        self.safety_triggered = True

        self.get_logger().error(
            "[SAFETY] Safety stop triggered!"
        )

    def clear_safety_stop(self):

        self.safety_triggered = False

        self.get_logger().info(
            "[SAFETY] Safety stop cleared"
        )

        if self.current_state == self.SAFETY_STOP:

            self.change_state(
                self.LOCALIZATION_CHECK
            )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = WorkflowHandler()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Keyboard interrupt received."
        )

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":

    main()