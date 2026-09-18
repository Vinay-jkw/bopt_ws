"""Rotate raw IMU data from the sensor frame into the robot body frame.

The WIT driver stamps its messages `imu_link` and publishes the values exactly
as they come off the serial link. The mounting is described by the URDF joint

    <joint name="imu_joint" type="fixed">
      <parent link="base_link"/>
      <child  link="imu_link"/>
      <origin xyz="0 0 0.385" rpy="0 0 0" />
    </joint>

so this node looks that transform up through TF and republishes the data
expressed in `target_frame`. Because the lookup is dynamic, editing the URDF
`rpy` (e.g. back to the commented-out "3.1415 0 3.1415" if the sensor is
mounted upside down) changes the output with no code change.

Only the *rotation* is applied. The translation matters solely for linear
acceleration, via the lever-arm terms alpha x r + omega x (omega x r); alpha
needs a differentiated gyro signal, which is noisy enough that the standard
imu_transformer skips it too. Yaw and yaw rate -- the quantities this node was
added for -- are identical for every point on a rigid body, so the 0.385 m
offset does not affect them at all.
"""

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

import tf2_ros

from wit_ros2_imu.quat_utils import (
    q_conjugate,
    q_from_msg,
    q_multiply,
    q_to_matrix,
)


class ImuTransformer(Node):

    def __init__(self):
        super().__init__('imu_transformer')

        self.declare_parameter('input_topic', 'imu/data')
        self.declare_parameter('output_topic', 'imu/transformed')
        self.declare_parameter('target_frame', 'base_link')
        # The sensor frame is normally taken from the message header; set this
        # only if the driver stamps something the URDF does not know about.
        self.declare_parameter('source_frame', '')
        self.declare_parameter('input_best_effort', False)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.source_frame_override = self.get_parameter('source_frame').value
        input_best_effort = self.get_parameter('input_best_effort').value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # imu_joint is fixed, so the lookup only has to succeed once.
        self._rotation = None
        self._cached_source_frame = None

        qos_in = QoSProfile(depth=10)
        if input_best_effort:
            qos_in.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Imu, output_topic, 10)
        self.sub = self.create_subscription(
            Imu, input_topic, self.imu_callback, qos_in)

        self.get_logger().info(
            f'Transforming {input_topic} -> {output_topic} into '
            f'frame "{self.target_frame}"')

    def _lookup_rotation(self, source_frame):
        """Fetch and cache the fixed source_frame -> target_frame rotation."""
        if self._rotation is not None and self._cached_source_frame == source_frame:
            return self._rotation

        try:
            # Time() (zero) asks for the latest available, which is what a
            # /tf_static entry always is.
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, rclpy.time.Time())
        except tf2_ros.TransformException as exc:
            self.get_logger().warn(
                f'No transform {source_frame} -> {self.target_frame} yet: {exc}',
                throttle_duration_sec=5.0)
            return None

        q = q_from_msg(tf.transform.rotation)
        self._rotation = q
        self._cached_source_frame = source_frame

        t = tf.transform.translation
        self.get_logger().info(
            f'Locked mounting transform {source_frame} -> {self.target_frame}: '
            f'xyz=({t.x:.3f}, {t.y:.3f}, {t.z:.3f}) '
            f'quat=({q[0]:.4f}, {q[1]:.4f}, {q[2]:.4f}, {q[3]:.4f})')
        return self._rotation

    def imu_callback(self, msg):
        source_frame = self.source_frame_override or msg.header.frame_id
        if not source_frame:
            self.get_logger().warn(
                'Incoming IMU message has an empty frame_id and no '
                'source_frame override is set; dropping.',
                throttle_duration_sec=5.0)
            return

        q_target_source = self._lookup_rotation(source_frame)
        if q_target_source is None:
            return

        R = q_to_matrix(q_target_source)

        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.target_frame

        # The orientation reports the sensor frame's pose in the IMU's own world
        # reference. Re-express it as the body frame's pose in that same world:
        #   q_world_target = q_world_source * q_source_target
        q_world_source = q_from_msg(msg.orientation)
        q_world_target = q_multiply(q_world_source, q_conjugate(q_target_source))
        out.orientation.x = float(q_world_target[0])
        out.orientation.y = float(q_world_target[1])
        out.orientation.z = float(q_world_target[2])
        out.orientation.w = float(q_world_target[3])

        # Angular velocity and linear acceleration are free vectors: rotate only.
        w = R @ np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        out.angular_velocity.x = float(w[0])
        out.angular_velocity.y = float(w[1])
        out.angular_velocity.z = float(w[2])

        a = R @ np.array([msg.linear_acceleration.x,
                          msg.linear_acceleration.y,
                          msg.linear_acceleration.z])
        out.linear_acceleration.x = float(a[0])
        out.linear_acceleration.y = float(a[1])
        out.linear_acceleration.z = float(a[2])

        # Covariances are 3x3 tensors: C' = R C R^T.
        out.orientation_covariance = self._rotate_cov(
            msg.orientation_covariance, R)
        out.angular_velocity_covariance = self._rotate_cov(
            msg.angular_velocity_covariance, R)
        out.linear_acceleration_covariance = self._rotate_cov(
            msg.linear_acceleration_covariance, R)

        self.pub.publish(out)

    @staticmethod
    def _rotate_cov(cov, R):
        C = np.array(cov, dtype=float).reshape(3, 3)
        # -1 in the first element is the REP-145 "not available" marker; keep it.
        if C[0, 0] == -1.0:
            return list(cov)
        return (R @ C @ R.T).flatten().tolist()


def main(args=None):
    rclpy.init(args=args)
    node = ImuTransformer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
