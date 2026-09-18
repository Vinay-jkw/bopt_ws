"""Record yaw from the transformed IMU, wheel odometry and AMCL, and log the gaps.

Subscribes:
  imu/transformed          sensor_msgs/Imu        body frame, from imu_transformer
  /byd/byd_cpp_odometry    nav_msgs/Odometry      raw wheel odometry
  /current_pose            geometry_msgs/PoseStamped   map -> load_wheel_base_link,
                           i.e. AMCL's correction composed onto the EKF odom

Samples all three on a fixed timer and writes one CSV row per sample. AMCL is
optional: if /current_pose is silent the run continues and its columns stay
empty, so a recording taken before localisation is up is still usable.

Why the CSV carries *relative* yaw as well as absolute: the three sources do not
share a zero. byd_odometry_publisher_cpp integrates `theta` from 0.0 the moment
the node boots, the WIT IMU reports an absolute heading against its own AHRS
reference, and /current_pose is referenced to the map origin. Their absolute
yaws differ by arbitrary constants that say nothing about accuracy. The
meaningful numbers are how far apart they drift *after* being aligned at t0.

Reading the three error columns:
  imu_odom_error_deg   IMU vs dead reckoning. Grows steadily -> wheel odometry
                       yaw is off (track width / wheel radius, or slip).
  imu_amcl_error_deg   IMU vs the localised pose. This is the one that matters
                       for navigation, since AMCL is what nav2 acts on.
  odom_amcl_error_deg  how hard AMCL is having to correct the dead reckoning.
                       Expect a sawtooth: it creeps up as odometry drifts, then
                       snaps back on each AMCL update. Large jumps mean the
                       particle filter is fighting the odometry.

yaw_rate_error_dps is the offset-free instantaneous check: gyro Z versus
odometry twist.angular.z. Opposite signs mean the imu_joint rpy in the URDF is
wrong. A steady non-zero value means a scale error in the wheel odometry; a
value that is zero while moving straight but spikes in turns points at slip.
"""

import csv
import math
import os
from datetime import datetime

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger

from wit_ros2_imu.quat_utils import q_from_msg, wrap_to_pi, yaw_from_quaternion

CSV_FIELDS = [
    'wall_time',
    'stamp_s',
    'elapsed_s',
    'imu_yaw_deg',
    'odom_yaw_deg',
    'amcl_yaw_deg',
    'imu_yaw_rel_deg',
    'odom_yaw_rel_deg',
    'amcl_yaw_rel_deg',
    'imu_odom_error_deg',
    'imu_amcl_error_deg',
    'odom_amcl_error_deg',
    'imu_yaw_rate_dps',
    'odom_yaw_rate_dps',
    'yaw_rate_error_dps',
    'odom_x',
    'odom_y',
    'amcl_x',
    'amcl_y',
    'odom_lin_vel',
    'imu_age_s',
    'odom_age_s',
    'amcl_age_s',
]


class YawTrack:
    """Accumulates unwrapped yaw for one source, so a test that turns through
    more than 180 deg plots as a continuous ramp instead of sawtoothing at the
    +/-pi seam."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.prev = None
        self.rel = 0.0

    def update(self, yaw):
        if self.prev is not None:
            self.rel += wrap_to_pi(yaw - self.prev)  # per-step delta wrapped,
        self.prev = yaw                              # running total is not
        return self.rel


class ErrorStats:
    """Running mean / max of one error channel."""

    def __init__(self, label):
        self.label = label
        self.reset()

    def reset(self):
        self.total = 0.0
        self.count = 0
        self.max_abs = 0.0
        self.last = 0.0

    def add(self, err_deg):
        self.last = err_deg
        self.total += abs(err_deg)
        self.max_abs = max(self.max_abs, abs(err_deg))
        self.count += 1

    def summary(self):
        if not self.count:
            return f'{self.label}: no data'
        return '{}: now {:+.2f}, mean |{:.2f}|, max |{:.2f}|'.format(
            self.label, self.last, self.total / self.count, self.max_abs)


class YawComparator(Node):

    def __init__(self):
        super().__init__('yaw_comparator')

        default_csv = os.path.join(
            os.path.expanduser('~'),
            'imu_odom_yaw_{}.csv'.format(datetime.now().strftime('%Y%m%d_%H%M%S')))

        self.declare_parameter('imu_topic', 'imu/transformed')
        self.declare_parameter('odom_topic', '/byd/byd_cpp_odometry')
        self.declare_parameter('amcl_topic', '/current_pose')
        self.declare_parameter('output_path', default_csv)
        self.declare_parameter('sample_rate_hz', 20.0)
        self.declare_parameter('summary_period_s', 10.0)
        # byd_odometry_publisher_cpp and current_pose_publisher both publish
        # BEST_EFFORT; a BEST_EFFORT subscriber also accepts a RELIABLE
        # publisher, so it is the safe default for both.
        self.declare_parameter('odom_best_effort', True)
        self.declare_parameter('amcl_best_effort', True)
        self.declare_parameter('imu_best_effort', False)
        # Discard samples where a source has gone stale, so a dead topic does
        # not quietly fill the CSV with a flat line.
        self.declare_parameter('max_age_s', 0.5)
        # AMCL is optional by default: rows are still written before
        # localisation comes up, just with the amcl_* columns left blank.
        self.declare_parameter('require_amcl', False)

        imu_topic = self.get_parameter('imu_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        amcl_topic = self.get_parameter('amcl_topic').value
        self.output_path = self.get_parameter('output_path').value
        rate = float(self.get_parameter('sample_rate_hz').value)
        summary_period = float(self.get_parameter('summary_period_s').value)
        self.max_age = float(self.get_parameter('max_age_s').value)
        self.require_amcl = bool(self.get_parameter('require_amcl').value)

        imu_qos = QoSProfile(depth=10)
        if self.get_parameter('imu_best_effort').value:
            imu_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odom_qos = QoSProfile(depth=10)
        if self.get_parameter('odom_best_effort').value:
            odom_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        amcl_qos = QoSProfile(depth=10)
        if self.get_parameter('amcl_best_effort').value:
            amcl_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.imu_msg = self.odom_msg = self.amcl_msg = None
        self.imu_rx = self.odom_rx = self.amcl_rx = None

        self.imu_track = YawTrack()
        self.odom_track = YawTrack()
        self.amcl_track = YawTrack()
        self.t_zero = None
        # AMCL usually comes up well after the IMU and odometry (its lifecycle
        # node has to activate first), so by the time it joins the other two
        # have already accumulated relative yaw. Comparing against that head
        # start would report it as error. Latch where they were when AMCL's
        # own track started from zero, and difference against that instead.
        self.amcl_baseline = None
        self.rows_written = 0
        self.amcl_rows = 0
        self.amcl_seen = False

        self.imu_odom = ErrorStats('imu-odom')
        self.imu_amcl = ErrorStats('imu-amcl')
        self.odom_amcl = ErrorStats('odom-amcl')

        self.create_subscription(Imu, imu_topic, self._imu_cb, imu_qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, odom_qos)
        self.create_subscription(PoseStamped, amcl_topic, self._amcl_cb, amcl_qos)
        self.create_service(Trigger, '~/reset_alignment', self._reset_cb)

        self.csv_file = open(self.output_path, 'w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
        self.writer.writeheader()
        self.csv_file.flush()

        self.create_timer(1.0 / rate, self._sample)
        if summary_period > 0.0:
            self.create_timer(summary_period, self._log_summary)

        self.get_logger().info(
            'Comparing {} vs {} vs {} at {:g} Hz (amcl {})'.format(
                imu_topic, odom_topic, amcl_topic, rate,
                'required' if self.require_amcl else 'optional'))
        self.get_logger().info(f'Writing {self.output_path}')

    def _imu_cb(self, msg):
        self.imu_msg = msg
        self.imu_rx = self.get_clock().now()

    def _odom_cb(self, msg):
        self.odom_msg = msg
        self.odom_rx = self.get_clock().now()

    def _amcl_cb(self, msg):
        self.amcl_msg = msg
        self.amcl_rx = self.get_clock().now()
        if not self.amcl_seen:
            self.amcl_seen = True
            self.get_logger().info('AMCL pose received; amcl_* columns are live')

    def _reset_cb(self, request, response):
        """Re-zero every source so a new test segment starts from 0 error."""
        for track in (self.imu_track, self.odom_track, self.amcl_track):
            track.reset()
        for stats in (self.imu_odom, self.imu_amcl, self.odom_amcl):
            stats.reset()
        self.t_zero = None
        self.amcl_baseline = None
        response.success = True
        response.message = 'Alignment reset; next sample becomes the new zero.'
        self.get_logger().info(response.message)
        return response

    def _age(self, rx_time, now):
        if rx_time is None:
            return None
        return (now - rx_time).nanoseconds * 1e-9

    def _sample(self):
        if self.imu_msg is None or self.odom_msg is None:
            self.get_logger().warn(
                'Waiting for both required topics (imu={}, odom={})'.format(
                    self.imu_msg is not None, self.odom_msg is not None),
                throttle_duration_sec=5.0)
            return

        now = self.get_clock().now()
        imu_age = self._age(self.imu_rx, now)
        odom_age = self._age(self.odom_rx, now)
        if imu_age > self.max_age or odom_age > self.max_age:
            self.get_logger().warn(
                f'Stale data (imu {imu_age:.2f}s, odom {odom_age:.2f}s); '
                'skipping sample',
                throttle_duration_sec=5.0)
            return

        # AMCL is sampled only when fresh; otherwise its columns stay blank.
        amcl_age = self._age(self.amcl_rx, now)
        amcl_ok = amcl_age is not None and amcl_age <= self.max_age
        if self.require_amcl and not amcl_ok:
            self.get_logger().warn(
                'AMCL pose missing or stale and require_amcl is set; '
                'skipping sample', throttle_duration_sec=5.0)
            return

        imu_yaw = yaw_from_quaternion(q_from_msg(self.imu_msg.orientation))
        odom_yaw = yaw_from_quaternion(q_from_msg(self.odom_msg.pose.pose.orientation))

        if self.t_zero is None:
            self.t_zero = now
            self.get_logger().info(
                'Aligned at imu_yaw={:.2f} deg, odom_yaw={:.2f} deg'.format(
                    math.degrees(imu_yaw), math.degrees(odom_yaw)))

        imu_rel = self.imu_track.update(imu_yaw)
        odom_rel = self.odom_track.update(odom_yaw)
        imu_odom_err = math.degrees(imu_rel - odom_rel)
        self.imu_odom.add(imu_odom_err)

        imu_rate = self.imu_msg.angular_velocity.z
        odom_rate = self.odom_msg.twist.twist.angular.z

        stamp = self.odom_msg.header.stamp
        row = {
            'wall_time': datetime.now().isoformat(timespec='milliseconds'),
            'stamp_s': round(stamp.sec + stamp.nanosec * 1e-9, 4),
            'elapsed_s': round((now - self.t_zero).nanoseconds * 1e-9, 3),
            'imu_yaw_deg': round(math.degrees(imu_yaw), 4),
            'odom_yaw_deg': round(math.degrees(odom_yaw), 4),
            'imu_yaw_rel_deg': round(math.degrees(imu_rel), 4),
            'odom_yaw_rel_deg': round(math.degrees(odom_rel), 4),
            'imu_odom_error_deg': round(imu_odom_err, 4),
            'imu_yaw_rate_dps': round(math.degrees(imu_rate), 4),
            'odom_yaw_rate_dps': round(math.degrees(odom_rate), 4),
            'yaw_rate_error_dps': round(math.degrees(imu_rate - odom_rate), 4),
            'odom_x': round(self.odom_msg.pose.pose.position.x, 4),
            'odom_y': round(self.odom_msg.pose.pose.position.y, 4),
            'odom_lin_vel': round(self.odom_msg.twist.twist.linear.x, 4),
            'imu_age_s': round(imu_age, 3),
            'odom_age_s': round(odom_age, 3),
            # Blank unless AMCL is live, so gaps are visible rather than faked.
            'amcl_yaw_deg': '',
            'amcl_yaw_rel_deg': '',
            'imu_amcl_error_deg': '',
            'odom_amcl_error_deg': '',
            'amcl_x': '',
            'amcl_y': '',
            'amcl_age_s': '',
        }

        if amcl_ok:
            amcl_yaw = yaw_from_quaternion(q_from_msg(self.amcl_msg.pose.orientation))
            amcl_rel = self.amcl_track.update(amcl_yaw)
            if self.amcl_baseline is None:
                self.amcl_baseline = (imu_rel, odom_rel)
            imu_base, odom_base = self.amcl_baseline
            imu_amcl_err = math.degrees((imu_rel - imu_base) - amcl_rel)
            odom_amcl_err = math.degrees((odom_rel - odom_base) - amcl_rel)
            self.imu_amcl.add(imu_amcl_err)
            self.odom_amcl.add(odom_amcl_err)
            row.update({
                'amcl_yaw_deg': round(math.degrees(amcl_yaw), 4),
                'amcl_yaw_rel_deg': round(math.degrees(amcl_rel), 4),
                'imu_amcl_error_deg': round(imu_amcl_err, 4),
                'odom_amcl_error_deg': round(odom_amcl_err, 4),
                'amcl_x': round(self.amcl_msg.pose.position.x, 4),
                'amcl_y': round(self.amcl_msg.pose.position.y, 4),
                'amcl_age_s': round(amcl_age, 3),
            })
            self.amcl_rows += 1

        self.writer.writerow(row)
        self.csv_file.flush()  # survive a kill -9 mid-test
        self.rows_written += 1

    def _log_summary(self):
        if self.imu_odom.count == 0:
            return
        elapsed = (self.get_clock().now() - self.t_zero).nanoseconds * 1e-9
        drift = (self.imu_odom.last / elapsed * 60.0) if elapsed > 1.0 else 0.0
        self.get_logger().info(
            'rows={} (amcl {}) | {} | {} | {} | imu-odom drift {:+.2f} deg/min'
            .format(self.rows_written, self.amcl_rows,
                    self.imu_odom.summary(), self.imu_amcl.summary(),
                    self.odom_amcl.summary(), drift))

    def close(self):
        if not self.csv_file.closed:
            self.csv_file.close()
        # By the time this runs the context is usually already torn down by the
        # signal handler, so rosout is gone; print the summary instead.
        print('\n[yaw_comparator] Wrote {} rows ({} with AMCL) to {}'.format(
            self.rows_written, self.amcl_rows, self.output_path))
        for stats in (self.imu_odom, self.imu_amcl, self.odom_amcl):
            print('[yaw_comparator] {}'.format(stats.summary()))


def main(args=None):
    rclpy.init(args=args)
    node = YawComparator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
