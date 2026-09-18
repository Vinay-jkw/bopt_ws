#!/usr/bin/env python3
"""
pose_scope_gui.py - live graphical view of orientation + speed from IMU and odometry.

A graphical replacement for pose_scope.py: same data, drawn with matplotlib
instead of ASCII, in a real window. Made for debugging on a laptop with a
screen (e.g. bench-testing an IMU by hand, off the robot).

Panels:
    1. Compass          - odom yaw (solid arrow) vs IMU yaw (hollow arrow),
                           both zeroed at startup so they start aligned.
    2. Angular rate      - wz from odom vs wz (angular_velocity.z) from IMU,
                           scrolling over the last N seconds. This is the
                           pair you actually care about matching.
    3. Yaw error         - odom_yaw - imu_yaw, scrolling. Watch this climb
                           on its own while both wz traces sit at 0 -- that's
                           the "orientation drifts while stationary" bug.
    4. Text readout      - live numbers: rates (Hz), speed, yaw values,
                           gyro saturation warning.

Usage:
    python3 pose_scope_gui.py
    python3 pose_scope_gui.py --imu /imu/data --odom /odometry/filtered
    python3 pose_scope_gui.py --odom /odom --window 20

Needs: pip install matplotlib --break-system-packages   (if not already present)
"""

import argparse
import csv
import math
import time
from collections import deque
from datetime import datetime

import matplotlib
matplotlib.use('TkAgg')  # change to 'Qt5Agg' if you prefer / have PySide6-Qt installed
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# Typical WIT gyro full-scale range, used only to flag likely saturation
# in the text readout. Adjust if you've reconfigured the sensor's FSR.
GYRO_SAT_RAD_S = math.radians(250) * 0.97  # ~90% of 250 deg/s default range


def yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def roll_from_quat(q):
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    return math.atan2(sinr, cosr)


def pitch_from_quat(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


class MotionLogger:
    """Writes one CSV row per IMU message (IMU is the focus of the log),
    paired with whatever the latest odom reading is at that instant.

    yaw_imu_deg and gz_imu (angular_velocity.z) are placed first since
    those are the two fields you actually care about -- everything else
    is there for context if you need to dig further later.
    """

    def __init__(self, path):
        self.path = path
        self.file = open(path, 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow([
            't_rel_s', 'stamp_wall',
            # -- focus columns --
            'yaw_imu_deg', 'gz_imu_rad_s',
            'yaw_odom_deg', 'wz_odom_rad_s',
            # -- rest of IMU, for reference --
            'roll_imu_deg', 'pitch_imu_deg',
            'qx', 'qy', 'qz', 'qw',
            'gx_imu_rad_s', 'gy_imu_rad_s',
            'ax_imu', 'ay_imu', 'az_imu',
            # -- odom extras --
            'speed_odom_m_s',
        ])
        self.n = 0

    def write(self, t_rel, imu_msg, node):
        q = imu_msg.orientation
        yaw = math.degrees(node.imu_yaw) if node.imu_yaw is not None else float('nan')
        roll = math.degrees(roll_from_quat(q))
        pitch = math.degrees(pitch_from_quat(q))
        av = imu_msg.angular_velocity
        la = imu_msg.linear_acceleration

        odom_yaw = math.degrees(node.odom_yaw) if node.odom_yaw is not None else float('nan')

        self.writer.writerow([
            f'{t_rel:.4f}', datetime.now().isoformat(timespec='milliseconds'),
            f'{yaw:.4f}', f'{av.z:.5f}',
            f'{odom_yaw:.4f}', f'{node.wz:.5f}',
            f'{roll:.4f}', f'{pitch:.4f}',
            f'{q.x:.6f}', f'{q.y:.6f}', f'{q.z:.6f}', f'{q.w:.6f}',
            f'{av.x:.5f}', f'{av.y:.5f}',
            f'{la.x:.6f}', f'{la.y:.6f}', f'{la.z:.6f}',
            f'{node.speed:.4f}',
        ])
        self.n += 1
        if self.n % 20 == 0:
            self.file.flush()

    def close(self):
        self.file.flush()
        self.file.close()


class PoseScopeNode(Node):
    """Pure data-collection node. No drawing here -- matplotlib owns the window
    and pulls from this node's latest values on a timer."""

    def __init__(self, imu_topic, odom_topic, window_s, logger=None):
        super().__init__('pose_scope_gui')
        self.logger = logger

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.imu_topic = imu_topic
        self.odom_topic = odom_topic

        self.imu_yaw0 = None
        self.odom_yaw0 = None

        self.imu_yaw = None
        self.odom_yaw = None
        self.gz = 0.0
        self.wz = 0.0
        self.speed = 0.0

        self.imu_stamps = deque(maxlen=200)
        self.odom_stamps = deque(maxlen=200)

        # scrolling history for the plots: (t, value)
        self.t0 = time.monotonic()
        self.window_s = window_s
        self.hist_t = deque()
        self.hist_wz_odom = deque()
        self.hist_wz_imu = deque()
        self.hist_err = deque()

        self.max_abs_gz_seen = 0.0

        self.create_subscription(Imu, imu_topic, self.on_imu, sensor_qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, sensor_qos)

    def on_imu(self, msg):
        self.imu_stamps.append(time.monotonic())
        yaw = yaw_from_quat(msg.orientation)
        if self.imu_yaw0 is None:
            self.imu_yaw0 = yaw
        self.imu_yaw = wrap(yaw - self.imu_yaw0)
        self.gz = msg.angular_velocity.z
        self.max_abs_gz_seen = max(self.max_abs_gz_seen, abs(self.gz))
        self._push_history()
        if self.logger is not None:
            self.logger.write(time.monotonic() - self.t0, msg, self)

    def on_odom(self, msg):
        self.odom_stamps.append(time.monotonic())
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        if self.odom_yaw0 is None:
            self.odom_yaw0 = yaw
        self.odom_yaw = wrap(yaw - self.odom_yaw0)
        self.wz = msg.twist.twist.angular.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        self.speed = math.hypot(vx, vy)
        self._push_history()

    def _push_history(self):
        t = time.monotonic() - self.t0
        err = None
        if self.odom_yaw is not None and self.imu_yaw is not None:
            err = math.degrees(wrap(self.odom_yaw - self.imu_yaw))
        self.hist_t.append(t)
        self.hist_wz_odom.append(self.wz)
        self.hist_wz_imu.append(self.gz)
        self.hist_err.append(err if err is not None else float('nan'))
        while self.hist_t and t - self.hist_t[0] > self.window_s:
            self.hist_t.popleft()
            self.hist_wz_odom.popleft()
            self.hist_wz_imu.popleft()
            self.hist_err.popleft()

    def rate(self, stamps):
        if len(stamps) < 2:
            return 0.0
        dt = stamps[-1] - stamps[0]
        return (len(stamps) - 1) / dt if dt > 0 else 0.0


def build_figure(node, window_s):
    fig = plt.figure(figsize=(11, 7))
    fig.canvas.manager.set_window_title('pose_scope_gui')
    gs = fig.add_gridspec(2, 2, height_ratios=[1.3, 1], width_ratios=[1, 1.4])

    ax_compass = fig.add_subplot(gs[0, 0], projection='polar')
    ax_text = fig.add_subplot(gs[1, 0])
    ax_wz = fig.add_subplot(gs[0, 1])
    ax_err = fig.add_subplot(gs[1, 1])

    # --- compass setup ---
    ax_compass.set_theta_zero_location('N')
    ax_compass.set_theta_direction(-1)
    ax_compass.set_ylim(0, 1)
    ax_compass.set_yticklabels([])
    ax_compass.set_title('yaw: odom (solid) vs imu (hollow)', fontsize=10)
    (odom_arrow,) = ax_compass.plot([0, 0], [0, 1], '-', lw=3, color='#1f77b4', label='odom')
    (imu_arrow,) = ax_compass.plot([0, 0], [0, 0.75], '--', lw=2, color='#d62728', label='imu')
    ax_compass.legend(loc='upper right', fontsize=8, bbox_to_anchor=(1.3, 1.1))

    # --- text panel ---
    ax_text.axis('off')
    text_obj = ax_text.text(0.02, 0.98, '', va='top', ha='left',
                             family='monospace', fontsize=10)

    # --- angular rate plot ---
    ax_wz.set_title('angular rate  (rad/s)', fontsize=10)
    ax_wz.set_xlim(0, window_s)
    ax_wz.set_ylim(-2.0, 2.0)
    ax_wz.axhline(0, color='gray', lw=0.5)
    (wz_odom_line,) = ax_wz.plot([], [], color='#1f77b4', label='odom wz')
    (wz_imu_line,) = ax_wz.plot([], [], color='#d62728', label='imu gz')
    ax_wz.legend(loc='upper right', fontsize=8)
    ax_wz.set_xlabel('rolling window (s)')

    # --- yaw error plot ---
    ax_err.set_title('yaw error = odom - imu  (deg)', fontsize=10)
    ax_err.set_xlim(0, window_s)
    ax_err.set_ylim(-30, 30)
    ax_err.axhline(0, color='gray', lw=0.5)
    (err_line,) = ax_err.plot([], [], color='#2ca02c')
    ax_err.set_xlabel('rolling window (s)')

    fig.tight_layout()

    artists = dict(odom_arrow=odom_arrow, imu_arrow=imu_arrow, text_obj=text_obj,
                   wz_odom_line=wz_odom_line, wz_imu_line=wz_imu_line, err_line=err_line,
                   ax_err=ax_err)
    return fig, artists


def make_update(node, artists, window_s):
    def update(_frame):
        rclpy.spin_once(node, timeout_sec=0.0)

        oy = node.odom_yaw
        iy = node.imu_yaw
        if oy is not None:
            artists['odom_arrow'].set_data([oy, oy], [0, 1])
        if iy is not None:
            artists['imu_arrow'].set_data([iy, iy], [0, 0.75])

        if node.hist_t:
            t0 = node.hist_t[0]
            xs = [t - t0 for t in node.hist_t]
            artists['wz_odom_line'].set_data(xs, list(node.hist_wz_odom))
            artists['wz_imu_line'].set_data(xs, list(node.hist_wz_imu))
            artists['err_line'].set_data(xs, list(node.hist_err))
            # auto-scale y on the error plot so drift is always visible
            finite_err = [e for e in node.hist_err if e == e]  # drop NaN
            if finite_err:
                lo, hi = min(finite_err), max(finite_err)
                pad = max(5.0, (hi - lo) * 0.2)
                artists['ax_err'].set_ylim(lo - pad, hi + pad)

        sat_flag = ''
        if node.max_abs_gz_seen > GYRO_SAT_RAD_S:
            sat_flag = '  ** GYRO NEAR/AT SATURATION -- turn slower to confirm **'

        err_now = node.hist_err[-1] if node.hist_err else float('nan')
        txt = (
            f"odom topic  {node.odom_topic:<22} {node.rate(node.odom_stamps):5.1f} Hz\n"
            f"imu  topic  {node.imu_topic:<22} {node.rate(node.imu_stamps):5.1f} Hz\n"
            f"\n"
            f"odom yaw    {math.degrees(oy) if oy is not None else float('nan'):+8.2f} deg\n"
            f"imu  yaw    {math.degrees(iy) if iy is not None else float('nan'):+8.2f} deg\n"
            f"yaw error   {err_now:+8.2f} deg\n"
            f"\n"
            f"odom wz     {node.wz:+7.3f} rad/s\n"
            f"imu  gz     {node.gz:+7.3f} rad/s\n"
            f"w diff      {node.wz - node.gz:+7.3f} rad/s\n"
            f"speed       {node.speed:6.3f} m/s\n"
            f"\n"
            f"peak |gz| seen  {node.max_abs_gz_seen:6.3f} rad/s "
            f"({math.degrees(node.max_abs_gz_seen):5.1f} deg/s)\n"
            f"{sat_flag}\n"
        )
        if node.logger is not None:
            txt += f"\nLOGGING -> {node.logger.path}  ({node.logger.n} rows)"
        artists['text_obj'].set_text(txt)

        return list(artists.values())

    return update


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--imu', default='/imu/data')
    p.add_argument('--odom', default='/odom')
    p.add_argument('--window', type=float, default=15.0, help='seconds of scrolling history shown')
    p.add_argument('--redraw-hz', type=float, default=20.0)
    p.add_argument('--log', nargs='?', const='__auto__', default=None,
                    help='log every IMU sample (+ paired odom) to CSV. '
                         'Pass a path, or leave blank to auto-name by timestamp.')
    args, ros_args = p.parse_known_args()

    logger = None
    if args.log is not None:
        log_path = args.log
        if log_path == '__auto__':
            log_path = 'motion_log_%s.csv' % datetime.now().strftime('%Y%m%d_%H%M%S')
        logger = MotionLogger(log_path)
        print(f'[pose_scope_gui] logging every IMU sample to: {log_path}')

    rclpy.init(args=ros_args)
    node = PoseScopeNode(args.imu, args.odom, args.window, logger=logger)

    fig, artists = build_figure(node, args.window)
    update = make_update(node, artists, args.window)

    ani = animation.FuncAnimation(
        fig, update, interval=1000.0 / args.redraw_hz, blit=False, cache_frame_data=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        if logger is not None:
            logger.close()
            print(f'[pose_scope_gui] wrote {logger.n} rows to {logger.path}')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()