#!/usr/bin/env python3
"""Exercise the real GICP node with synthetic scans, without running Gazebo.

Run after sourcing scripts/env.sh, with an unused ROS_DOMAIN_ID.
Temporary PCD targets are created here; existing maps are never written.
"""
import math
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField


ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
EXECUTABLE = ROOT / 'install/robocon_cloud_localization/lib/robocon_cloud_localization/loop_closure_pgo'
CONFIG = ROOT / 'src/robocon_cloud_localization/config/localization.yaml'
rng = np.random.default_rng(42)
# Three perpendicular planes and an asymmetric raised patch constrain all axes.
uv = rng.uniform(-1.5, 1.5, (800, 2))
source = np.concatenate([
    np.column_stack([uv, np.zeros(len(uv))]),
    np.column_stack([np.full(len(uv), 1.5), uv]),
    np.column_stack([uv[:, 0], np.full(len(uv), -1.5), uv[:, 1]]),
    rng.uniform([-.8, .1, .4], [-.1, .6, .8], (300, 3)),
])
yaw = .03
rotation = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                     [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
translation = np.array([.04, -.03, .45])
target = source @ rotation.T + translation


class Probe:
    def __init__(self):
        self.node = rclpy.create_node('initial_alignment_regression')
        self.lio = self.node.create_publisher(Odometry, '/fast_lio/position', 10)
        self.wheel = self.node.create_publisher(Odometry, '/odom', 10)
        self.cloud = self.node.create_publisher(PointCloud2, '/livox/lidar', qos_profile_sensor_data)
        self.poses = []
        self.sub = self.node.create_subscription(Odometry, '/localization/pose', self.poses.append, 10)

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=.02)

    def scan(self, points):
        stamp = self.node.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.pose.pose.orientation.w = 1.
        self.lio.publish(odom)
        self.wheel.publish(odom)
        self.spin(.08)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = 'mid360_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                      for i, k in enumerate(('x', 'y', 'z', 'intensity'))]
        msg.point_step = 16
        msg.row_step = msg.width * 16
        msg.is_dense = True
        msg.data = np.column_stack([points, np.zeros(len(points))]).astype('<f4').tobytes()
        count = len(self.poses)
        self.cloud.publish(msg)
        deadline = time.monotonic() + 6
        while len(self.poses) == count and time.monotonic() < deadline:
            self.spin(.03)
        assert len(self.poses) > count, 'No localization output'
        p = self.poses[-1].pose.pose.position
        return np.array([p.x, p.y, p.z])

    def case(self, name, map_path, callback, extra=()):
        with (OUT / f'{name}.log').open('w') as logfile:
            proc = subprocess.Popen([
                str(EXECUTABLE), '--ros-args', '--params-file', str(CONFIG),
                '-p', 'use_sim_time:=false', '-p', 'mode:=localize',
                '-p', f'map_file:={map_path}', *extra,
            ], stdout=logfile, stderr=subprocess.STDOUT)
            try:
                self.poses.clear()
                deadline = time.monotonic() + 10
                while self.cloud.get_subscription_count() == 0 and time.monotonic() < deadline:
                    self.spin(.05)
                assert proc.poll() is None and self.cloud.get_subscription_count(), 'Node not ready'
                self.spin(.3)
                callback()
                print(f'PASS {name}', flush=True)
            finally:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                self.spin(.4)


def main():
    rclpy.init()
    probe = Probe()
    try:
        with tempfile.TemporaryDirectory(prefix='icp-initial-alignment-') as tmp:
            path = Path(tmp) / 'target.pcd'
            header = ('VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n'
                      'TYPE F F F F\nCOUNT 1 1 1 1\n'
                      f'WIDTH {len(target)}\nHEIGHT 1\nPOINTS {len(target)}\nDATA binary\n')
            path.write_bytes(header.encode() + np.column_stack([
                target, np.zeros(len(target))]).astype('<f4').tobytes())

            def initialize_and_track():
                for _ in range(2):
                    assert np.linalg.norm(probe.scan(source)) < 1e-5, 'Accepted before 3 scans'
                aligned = probe.scan(source)
                assert np.linalg.norm(aligned-translation) < .025, aligned
                # The same scan now implies a further 0.50 m height jump.
                # It is below the startup limit but above the tracking limit.
                for _ in range(4):
                    result = probe.scan(source - [0, 0, .50])
                    assert np.linalg.norm(result-aligned) < 1e-5, result
                assert 'GICP tracking correction rejected' in (OUT/'initialize_and_track.log').read_text()

            probe.case('initialize_and_track', path, initialize_and_track)

            def reject_large_initial():
                for _ in range(4):
                    assert np.linalg.norm(probe.scan(source)) < 1e-5
                assert 'GICP initializing correction rejected' in (OUT/'reject_large_initial.log').read_text()

            probe.case('reject_large_initial', path, reject_large_initial,
                       ('-p', 'localization_initial_max_correction:=0.20'))

            def reject_partial_overlap():
                distant = rng.uniform([10, 10, 10], [15, 15, 15], (7000, 3))
                partial = np.concatenate([source, distant])
                for _ in range(4):
                    assert np.linalg.norm(probe.scan(partial)) < 1e-5
                assert 'initialization rejected: overlap=' in (OUT/'reject_partial_overlap.log').read_text()

            probe.case('reject_partial_overlap', path, reject_partial_overlap)
    finally:
        probe.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
