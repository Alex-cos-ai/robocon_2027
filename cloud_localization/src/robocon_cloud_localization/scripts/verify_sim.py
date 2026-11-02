#!/usr/bin/env python3
"""Exercise simulated LiDAR, IMU, FAST-LIO2 and loop-closure/map-save service."""
import argparse
import time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from std_srvs.srv import Trigger


class Check(Node):
    def __init__(self):
        super().__init__('robocon_sim_verifier')
        self.data = {'cloud': 0, 'points': 0, 'imu': 0, 'lio': 0, 'wheel': 0,
                     'truth': 0, 'path': 0, 'localized': 0, 'icp_updates': 0}
        self.first_truth = None
        self.last_lio = None
        self.last_wheel = None
        self.last_localized = None
        self.lio_travel = 0.0
        self.wheel_travel = 0.0
        self.localized_travel = 0.0
        self.travel = 0.0
        self.truth_path = 0.0
        self.last_truth = None
        self.outside_field = 0
        self.icp_fitness = []
        self.cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(PointCloud2, '/livox/lidar', self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', lambda _: self.inc('imu'), qos_profile_sensor_data)
        self.create_subscription(Odometry, '/fast_lio/position', self.on_lio, 10)
        self.create_subscription(Odometry, '/odom', self.on_wheel, 10)
        self.create_subscription(Odometry, '/localization/pose', self.on_localized, 10)
        self.create_subscription(Odometry, '/simulation/ground_truth', self.on_truth, 10)
        self.create_subscription(Path, '/localization/optimized_path', lambda m: self.set('path', len(m.poses)), 10)

    def inc(self, key): self.data[key] += 1
    def set(self, key, value): self.data[key] = value
    def on_cloud(self, msg):
        self.inc('cloud')
        self.data['points'] = msg.width * msg.height
    def on_truth(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.first_truth is None: self.first_truth = xy
        if self.last_truth is not None:
            self.truth_path += ((xy[0] - self.last_truth[0]) ** 2 +
                                (xy[1] - self.last_truth[1]) ** 2) ** 0.5
        self.travel = ((xy[0] - self.first_truth[0]) ** 2 + (xy[1] - self.first_truth[1]) ** 2) ** 0.5
        self.last_truth = xy
        if min(xy) < -5.25 or max(xy) > 5.25:
            self.outside_field += 1
        self.inc('truth')
    def on_lio(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_lio is not None:
            self.lio_travel += ((xy[0] - self.last_lio[0]) ** 2 + (xy[1] - self.last_lio[1]) ** 2) ** 0.5
        self.last_lio = xy
        self.inc('lio')
    def on_wheel(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_wheel is not None:
            self.wheel_travel += ((xy[0] - self.last_wheel[0]) ** 2 + (xy[1] - self.last_wheel[1]) ** 2) ** 0.5
        self.last_wheel = xy
        self.inc('wheel')
    def on_localized(self, msg):
        p = msg.pose.pose.position
        xy = (p.x, p.y)
        if self.last_localized is not None:
            self.localized_travel += ((xy[0] - self.last_localized[0]) ** 2 +
                                      (xy[1] - self.last_localized[1]) ** 2) ** 0.5
        self.last_localized = xy
        self.inc('localized')
        if msg.pose.covariance[0] < 0.99:
            self.inc('icp_updates')
            self.icp_fitness.append(msg.pose.covariance[0])
    def drive(self, x=0.0, y=0.0, yaw=0.0):
        m = Twist(); m.linear.x = x; m.linear.y = y; m.angular.z = yaw; self.cmd.publish(m)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20.0)
    parser.add_argument('--save-map', action='store_true', help='call /save_map after collecting validation data')
    parser.add_argument('--localize', action='store_true', help='require ICP/GICP pose output from a loaded map')
    parser.add_argument('--require-loop', action='store_true', help='require at least one verified loop in the saved map')
    args = parser.parse_args()
    if args.duration < 8: parser.error('duration must be at least 8 seconds')
    rclpy.init(); node = Check(); start = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - start < args.duration:
            elapsed = time.monotonic() - start
            straight_time = 0.80 / 0.22
            turn_time = (3.141592653589793 / 2.0) / 0.90
            phase = elapsed % (straight_time + turn_time)
            if phase < straight_time:
                node.drive(x=0.22)
            else:
                node.drive(yaw=0.90)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.drive()
        for _ in range(10): rclpy.spin_once(node, timeout_sec=0.02)
        mean_icp_fitness = (
            sum(node.icp_fitness) / len(node.icp_fitness) if node.icp_fitness else float('nan')
        )
        print(f"simulation evidence: scans={node.data['cloud']} points/scan={node.data['points']} "
              f"IMU={node.data['imu']} FAST-LIO2 odometry={node.data['lio']} "
              f"wheel odometry={node.data['wheel']} "
              f"ground-truth samples={node.data['truth']} optimized poses={node.data['path']} "
              f"localized poses={node.data['localized']} ICP-refined poses={node.data['icp_updates']} "
              f"mean accepted ICP fitness={mean_icp_fitness:.3f}m "
              f"out-of-field samples={node.outside_field} "
              f"localization path length={node.localized_travel:.2f}m "
              f"truth displacement={node.travel:.2f}m truth path length={node.truth_path:.2f}m "
              f"wheel path length={node.wheel_travel:.2f}m FAST-LIO2 path length={node.lio_travel:.2f}m")
        failures = []
        for key in ('cloud', 'imu', 'lio', 'wheel', 'truth'):
            if not node.data[key]: failures.append(f'no {key} messages')
        if node.data['points'] < 100: failures.append('simulated MID360 scan has fewer than 100 points')
        if node.truth_path < 0.30: failures.append('robot did not move far enough to create a useful scan trajectory')
        if args.localize and not node.data['localized']: failures.append('GICP relocalizer did not publish a pose')
        if args.localize and not node.data['icp_updates']:
            failures.append('no GICP correction passed the fitness and continuity gates')
        if node.outside_field:
            failures.append('Waffle footprint crossed the Robocon field safety boundary')
        if args.save_map and args.localize: parser.error('--save-map cannot be used with --localize')
        if args.save_map:
            client = node.create_client(Trigger, '/save_map')
            if not client.wait_for_service(timeout_sec=5): failures.append('/save_map service unavailable')
            else:
                request = client.call_async(Trigger.Request())
                end = time.monotonic() + 20
                while rclpy.ok() and not request.done() and time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if not request.done(): failures.append('timed out waiting for /save_map')
                else:
                    response = request.result()
                    print(f'map save: success={response.success}: {response.message}')
                    if not response.success: failures.append(response.message)
                    if response.success and args.require_loop:
                        from pathlib import Path
                        info = Path.cwd() / 'maps' / 'fastlio_mid360_downward' / 'map_info.txt'
                        text = info.read_text() if info.exists() else ''
                        if 'loop_closures=' not in text or int(text.split('loop_closures=', 1)[1].splitlines()[0]) < 1:
                            failures.append('no verified Scan Context + GICP loop was added to iSAM2')
        if failures: raise RuntimeError('; '.join(failures))
        print('simulation pipeline checks passed')
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__': main()
