#!/usr/bin/env python3
"""Bounded teleop and repeatable perimeter scan route."""
import argparse
import math
import select
import sys
import termios
import time
import tty
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Driver(Node):
    def __init__(self):
        super().__init__('robocon_sim_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)
        self.current = Twist()

    def tick(self):
        self.pub.publish(self.current)

    def set(self, x=0.0, y=0.0, yaw=0.0):
        self.current = Twist()
        self.current.linear.x, self.current.linear.y = float(x), float(y)
        self.current.angular.z = float(yaw)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pattern', choices=['perimeter', 'manual'], default='perimeter')
    parser.add_argument('--side', type=float, default=2.4, help='square side in metres')
    parser.add_argument('--speed', type=float, default=0.30, help='translation speed in m/s')
    parser.add_argument('--yaw-rate', type=float, default=0.45, help='turn speed in rad/s')
    parser.add_argument('--loops', type=int, default=2)
    args = parser.parse_args()
    if args.side <= 0 or args.speed <= 0 or args.yaw_rate <= 0 or args.loops < 1:
        parser.error('side, speed, yaw-rate, and loops must be positive')
    rclpy.init()
    node = Driver()
    try:
        if args.pattern == 'manual':
            print('W/S forward/reverse, A/D rotate, Q/E drive in arcs, space stop, Ctrl-C quit')
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd) if sys.stdin.isatty() else None
            try:
                if old:
                    tty.setcbreak(fd)
                while rclpy.ok():
                    rclpy.spin_once(node, timeout_sec=0.01)
                    ready, _, _ = select.select([sys.stdin], [], [], 0.02)
                    if not ready:
                        continue
                    key = sys.stdin.read(1).lower()
                    if key == chr(3):
                        break
                    command = {'w': (0.30, 0, 0), 's': (-0.30, 0, 0), 'a': (0, 0, 0.45),
                               'd': (0, 0, -0.45), 'q': (0.20, 0, 0.45), 'e': (0.20, 0, -0.45)}
                    node.set(*command.get(key, (0, 0, 0)))
            finally:
                if old:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old)
        else:
            straight = args.side / args.speed
            turn = (math.pi / 2) / args.yaw_rate
            for lap in range(args.loops):
                node.get_logger().info(f'Starting square scan {lap + 1}/{args.loops}')
                for _ in range(4):
                    node.set(args.speed, 0, 0)
                    end = time.monotonic() + straight
                    while rclpy.ok() and time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    node.set()
                    settle = time.monotonic() + 0.35
                    while rclpy.ok() and time.monotonic() < settle:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    node.set(0, 0, args.yaw_rate)
                    end = time.monotonic() + turn
                    while rclpy.ok() and time.monotonic() < end:
                        rclpy.spin_once(node, timeout_sec=0.02)
                    node.set()
                    settle = time.monotonic() + 0.4
                    while rclpy.ok() and time.monotonic() < settle:
                        rclpy.spin_once(node, timeout_sec=0.02)
            node.get_logger().info('Perimeter scan complete; stopping the robot.')
    except KeyboardInterrupt:
        pass
    finally:
        node.set()
        for _ in range(8):
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
