#!/usr/bin/env python3
"""Send one map-frame Nav2 goal: --x METERS --y METERS --yaw RADIANS."""

import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from tf2_ros import Buffer, TransformListener


class PointNavigator(Node):
    def __init__(self, options):
        super().__init__("point_navigator")
        self.options = options
        self.client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.goal_handle = None
        self.send_future = None
        self.result_future = None
        self.last_feedback = 0.0

    def spin_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok(context=self.context):
            if predicate():
                return True
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def feedback(self, message):
        if time.monotonic() - self.last_feedback < 2.0:
            return
        self.last_feedback = time.monotonic()
        data = message.feedback
        pose = data.current_pose.pose.position
        self.get_logger().info(
            f"当前位置 ({pose.x:.2f}, {pose.y:.2f})，"
            f"剩余路径 {data.distance_remaining:.2f} m，"
            f"恢复次数 {data.number_of_recoveries}"
        )

    def cancel(self):
        # A server may accept the goal just after the user interrupts the send.
        if self.goal_handle is None and self.send_future is not None:
            if not self.spin_until(self.send_future.done, 5.0):
                self.get_logger().error("目标应答超时，无法确认目标是否已被接受，请检查 Nav2。")
                return
            self.goal_handle = self.send_future.result()
        if self.goal_handle is None or not self.goal_handle.accepted:
            return
        if self.result_future is None:
            self.result_future = self.goal_handle.get_result_async()
        if self.result_future.done():
            return
        self.get_logger().info("正在取消导航目标……")
        future = self.goal_handle.cancel_goal_async()
        if not self.spin_until(future.done, 5.0) or not future.result().goals_canceling:
            self.get_logger().error("未确认目标取消，请检查 Nav2 当前任务状态。")
        elif self.spin_until(self.result_future.done, 5.0):
            self.get_logger().info("导航目标已结束。")
        else:
            self.get_logger().error("已请求取消，但未收到任务结束结果，请检查 Nav2。")

    def navigate(self):
        args = self.options
        self.get_logger().info("等待 Nav2 和 map → base_footprint 定位 TF……")
        if not self.spin_until(
            lambda: self.client.server_is_ready()
            and self.tf.can_transform("map", "base_footprint", rclpy.time.Time())
            and (not self.get_parameter("use_sim_time").value
                 or self.get_clock().now().nanoseconds > 0),
            args.server_timeout,
        ):
            self.get_logger().error("等待超时。请先启动导航，并在 RViz 设置初始位姿。")
            return 2

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)
        self.get_logger().info(
            f"发送目标：map x={args.x:.3f} m, y={args.y:.3f} m, "
            f"yaw={args.yaw:.3f} rad ({math.degrees(args.yaw):.1f}°)"
        )
        self.send_future = self.client.send_goal_async(goal, feedback_callback=self.feedback)
        if not self.spin_until(self.send_future.done, args.server_timeout):
            self.get_logger().error("等待目标应答超时。")
            self.cancel()
            return 2
        self.goal_handle = self.send_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error("Nav2 拒绝了目标。")
            return 3
        self.result_future = self.goal_handle.get_result_async()
        if not self.spin_until(self.result_future.done, args.timeout):
            self.get_logger().error(f"导航超过 {args.timeout:g} 秒，取消本次目标。")
            self.cancel()
            return 4
        status = self.result_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("已到达目标位置和朝向。")
            return 0
        if status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warning("导航已取消。")
            return 5
        self.get_logger().error(f"导航失败，动作状态码：{status}。请检查 Nav2 日志。")
        return 6


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, required=True, help="目标地图 x 坐标，米")
    parser.add_argument("--y", type=float, required=True, help="目标地图 y 坐标，米")
    parser.add_argument("--yaw", type=float, default=0.0, help="目标地图朝向，弧度，默认 0")
    parser.add_argument("--timeout", type=float, default=180.0, help="导航时限，墙钟秒")
    parser.add_argument("--server-timeout", type=float, default=60.0, help="等待就绪时限，墙钟秒")
    options = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if not all(math.isfinite(v) for v in vars(options).values()):
        parser.error("坐标、角度和超时必须是有限数值")
    if options.timeout <= 0 or options.server_timeout <= 0:
        parser.error("超时必须大于 0")
    # Keep ROS alive during Ctrl+C so the action cancellation can complete.
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    node = PointNavigator(options)
    try:
        return node.navigate()
    except KeyboardInterrupt:
        node.cancel()
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
