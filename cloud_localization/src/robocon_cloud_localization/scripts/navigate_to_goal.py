#!/usr/bin/env python3
"""Start Nav2 MPPI (if needed) and send one map-frame goal pose.

Run after localization.launch.py:
  python3 src/robocon_cloud_localization/scripts/navigate_to_goal.py X Y --yaw YAW

Yaw is in radians. If this script starts Nav2, it shuts that Nav2 instance down
when the goal finishes; localization.launch.py remains running.
"""

from __future__ import annotations
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
from typing import Any


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send a fixed map-frame goal through Nav2's NavigateToPose action."
    )
    parser.add_argument("x", type=float, help="goal x in the map frame, metres")
    parser.add_argument("y", type=float, help="goal y in the map frame, metres")
    parser.add_argument(
        "--yaw", type=float, default=0.0, help="goal yaw in the map frame, radians (default: 0)"
    )
    parser.add_argument("--frame", default="map", help="goal frame (default: map)")
    parser.add_argument(
        "--timeout", type=float, default=300.0, help="goal timeout in seconds (default: 300)"
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=90.0,
        help="maximum seconds to wait for localization and Nav2 (default: 90)",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "nav2_mppi.yaml",
        help="Nav2 parameter overrides YAML",
    )
    return parser.parse_args()


def merge_dicts(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """Recursively apply override keys while retaining stock Nav2 settings."""
    merged = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged


def build_merged_nav2_params(config_path: Path) -> str:
    import yaml
    from ament_index_python.packages import get_package_share_directory

    config_path = Path(config_path)
    nav2_defaults = (
        Path(get_package_share_directory("nav2_bringup")) / "params" / "nav2_params.yaml"
    )
    if not nav2_defaults.is_file():
        raise FileNotFoundError(f"Nav2 default parameters not found: {nav2_defaults}")
    if not config_path.is_file():
        raise FileNotFoundError(f"MPPI parameter file not found: {config_path}")

    with nav2_defaults.open("r", encoding="utf-8") as stream:
        defaults = yaml.safe_load(stream)
    with config_path.open("r", encoding="utf-8") as stream:
        overrides = yaml.safe_load(stream)
    merged = merge_dicts(defaults, overrides)

    # The stock file's FollowPath subtree is for DWB. Replace that subtree so
    # none of DWB's critic keys leak into the MPPI plugin configuration.
    controller_overrides = overrides["controller_server"]["ros__parameters"]
    merged["controller_server"]["ros__parameters"]["FollowPath"] = controller_overrides[
        "FollowPath"
    ]

    fd, path = tempfile.mkstemp(prefix="robocon_nav2_mppi_", suffix=".yaml")
    with os.fdopen(fd, "w", encoding="utf-8") as stream:
        yaml.safe_dump(merged, stream, sort_keys=False)
    return path


def main() -> int:
    args = parse_args()
    try:
        import rclpy
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
        from tf2_ros import Buffer, TransformException, TransformListener
    except ImportError as exc:
        print(
            "ROS 2/Nav2 Python modules are unavailable. From the workspace root, "
            "source install/setup.bash and run this script again.\n"
            f"Import error: {exc}",
            file=sys.stderr,
        )
        return 2

    class GoalClient(Node):
        def __init__(self) -> None:
            super().__init__(
                "robocon_mppi_goal_client",
                parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)],
                automatically_declare_parameters_from_overrides=True,
            )
            self.action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
            self.map_seen = False
            self.create_subscription(
                PointCloud2,
                "/localization/optimized_map",
                self._on_map,
                1,
            )
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        def _on_map(self, _message: PointCloud2) -> None:
            self.map_seen = True

        def localization_ready(self, frame: str) -> bool:
            try:
                self.tf_buffer.lookup_transform(
                    frame,
                    "base_footprint",
                    rclpy.time.Time(),
                )
                return True
            except TransformException:
                return False

    nav2_process: subprocess.Popen[bytes] | None = None
    merged_params: str | None = None
    nav2_ready = False
    rclpy.init()
    node = GoalClient()
    try:
        print("Waiting briefly for an existing Nav2 NavigateToPose action server...")
        existing_nav2 = node.action_client.wait_for_server(timeout_sec=2.0)
        if not existing_nav2:
            merged_params = build_merged_nav2_params(args.config.resolve())
            command = [
                "ros2",
                "launch",
                "nav2_bringup",
                "navigation_launch.py",
                f"params_file:={merged_params}",
                "use_sim_time:=true",
                "autostart:=true",
            ]
            print("Starting Nav2 navigation with MPPI; localization.launch.py remains running.")
            try:
                nav2_process = subprocess.Popen(command, start_new_session=True)
            except FileNotFoundError:
                print("Could not find `ros2`; source the workspace setup file first.", file=sys.stderr)
                return 2
        else:
            print("Using the already-running Nav2 NavigateToPose action server.")

        deadline = time.monotonic() + args.startup_timeout
        map_ready = False
        tf_ready = False
        action_ready = False
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            map_ready = node.map_seen
            tf_ready = node.localization_ready(args.frame)
            action_ready = node.action_client.server_is_ready()
            if nav2_process is not None and nav2_process.poll() is not None:
                print(
                    f"Nav2 launch exited with code {nav2_process.returncode} before becoming ready.",
                    file=sys.stderr,
                )
                return 3
            if map_ready and tf_ready and action_ready:
                nav2_ready = True
                break

        if not nav2_ready:
            missing = []
            if not map_ready:
                missing.append("/localization/optimized_map")
            if not tf_ready:
                missing.append(f"TF {args.frame} -> base_footprint")
            if not action_ready:
                missing.append("/navigate_to_pose")
            print(
                "Timed out waiting for: " + ", ".join(missing) + ". "
                "Confirm localization.launch.py is running and has published its map/TF.",
                file=sys.stderr,
            )
            return 4

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = args.frame
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.position.z = 0.0
        goal.pose.pose.orientation.z = math.sin(args.yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(args.yaw / 2.0)

        print(
            f"Sending goal: frame={args.frame}, x={args.x:.3f}, y={args.y:.3f}, "
            f"yaw={args.yaw:.3f} rad"
        )
        send_future = node.action_client.send_goal_async(goal)
        deadline = time.monotonic() + args.startup_timeout
        while not send_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not send_future.done():
            print("Timed out waiting for Nav2 to accept the goal.", file=sys.stderr)
            return 5

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("Nav2 rejected the goal.", file=sys.stderr)
            return 6

        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + args.timeout
        while not result_future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result_future.done():
            goal_handle.cancel_goal_async()
            print(f"Navigation timed out after {args.timeout:.1f} seconds.", file=sys.stderr)
            return 7

        wrapped_result = result_future.result()
        if wrapped_result.status == GoalStatus.STATUS_SUCCEEDED:
            print("Navigation succeeded.")
            return 0
        print(f"Navigation finished with action status {wrapped_result.status}.", file=sys.stderr)
        return 8
    finally:
        if nav2_process is not None:
            try:
                os.killpg(nav2_process.pid, signal.SIGINT)
                nav2_process.wait(timeout=10.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(nav2_process.pid, signal.SIGTERM)
                    nav2_process.wait(timeout=3.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    pass
        if merged_params is not None:
            try:
                os.unlink(merged_params)
            except FileNotFoundError:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
