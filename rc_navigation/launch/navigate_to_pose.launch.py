"""Send a single goal to an already running navigation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", description="Target x in map, meters."),
        DeclareLaunchArgument("y", description="Target y in map, meters."),
        DeclareLaunchArgument("yaw", default_value="0.0", description="Target yaw, radians."),
        DeclareLaunchArgument("timeout", default_value="180.0"),
        DeclareLaunchArgument("server_timeout", default_value="60.0"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="rc_navigation",
            executable="navigate_to_pose.py",
            output="screen",
            emulate_tty=True,
            parameters=[{"use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool)}],
            arguments=[
                "--x", LaunchConfiguration("x"),
                "--y", LaunchConfiguration("y"),
                "--yaw", LaunchConfiguration("yaw"),
                "--timeout", LaunchConfiguration("timeout"),
                "--server-timeout", LaunchConfiguration("server_timeout"),
            ],
        ),
    ])
