"""Open the Robocon 2027 competition field in Gazebo Classic."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robocon2027_description")
    gazebo_models = PathJoinSubstitution([package_share, "gazebo_models"])
    default_world = PathJoinSubstitution(
        [gazebo_models, "robocon_track.world"]
    )
    world = LaunchConfiguration("world")
    gazebo_env = {"GAZEBO_MODEL_PATH": gazebo_models}

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world containing the Robocon 2027 field.",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    world,
                    "-s",
                    "libgazebo_ros_init.so",
                ],
                additional_env=gazebo_env,
                output="screen",
            ),
            ExecuteProcess(
                cmd=["gzclient"],
                additional_env=gazebo_env,
                condition=IfCondition(LaunchConfiguration("gui")),
                output="screen",
            ),
        ]
    )
# ros2 launch robocon2027_description robocon2027_world.launch.py