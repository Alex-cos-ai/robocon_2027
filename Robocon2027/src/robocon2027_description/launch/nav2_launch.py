"""Start the Robocon world, robot model, and (optionally) Nav2.

This workspace uses Gazebo Classic 11.  Nav2 is disabled by default because
the repository does not contain a map yet; pass ``start_nav2:=true`` and a
valid ``map:=...`` file when localization is needed.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robocon2027_description")
    gazebo_models = PathJoinSubstitution([package_share, "gazebo_models"])
    world_file = PathJoinSubstitution([gazebo_models, "robocon_track.world"])
    robot_xacro = PathJoinSubstitution([package_share, "urdf", "tr.xacro"])
    default_params = PathJoinSubstitution(
        [package_share, "config", "nav2_params.yaml"]
    )
    default_map = PathJoinSubstitution(
        [package_share, "maps", "robocon2027_slam.yaml"]
    )

    world = LaunchConfiguration("world")
    xacro_file = LaunchConfiguration("robot_xacro")
    robot_description = Command([FindExecutable(name="xacro"), " ", xacro_file])
    start_nav2 = LaunchConfiguration("start_nav2")

    gazebo_env = {
        # The cloned repository's world resolves model://robocon_ground here.
        "GAZEBO_MODEL_PATH": gazebo_models,
    }

    gazebo_server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        additional_env=gazebo_env,
        output="screen",
    )

    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        additional_env=gazebo_env,
        output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                arguments=[
                    "-entity",
                    "tr",
                    "-topic",
                    "robot_description",
                    "-x",
                    "0.0",
                    "-y",
                    "0.0",
                    "-z",
                    "0.05",
                ],
                output="screen",
            )
        ],
    )

    common_nav2 = {"use_sim_time": True}

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("world", default_value=world_file),
            DeclareLaunchArgument("robot_xacro", default_value=robot_xacro),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument(
                "start_nav2",
                default_value="false",
                description="Start Nav2; requires a valid map file.",
            ),
            gazebo_server,
            gazebo_client,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description, "use_sim_time": True},
                    {"frame_prefix": "tr/"},
                ],
                output="screen",
            ),
            spawn_robot,
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"yaml_filename": LaunchConfiguration("map"), **common_nav2},
                ],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[LaunchConfiguration("params_file"), common_nav2],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[LaunchConfiguration("params_file"), common_nav2],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_controller",
                executable="controller_server",
                name="controller_server",
                parameters=[LaunchConfiguration("params_file"), common_nav2],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[LaunchConfiguration("params_file"), common_nav2],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[LaunchConfiguration("params_file"), common_nav2],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {
                        **common_nav2,
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "amcl",
                            "planner_server",
                            "controller_server",
                            "behavior_server",
                            "bt_navigator",
                        ],
                    },
                ],
                output="screen",
                condition=IfCondition(start_nav2),
            ),
        ]
    )
