"""Launch the Robocon field simulation and the 2-D Nav2 stack.

The robot model publishes ``/scan`` from a planar Gazebo ray sensor and
``/odom`` from the differential-drive plugin.  AMCL is used by default for
map-based localization; pass ``slam:=true`` to run slam_toolbox instead when a
map still needs to be built.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    package_name = "rc_navigation"
    share = Path(get_package_share_directory(package_name))
    nav2_bringup_share = Path(get_package_share_directory("nav2_bringup"))

    default_world = share / "worlds" / "robocon_track.world"
    default_robot_xacro = share / "urdf" / "robot_2d.xacro"
    default_params = share / "config" / "nav2_params.yaml"
    default_map = share / "maps" / "robocon_track.yaml"
    default_rviz = share / "rviz" / "nav2.rviz"
    model_path = share / "models"

    gui = LaunchConfiguration("gui")
    world = LaunchConfiguration("world")
    robot_xacro = LaunchConfiguration("robot_xacro")
    params_file = LaunchConfiguration("params_file")
    map_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam = LaunchConfiguration("slam")
    start_nav2 = LaunchConfiguration("start_nav2")
    start_rviz = LaunchConfiguration("start_rviz")
    initial_pose_x = LaunchConfiguration("initial_pose_x")
    initial_pose_y = LaunchConfiguration("initial_pose_y")
    initial_pose_yaw = LaunchConfiguration("initial_pose_yaw")

    # Rewriting these three values keeps the launch defaults aligned with the
    # Gazebo spawn pose while still allowing an experiment to override them.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            "amcl.ros__parameters.initial_pose.x": initial_pose_x,
            "amcl.ros__parameters.initial_pose.y": initial_pose_y,
            "amcl.ros__parameters.initial_pose.yaw": initial_pose_yaw,
        },
        convert_types=True,
    )

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", robot_xacro]),
        value_type=str,
    )

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
        additional_env={"GAZEBO_MODEL_PATH": str(model_path)},
        output="screen",
    )
    gazebo_client = ExecuteProcess(
        cmd=["gzclient"],
        additional_env={"GAZEBO_MODEL_PATH": str(model_path)},
        condition=IfCondition(gui),
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
        output="screen",
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robocon_waffle",
        arguments=[
            "-entity",
            "robocon_waffle_2d",
            "-topic",
            "robot_description",
            "-x",
            initial_pose_x,
            "-y",
            initial_pose_y,
            "-z",
            "0.05",
            "-Y",
            initial_pose_yaw,
        ],
        output="screen",
    )

    normalized_slam = PythonExpression([
        "'True' if '", slam, "'.lower() == 'true' else 'False'"
    ])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(nav2_bringup_share / "launch" / "bringup_launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "use_namespace": "false",
            "slam": normalized_slam,
            "map": map_file,
            "use_sim_time": use_sim_time,
            "params_file": configured_params,
            "autostart": "true",
            "use_composition": "False",
            "use_respawn": "false",
            "log_level": "info",
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(default_rviz)],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(start_rviz),
    )

    # Starting Nav2 after spawn avoids lifecycle activation before the robot's
    # TF tree and LaserScan publisher exist.  The stack remains independently
    # switchable for headless controller or Gazebo-only tests.
    post_spawn = GroupAction(
        condition=IfCondition(start_nav2),
        actions=[
            TimerAction(period=3.0, actions=[nav2, rviz]),
        ],
    )

    return LaunchDescription(
        [
            UnsetEnvironmentVariable(name="ROS_LOCALHOST_ONLY"),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the Gazebo client (false for headless runs).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=str(default_world),
                description="Gazebo world to load.",
            ),
            DeclareLaunchArgument(
                "robot_xacro",
                default_value=str(default_robot_xacro),
                description="2-D differential-drive robot xacro.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(default_params),
                description="Nav2 and AMCL parameter file.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=str(default_map),
                description="Occupancy-grid YAML for AMCL/global planning.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use Gazebo clock.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Use slam_toolbox instead of AMCL/map_server.",
            ),
            DeclareLaunchArgument(
                "start_nav2",
                default_value="true",
                description="Start localization and the Nav2 navigation servers.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the package navigation layout.",
            ),
            DeclareLaunchArgument(
                "initial_pose_x",
                default_value="-5.15",
                description="Gazebo spawn x and default AMCL initial-pose x.",
            ),
            DeclareLaunchArgument(
                "initial_pose_y",
                default_value="5.15",
                description="Gazebo spawn y and default AMCL initial-pose y.",
            ),
            DeclareLaunchArgument(
                "initial_pose_yaw",
                default_value="0.0",
                description="Gazebo spawn heading and default AMCL initial yaw.",
            ),
            SetEnvironmentVariable(
                name="GAZEBO_MODEL_PATH", value=str(model_path)
            ),
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn_robot]),
            RegisterEventHandler(
                OnProcessExit(target_action=spawn_robot, on_exit=[post_spawn])
            ),
        ]
    )
