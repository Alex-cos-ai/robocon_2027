import os
import re
from pathlib import Path
from xml.dom import Node as XmlNode

from ament_index_python.packages import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("robocon_cloud_localization"))
    fastlio_share = Path(get_package_share_directory("fast_lio"))
    root = os.environ.get("CLOUD_WS_ROOT")
    if root:
        root = Path(root).expanduser().resolve()
    else:
        root = next(
            (
                candidate
                for candidate in share.parents
                if (candidate / "vendor" / "install" / "lib").is_dir()
            ),
            Path.cwd(),
        )

    plugin_path = root / "install" / "robocon_cloud_localization" / "lib"
    model_path = share / "models"
    default_world_path = share / "worlds" / "robocon_track.world"
    gazebo_plugin_paths = [
        str(plugin_path),
        "/opt/ros/humble/lib",
        "/usr/lib/x86_64-linux-gnu/gazebo-11/plugins",
        *os.environ.get("GAZEBO_PLUGIN_PATH", "").split(os.pathsep),
    ]
    gazebo_plugin_path = os.pathsep.join(dict.fromkeys(path for path in gazebo_plugin_paths if path))
    world = LaunchConfiguration("world")
    fastlio_config_path = fastlio_share / "config" / "mid360_sim.yaml"
    localization_config_path = share / "config" / "localization.yaml"
    rviz_config_path = share / "config" / "localization.rviz"

    mode = LaunchConfiguration("mode")
    gui = LaunchConfiguration("gui")
    spawn_x = LaunchConfiguration("initial_x")
    spawn_y = LaunchConfiguration("initial_y")
    spawn_yaw = LaunchConfiguration("initial_yaw")
    map_file = LaunchConfiguration("map_file")
    guess_x = LaunchConfiguration("initial_pose_x")
    guess_y = LaunchConfiguration("initial_pose_y")
    guess_z = LaunchConfiguration("initial_pose_z")
    guess_roll = LaunchConfiguration("initial_pose_roll")
    guess_pitch = LaunchConfiguration("initial_pose_pitch")
    guess_yaw = LaunchConfiguration("initial_pose_yaw")
    world_to_map_x = LaunchConfiguration("world_to_map_x")
    world_to_map_y = LaunchConfiguration("world_to_map_y")
    world_to_map_z = LaunchConfiguration("world_to_map_z")
    world_to_map_roll = LaunchConfiguration("world_to_map_roll")
    world_to_map_pitch = LaunchConfiguration("world_to_map_pitch")
    world_to_map_yaw = LaunchConfiguration("world_to_map_yaw")

    description_doc = xacro.process_file(
        str(share / "urdf" / "robot_local.xacro"),
    )
    robot = description_doc.documentElement
    for element in list(robot.childNodes):
        if element.nodeType != XmlNode.ELEMENT_NODE:
            continue
        if (element.tagName == "joint" and element.getAttribute("name") == "scan_joint") or (
            element.tagName == "link" and element.getAttribute("name") == "base_scan"
        ):
            robot.removeChild(element)
    description_doc.normalize()
    # A compact comment-free XML string keeps libyaml from treating the URDF's
    # multiline comments as YAML mappings when robot_description is passed in.
    turtlebot_share = get_package_share_directory("turtlebot3_description")
    description_xml = description_doc.toxml().replace(
        "package://turtlebot3_description/",
        str(Path(turtlebot_share)) + os.sep,
    )
    description = "".join(
        re.sub(r"<!--.*?-->", "", description_xml, flags=re.S).splitlines()
    )

    env_model = SetEnvironmentVariable("GAZEBO_MODEL_PATH", str(model_path))
    env_plugins = SetEnvironmentVariable(
        "GAZEBO_PLUGIN_PATH",
        gazebo_plugin_path,
    )

    server = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world,
            "-s",
            "libgazebo_ros_init.so",
            "-s",
            "libgazebo_ros_factory.so",
        ],
        additional_env={
            "GAZEBO_MODEL_PATH": str(model_path),
            "GAZEBO_PLUGIN_PATH": gazebo_plugin_path,
        },
        output="screen",
    )
    client = ExecuteProcess(
        cmd=["gzclient"],
        additional_env={"GAZEBO_MODEL_PATH": str(model_path)},
        condition=IfCondition(gui),
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "use_sim_time": True,
                "robot_description": ParameterValue(description, value_type=str),
            }
        ],
        output="screen",
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "robocon_waffle_mid360",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            "0.62",# 0.62 L1
            "-Y",
            spawn_yaw,
        ],
        output="screen",
    )

    common_nodes = [
        Node(
            package="fast_lio",
            executable="fastlio_mapping",
            parameters=[
                str(fastlio_config_path),
                {
                    "use_sim_time": True,
                    # Localization consumes the configured PCD as its target.
                    # FAST-LIO's normal shutdown save uses the same configured
                    # path, so allow map saving only during mapping sessions.
                    "pcd_save": {
                        "pcd_save_en": ParameterValue(
                            PythonExpression(["'", mode, "' == 'map'"]),
                            value_type=bool,
                        ),
                    },
                },
            ],
            output="screen",
        ),
        Node(
            package="robocon_cloud_localization",
            executable="loop_closure_pgo",
            parameters=[
                str(localization_config_path),
                {
                    "mode": mode,
                    "localization_global_backend": LaunchConfiguration("relocalization_backend"),
                    "map_directory": str(root / "maps" / "fastlio_mid360_downward"),
                    "map_file": map_file,
                    "initial_pose_x": ParameterValue(guess_x, value_type=float),
                    "initial_pose_y": ParameterValue(guess_y, value_type=float),
                    "initial_pose_z": ParameterValue(guess_z, value_type=float),
                    "initial_pose_roll": ParameterValue(guess_roll, value_type=float),
                    "initial_pose_pitch": ParameterValue(guess_pitch, value_type=float),
                    "initial_pose_yaw": ParameterValue(guess_yaw, value_type=float),
                },
            ],
            output="screen",
        ),
    ]

    # Mapping owns the local map frame. Only localization needs the measured
    # Gazebo-world-to-PCD calibration; mapping's local frame has no such
    # calibration, so it must stay separate from Gazebo's world frame.
    world_to_map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_tf",
        arguments=[
            "--x", world_to_map_x, "--y", world_to_map_y, "--z", world_to_map_z,
            "--roll", world_to_map_roll, "--pitch", world_to_map_pitch,
            "--yaw", world_to_map_yaw, "--frame-id", "world", "--child-frame-id", "map",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'localize'"])),
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", str(rviz_config_path)],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(gui),
        output="screen",
    )

    return LaunchDescription(
        [
            # This launch can be invoked by source path, bypassing the
            # workspace environment hook. Keep Gazebo on the default DDS
            # discovery scope so it can communicate with teleop nodes started
            # from a normal ROS-sourced terminal.
            UnsetEnvironmentVariable(name="ROS_LOCALHOST_ONLY"),
            DeclareLaunchArgument("mode", default_value="map", description="map or localize"),
            DeclareLaunchArgument("relocalization_backend", default_value="bbs_gpu_gicp",
                                  choices=["grid_gicp", "bbs_cpu_gicp", "bbs_gpu_gicp"],
                                  description="Global recovery backend (shared GICP refinement)"),
            DeclareLaunchArgument(
                "world",
                default_value=str(default_world_path),
                description="Gazebo world; the default loads the local Robocon2027 field models",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Open Gazebo and RViz automatically (set false for headless mode)",
            ),
            DeclareLaunchArgument(
                "initial_x",
                default_value="0.0",# 在mapping.launch.py里面调
                description="Waffle base_footprint x at the upper-left field start area",
            ),
            DeclareLaunchArgument(
                "initial_y",
                default_value="0.0",# -5.15
                description="Waffle base_footprint y at the upper-left field start area",
            ),
            DeclareLaunchArgument(
                "initial_yaw",
                default_value="0.0",
                description="Waffle base_footprint heading in radians",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value=str(share / "maps" / "laser_map.pcd"),
                description="PCD target used by GICP when mode is localize",
            ),
            DeclareLaunchArgument(  #------------ICP relocalization
                "initial_pose_x",
                default_value="5.15",
                description="initial map-frame x guess for ICP relocalization",
            ),
            DeclareLaunchArgument(
                "initial_pose_y",
                default_value="-5.15",
                description="initial map-frame y guess for ICP relocalization",
            ),
            DeclareLaunchArgument(
                "initial_pose_z", default_value="0.0",
                description="initial lidar z in the PCD map, metres",
            ),
            DeclareLaunchArgument(
                "initial_pose_roll", default_value="0.0",
                description="initial lidar roll in the PCD map, radians",
            ),
            DeclareLaunchArgument(
                "initial_pose_pitch", default_value="0.0",
                description="initial lidar pitch in the PCD map, radians",
            ),
            DeclareLaunchArgument(
                "initial_pose_yaw",
                default_value="0.0",
                description="initial map-frame yaw guess for ICP relocalization",
            ),
            DeclareLaunchArgument(
                "world_to_map_x",
                default_value="0.0",
                description="calibrated translation x from Gazebo world to PCD map",
            ),
            DeclareLaunchArgument(
                "world_to_map_y",
                default_value="0.0",
                description="calibrated translation y from Gazebo world to PCD map",
            ),
            DeclareLaunchArgument(
                "world_to_map_z",
                default_value="0.0",
                description="calibrated translation z from Gazebo world to PCD map",
            ),
            DeclareLaunchArgument(
                "world_to_map_roll",
                default_value="0.0",
                description="calibrated roll from Gazebo world to PCD map, radians",
            ),
            DeclareLaunchArgument(
                "world_to_map_pitch",
                default_value="0.0",
                description="calibrated pitch from Gazebo world to PCD map, radians",
            ),
            DeclareLaunchArgument(
                "world_to_map_yaw",
                default_value="0.0",
                description="calibrated yaw from Gazebo world to PCD map, radians",
            ),
            env_model,
            env_plugins,
            server,
            client,
            # Gazebo owns /clock. Start the state publisher after the server
            # has emitted its first clock sample so fixed TF stamps do not
            # jump backwards during startup.
            TimerAction(period=1.0, actions=[robot_state_publisher]),
            TimerAction(period=4.0, actions=[spawn_robot]),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[
                        TimerAction(
                            period=2.0,
                            actions=[
                                *common_nodes,
                                world_to_map,
                                rviz,
                            ],
                        )
                    ],
                )
            ),
        ]
    )
