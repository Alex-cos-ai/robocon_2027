"""Open the Robocon 2027 competition field in Gazebo Classic and spawn TurtleBot3 Waffle Pi."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("robocon2027_description")
    gazebo_models = PathJoinSubstitution([package_share, "gazebo_models"])
    default_world = PathJoinSubstitution([gazebo_models, "robocon_track.world"])
    world = LaunchConfiguration("world")
    gazebo_env = {"GAZEBO_MODEL_PATH": gazebo_models}

    tb3_model_sdf = PathJoinSubstitution([
        package_share, "gazebo_models", "turtlebot3_waffle_pi", "model.sdf"
    ])

    # ---------- TurtleBot3 Waffle Pi 的 URDF ----------
    tb3_description_share = FindPackageShare("turtlebot3_description")
    tb3_urdf = PathJoinSubstitution([
        tb3_description_share, "urdf", "turtlebot3_waffle_pi.urdf"
    ])

    # ---------- 【新增】把 URDF 里的 ${namespace} 替换掉 ----------
    robot_description = Command([
        "sed 's/\\${namespace}//g' ", tb3_urdf
    ])

    return LaunchDescription(
        [
            # ---------- 参数 ----------
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Gazebo world containing the Robocon 2027 field.",
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("x", default_value="5.0"),
            DeclareLaunchArgument("y", default_value="-5.0"),
            DeclareLaunchArgument("z", default_value="0.05"),
            DeclareLaunchArgument("yaw", default_value="3.1415926"),

            # ---------- 1. gzserver：加载 world + ROS 插件 ----------
            ExecuteProcess(
                cmd=[
                    "gzserver",
                    "--verbose",
                    world,
                    "-s", "libgazebo_ros_init.so",
                    "-s", "libgazebo_ros_factory.so",
                ],
                additional_env=gazebo_env,
                output="screen",
            ),

            # ---------- 2. gzclient（GUI） ----------
            ExecuteProcess(
                cmd=["gzclient"],
                additional_env=gazebo_env,
                condition=IfCondition(LaunchConfiguration("gui")),
                output="screen",
            ),

            # ---------- 3. robot_state_publisher：发布完整 TF 树 ----------
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                namespace="",   # 空命名空间
                parameters=[
                    {
                        "use_sim_time": True,
                        "robot_description": robot_description,   # 【改动】用替换后的 URDF
                    }
                ],
                output="screen",
            ),

            # ---------- 4. 延迟 8 秒后 spawn TurtleBot3 Waffle Pi ----------
            TimerAction(
                period=0.0,   # 【改动】0.0 → 8.0，等 Gazebo 加载完
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2", "run", "gazebo_ros", "spawn_entity.py",
                            "-entity", "turtlebot3_waffle_pi",
                            "-file", tb3_model_sdf,
                            "-x", LaunchConfiguration("x"),
                            "-y", LaunchConfiguration("y"),
                            "-z", LaunchConfiguration("z"),
                            "-Y", LaunchConfiguration("yaw"),
                        ],
                        additional_env=gazebo_env,
                        output="screen",
                    ),
                ],
            ),
        ]
    )
# ros2 launch robocon2027_description test.py
# ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
# ros2 run nav2_map_server map_saver_cli -f src/robocon2027_description/room
# ros2 run teleop_twist_keyboard teleop_twist_keyboard 