"""Start the Robocon field simulation in saved-map GICP localization mode."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('robocon_cloud_localization'))
    # This report is a frozen launch snapshot. Include its sibling simulation
    # launch instead of silently resolving the installed package's version.
    simulation = Path(__file__).resolve().parent / 'simulation.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo and RViz'),
        DeclareLaunchArgument('initial_x', default_value='-5.15', description='Waffle start x in the upper-left field start area'),
        DeclareLaunchArgument('initial_y', default_value='5.15', description='Waffle start y in the upper-left field start area'),
        DeclareLaunchArgument('map_file', default_value=str(share / 'maps' / 'laser_map.pcd'), description='PCD map used as the GICP target'),
        DeclareLaunchArgument('initial_pose_x', default_value='-5.15', description='Initial map-frame x estimate'),
        DeclareLaunchArgument('initial_pose_y', default_value='5.15', description='Initial map-frame y estimate'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0', description='Initial map-frame yaw estimate'),
        DeclareLaunchArgument('world_to_map_x', default_value='0.0', description='Calibrated world-to-map translation x'),
        DeclareLaunchArgument('world_to_map_y', default_value='0.0', description='Calibrated world-to-map translation y'),
        DeclareLaunchArgument('world_to_map_z', default_value='0.0', description='Calibrated world-to-map translation z'),
        DeclareLaunchArgument('world_to_map_roll', default_value='0.0', description='Calibrated world-to-map roll in radians'),
        DeclareLaunchArgument('world_to_map_pitch', default_value='0.0', description='Calibrated world-to-map pitch in radians'),
        DeclareLaunchArgument('world_to_map_yaw', default_value='3.1415926536', description='Calibrated world-to-map yaw in radians'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulation)),
            launch_arguments={
                'mode': 'localize',
                'gui': LaunchConfiguration('gui'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'map_file': LaunchConfiguration('map_file'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_yaw': LaunchConfiguration('initial_pose_yaw'),
                'world_to_map_x': LaunchConfiguration('world_to_map_x'),
                'world_to_map_y': LaunchConfiguration('world_to_map_y'),
                'world_to_map_z': LaunchConfiguration('world_to_map_z'),
                'world_to_map_roll': LaunchConfiguration('world_to_map_roll'),
                'world_to_map_pitch': LaunchConfiguration('world_to_map_pitch'),
                'world_to_map_yaw': LaunchConfiguration('world_to_map_yaw'),
            }.items(),
        ),
    ])
