"""Start the Robocon field simulation in FAST-LIO2 mapping mode."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('robocon_cloud_localization'))
    simulation = share / 'launch' / 'simulation.launch.py'
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo and RViz'),
        DeclareLaunchArgument('save_fastlio_pcd', default_value='true', choices=['true', 'false'],
                              description='Save raw frontend PCD independently of optimized /save_map'),
        DeclareLaunchArgument(
            'world',
            default_value=str(share / 'worlds' / 'robocon_track.world'),
            description='Gazebo world to map',
        ),
        DeclareLaunchArgument('initial_x', default_value='-2.5', description='Robot start x in field coordinates'),
        DeclareLaunchArgument('initial_y', default_value='2.5', description='Robot start y in field coordinates'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0', description='Robot start heading in radians'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulation)),
            launch_arguments={
                'mode': 'map',
                'gui': LaunchConfiguration('gui'),
                'save_fastlio_pcd': LaunchConfiguration('save_fastlio_pcd'),
                'world': LaunchConfiguration('world'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'initial_yaw': LaunchConfiguration('initial_yaw'),
            }.items(),
        ),
    ])
