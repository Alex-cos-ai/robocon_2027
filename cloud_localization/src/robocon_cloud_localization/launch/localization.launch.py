"""Start the Robocon field simulation in saved-map GICP localization mode."""
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
        DeclareLaunchArgument('relocalization_backend', default_value='bbs_gpu_gicp',
                              choices=['grid_gicp', 'bbs_cpu_gicp', 'bbs_gpu_gicp'],
                              description='Global search algorithm; GICP refinement is shared'),
        DeclareLaunchArgument('initial_x', default_value='-5.15', description='Waffle start x in the upper-left field start area'),
        DeclareLaunchArgument('initial_y', default_value='5.15', description='Waffle start y in the upper-left field start area'),
        # mid360_sim_1.pcd is a FAST-LIO map in its own local frame, anchored
        # near the mapping run's start pose, not in Gazebo world coordinates.
        DeclareLaunchArgument('map_file', default_value=str(share / 'maps' / 'mid360_sim_2.pcd'), description='PCD map used as the GICP target'),
        
        #icp initial pose -----------覆盖simulation x=-2.63 y = -2.63
        DeclareLaunchArgument('initial_pose_x', default_value='0.0', description='Initial lidar x estimate in the FAST-LIO map frame'),
        DeclareLaunchArgument('initial_pose_y', default_value='0.0', description='Initial lidar y estimate in the FAST-LIO map frame'),
        DeclareLaunchArgument('initial_pose_z', default_value='0.0', description='Initial lidar z in the PCD map, metres'),
        DeclareLaunchArgument('initial_pose_roll', default_value='0.0', description='Initial lidar roll in the PCD map, radians'),
        DeclareLaunchArgument('initial_pose_pitch', default_value='0.0', description='Initial lidar pitch in the PCD map, radians'),
        DeclareLaunchArgument('initial_pose_yaw', default_value='0.0', description='Initial map-frame yaw estimate'),
        
        DeclareLaunchArgument('world_to_map_x', default_value='-5.15', description='Map origin x in Gazebo world coordinates'),
        DeclareLaunchArgument('world_to_map_y', default_value='5.15', description='Map origin y in Gazebo world coordinates'),
        DeclareLaunchArgument('world_to_map_z', default_value='0.0', description='Calibrated world-to-map translation z'),
        DeclareLaunchArgument('world_to_map_roll', default_value='0.0', description='Calibrated world-to-map roll in radians'),
        DeclareLaunchArgument('world_to_map_pitch', default_value='0.0', description='Calibrated world-to-map pitch in radians'),
        DeclareLaunchArgument('world_to_map_yaw', default_value='0.0', description='Calibrated world-to-map yaw in radians'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(simulation)),
            launch_arguments={
                'mode': 'localize',
                'relocalization_backend': LaunchConfiguration('relocalization_backend'),
                'gui': LaunchConfiguration('gui'),
                'initial_x': LaunchConfiguration('initial_x'),
                'initial_y': LaunchConfiguration('initial_y'),
                'map_file': LaunchConfiguration('map_file'),
                'initial_pose_x': LaunchConfiguration('initial_pose_x'), #icp初始位置
                'initial_pose_y': LaunchConfiguration('initial_pose_y'),
                'initial_pose_z': LaunchConfiguration('initial_pose_z'),
                'initial_pose_roll': LaunchConfiguration('initial_pose_roll'),
                'initial_pose_pitch': LaunchConfiguration('initial_pose_pitch'),
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
