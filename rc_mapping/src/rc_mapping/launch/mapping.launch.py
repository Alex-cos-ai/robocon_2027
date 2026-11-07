import os
import sysconfig

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory('rc_mapping')
    config_dir = os.path.join(package_dir, 'config')
    system_library_dir = os.path.join('/usr/lib', sysconfig.get_config_var('MULTIARCH'))
    pcl_environment = {
        'LD_LIBRARY_PATH': system_library_dir + os.pathsep + os.environ.get('LD_LIBRARY_PATH', ''),
    }
    with open(os.path.join(package_dir, 'urdf', 'mapping.urdf'), encoding='utf-8') as robot_file:
        robot_description = robot_file.read()

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_livox = LaunchConfiguration('use_livox')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'use_livox', default_value='true',
            description='Start the MID360 driver; disable when replaying a rosbag',
        ),
        DeclareLaunchArgument(
            'livox_config', default_value=os.path.join(config_dir, 'MID360_driver.json'),
        ),
        DeclareLaunchArgument(
            'fast_lio_config', default_value=os.path.join(config_dir, 'fast_lio.yaml'),
        ),
        DeclareLaunchArgument(
            'slam_config', default_value=os.path.join(config_dir, 'mapper_params_online_async.yaml'),
        ),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(package_dir, 'rviz', 'mapping.rviz'),
        ),
        DeclareLaunchArgument(
            'map_file_path', default_value=os.path.abspath('maps/scans.pcd'),
            description='PCD output path for the FAST-LIO /map_save service; parent directory must exist',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }],
        ),
        Node(
            package='livox_ros_driver2',
            executable='livox_ros_driver2_node',
            name='livox_lidar_publisher',
            output='screen',
            additional_env=pcl_environment,
            condition=IfCondition(use_livox),
            parameters=[{
                'use_sim_time': use_sim_time,
                'xfer_format': 1,
                'multi_topic': 0,
                'data_src': 0,
                'publish_freq': 20.0,
                'output_data_type': 0,
                'frame_id': 'livox_frame',
                'user_config_path': LaunchConfiguration('livox_config'),
            }],
        ),
        Node(
            package='fast_lio',
            executable='fastlio_mapping',
            name='laser_mapping',
            output='screen',
            additional_env=pcl_environment,
            parameters=[LaunchConfiguration('fast_lio_config'), {
                'use_sim_time': use_sim_time,
                'map_file_path': LaunchConfiguration('map_file_path'),
            }],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            output='screen',
            remappings=[('cloud_in', '/cloud_registered'), ('scan', '/scan')],
            parameters=[{
                'use_sim_time': use_sim_time,
                'target_frame': 'footprint_link',
                'transform_tolerance': 0.1,
                'min_height': -0.04,
                'max_height': 0.03,
                'angle_min': -3.14159,
                'angle_max': 3.14159,
                'angle_increment': 0.0043,
                'scan_time': 0.3333,
                'range_min': 0.45,
                'range_max': 10.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[LaunchConfiguration('slam_config'), {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(use_rviz),
        ),
    ])
