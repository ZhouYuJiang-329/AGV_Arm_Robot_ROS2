"""Read-only map localization display; odometry and lidar TF start separately."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_move'))
    return LaunchDescription([
        DeclareLaunchArgument('connection'),
        DeclareLaunchArgument('map', description='ROS map YAML matching expected_map'),
        DeclareLaunchArgument('expected_map', description='Exact API1000 current_map'),
        DeclareLaunchArgument('params_file', default_value=str(
            share / 'config/seer_localization.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        Node(package='robot_move', executable='seer_localization_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'connection': ParameterValue(LaunchConfiguration('connection'), value_type=str),
                 'expected_map': ParameterValue(
                     LaunchConfiguration('expected_map'), value_type=str),
                 'use_sim_time': False}]),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{
                 'use_sim_time': False,
                 'yaml_filename': LaunchConfiguration('map'),
                 'frame_id': 'map',
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='localization_display_lifecycle', output='screen',
             parameters=[{'use_sim_time': False, 'autostart': True,
                          'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', str(share / 'config/seer_localization.rviz')],
             parameters=[{'use_sim_time': False}]),
    ])
