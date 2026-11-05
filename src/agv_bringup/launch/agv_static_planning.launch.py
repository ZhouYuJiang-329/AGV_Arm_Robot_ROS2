"""Map + explicit start/goal -> A* path in RViz, without motion or sensors."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('agv_bringup'))
    map_file = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_file.is_file():
        raise RuntimeError(f'Map YAML does not exist: {map_file}')
    params = str(share/'config/agv_static_planning.yaml')
    return [
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[params, {'yaml_filename': str(map_file)}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',
             parameters=[params], remappings=[('plan', 'planner_raw_plan')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_planning',
             parameters=[{'autostart': True, 'use_sim_time': False, 'node_names': ['map_server', 'planner_server']}]),
        Node(package='agv_bringup', executable='agv_static_planning', output='screen'),
        Node(package='rviz2', executable='rviz2', arguments=['-d', str(share/'config/agv_static_planning.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Required path to map YAML'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('domain_id', default_value='188'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        OpaqueFunction(function=setup),
    ])
