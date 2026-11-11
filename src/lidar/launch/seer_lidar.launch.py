"""Read-only Seer lidar acquisition; does not start a chassis driver or publish TF."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = str(Path(get_package_share_directory('lidar')) / 'config/seer_lidar.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('connection', description='Controller IPv4 address'),
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='lidar', executable='lidar_node', output='screen', parameters=[
            LaunchConfiguration('params_file'),
            {'connection': ParameterValue(LaunchConfiguration('connection'), value_type=str),
             'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
        ]),
    ])
