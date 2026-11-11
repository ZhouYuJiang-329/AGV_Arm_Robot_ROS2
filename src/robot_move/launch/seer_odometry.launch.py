"""Independent read-only estimated odometry acquisition."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('connection'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('robot_move'), 'config', 'seer_odometry.yaml')),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument(
            'rviz', default_value='false', description='Open the odometry viewer'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='robot_move', executable='seer_odometry_node', output='screen',
             parameters=[LaunchConfiguration('params_file'), {
                 'connection': LaunchConfiguration('connection'),
                 'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                 'use_sim_time': ParameterValue(
                     LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='rviz2', executable='rviz2', output='screen',
             condition=IfCondition(LaunchConfiguration('rviz')),
             arguments=['-d', os.path.join(
                 get_package_share_directory('robot_move'), 'config', 'seer_odometry.rviz')],
             parameters=[{'use_sim_time': ParameterValue(
                 LaunchConfiguration('use_sim_time'), value_type=bool)}]),
    ])
