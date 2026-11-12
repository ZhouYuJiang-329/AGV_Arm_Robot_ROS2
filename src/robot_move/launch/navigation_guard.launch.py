"""Exclusive navigation chassis driver. No initialization actions or control takeover."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('connection'),
        DeclareLaunchArgument('params_file', default_value=(
            get_package_share_directory('robot_move') + '/config/navigation_guard.yaml')),
        Node(package='robot_move', executable='navigation_guard_node', output='screen',
             parameters=[LaunchConfiguration('params_file'),
                         {'connection': LaunchConfiguration('connection')}]),
    ])
