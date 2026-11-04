"""Start the shared Gazebo AGV with SLAM and no pre-existing map or Nav2."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('agv_bringup'))
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('domain_id', default_value='187'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/agv_navigation_sim.launch.py')),
            launch_arguments={
                'mode': 'mapping', 'map': '',
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
                'domain_id': LaunchConfiguration('domain_id'),
            }.items()),
    ])
