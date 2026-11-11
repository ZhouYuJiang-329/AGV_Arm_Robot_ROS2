"""Show the CAD model driven by existing odom TF, with zero preview joints."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _setup(context):
    share = Path(get_package_share_directory('agv_description'))
    # Geometry only: deliberately omit ros2_control and all hardware plugins.
    root = ET.fromstring(xacro.process_file(str(share / 'urdf/agv.xacro')).toxml())
    root.set('name', 'agv_odometry_view')
    description = ET.tostring(root, encoding='unicode')
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    parameters = {'robot_description': description, 'use_sim_time': use_sim_time}
    return [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace='odometry_view', parameters=[parameters]),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             namespace='odometry_view', parameters=[parameters]),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', str(share / 'rviz/odometry_view.rviz')],
             parameters=[{'use_sim_time': use_sim_time}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        OpaqueFunction(function=_setup),
    ])
