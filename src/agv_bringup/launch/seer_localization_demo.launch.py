"""Show synthetic Seer localization and continuous odometry without hardware access."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _setup(context):
    description = Path(get_package_share_directory('agv_description'))
    share = Path(get_package_share_directory('agv_bringup'))
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f'Map YAML does not exist: {map_path}')
    # Geometry only, with preview joints: no ros2_control or hardware plugins.
    root = ET.fromstring(xacro.process_file(str(description / 'urdf/agv.xacro')).toxml())
    root.set('name', 'synthetic_localization_demo')
    robot = ET.tostring(root, encoding='unicode')
    params = {'robot_description': ParameterValue(robot, value_type=str), 'use_sim_time': False}
    return [
        LogInfo(msg='SYNTHETIC DEMO: no controller connection; automatic correction every 20s'),
        Node(package='robot_move', executable='seer_localization_demo', output='screen'),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[params], namespace='localization_demo'),
        Node(package='joint_state_publisher', executable='joint_state_publisher',
             parameters=[params], namespace='localization_demo'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': str(map_path), 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='demo_map_lifecycle', parameters=[{
                 'use_sim_time': False, 'autostart': True, 'node_names': ['map_server']}]),
        Node(package='rviz2', executable='rviz2', name='synthetic_localization_rviz',
             arguments=['-d', str(share / 'config/seer_localization_demo.rviz')],
             parameters=[{'use_sim_time': False}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Converted Seer map YAML for visual reference'),
        DeclareLaunchArgument('domain_id', default_value='217'),
        DeclareLaunchArgument('rviz', default_value='true'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        OpaqueFunction(function=_setup),
    ])
