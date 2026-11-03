"""Preview this project's CAD geometry in Gazebo without loading real hardware.

All joints are fixed at zero for this first model check. Wheel kinematics and
navigation sensors will be added after the physical chassis type is confirmed.
"""

from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def build_preview_description():
    """Reuse geometry only, freeze joints, and resolve mesh files for Gazebo."""
    share = Path(get_package_share_directory('agv_description'))
    root = ET.fromstring(xacro.process_file(str(share / 'urdf/agv.xacro')).toxml())
    root.set('name', 'agv_preview')
    if root.findall('ros2_control'):
        raise RuntimeError('Geometry input unexpectedly contains a hardware definition')
    for joint in root.findall('joint'):
        # SDF uses a shared frame namespace; CAD joint/link names overlap.
        joint.set('name', 'preview_fixed_' + joint.attrib['name'])
        joint.set('type', 'fixed')
        for child in list(joint):
            if child.tag in ('axis', 'limit', 'dynamics', 'mimic', 'safety_controller'):
                joint.remove(child)
    for mesh in root.findall('.//mesh'):
        uri = mesh.get('filename', '')
        if uri.startswith('package://'):
            package, relative = uri[len('package://'):].split('/', 1)
            path = Path(get_package_share_directory(package)) / relative
            if not path.is_file():
                raise RuntimeError(f'Mesh does not exist: {path}')
            mesh.set('filename', path.as_uri())
    ET.SubElement(ET.SubElement(root, 'gazebo'), 'static').text = 'true'
    return ET.tostring(root, encoding='unicode')


def launch_preview(context):
    description = build_preview_description()
    gui = LaunchConfiguration('gui').perform(context).lower()
    if gui not in ('true', 'false'):
        raise RuntimeError('gui must be true or false')
    gz_args = '-r -v 2 empty.sdf' + (' -s' if gui == 'false' else '')
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': ParameterValue(description, value_type=str),
                          'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='ros_gz_sim', executable='create',
             arguments=['-topic', 'robot_description', '-name', 'agv_preview',
                        '-z', LaunchConfiguration('z')], output='screen'),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Show Gazebo GUI'),
        DeclareLaunchArgument('z', default_value='1.0',
                              description='Preview height; ground contact is not calibrated yet'),
        OpaqueFunction(function=launch_preview),
    ])
