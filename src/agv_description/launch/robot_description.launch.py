"""Exclusive description/standalone/off TF ownership per lidar. No hardware nodes."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro
import yaml


HARDWARE_ARGS = (
    ('chassis_ip', '192.168.1.109'),
    ('agv_max_linear_speed', '0.0'), ('agv_max_angular_speed', '0.0'),
    ('agv_init_set_do', 'true'), ('agv_init_do_id', '0'),
    ('agv_init_do_status', 'true'), ('agv_init_relocate', 'true'),
)


def _setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    share = Path(get_package_share_directory('agv_description'))
    model = value('model')
    owners = {sensor: value(f'{sensor}_lidar_tf_owner') for sensor in ('front', 'back')}
    mounts = value('lidar_mounts_file')
    if model == 'front_only' and owners['front'] != 'standalone':
        raise RuntimeError('model=front_only requires front_lidar_tf_owner=standalone')
    if model == 'front_only' and owners['back'] == 'description':
        raise RuntimeError('back_lidar_tf_owner=description requires model=full')
    active = [sensor for sensor, owner in owners.items() if owner != 'off']
    if active:
        with open(mounts, encoding='utf-8') as stream:
            config = yaml.safe_load(stream)
        for sensor in active:
            mount = config.get(sensor, {})
            for key in ('x_m', 'y_m', 'z_relative_m', 'roll_deg', 'pitch_deg', 'yaw_deg'):
                number = mount.get(key)
                if isinstance(number, bool) or not isinstance(number, (int, float)) or not math.isfinite(number):
                    raise RuntimeError(f'{mounts}: {sensor}.{key} must be an explicit finite number; '
                                       'see docs/前雷达安装TF.md')
    use_sim_time = value('use_sim_time').lower() == 'true'
    mappings = {key: value(key) for key, _ in HARDWARE_ARGS}
    mappings['lidar_mounts_file'] = mounts
    actions = []
    for sensor in active:
        actions.append(LogInfo(msg=f'{sensor} lidar TF enabled with supplied external parameters; '
                               'installation calibration is NOT certified. '
                               f"Configured z_relative_m={config[sensor]['z_relative_m']} m. "
                               f'Run no other publisher for laser_{sensor}_link.'))
    # Each lidar joint belongs to exactly one of the two descriptions.
    if model == 'full':
        full_mappings = dict(mappings, **{
            f'enable_{sensor}_lidar_tf': str(owner == 'description').lower()
            for sensor, owner in owners.items()})
        description = xacro.process_file(str(share / 'urdf/agv_robot.urdf.xacro'),
                                         mappings=full_mappings).toxml()
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name='robot_state_publisher', output='screen',
                            parameters=[{'robot_description': description,
                                         'use_sim_time': use_sim_time}]))
    standalone = [sensor for sensor, owner in owners.items() if owner == 'standalone']
    if standalone:
        lidar_mappings = dict(mappings, **{
            f'enable_{sensor}_lidar_tf': str(owner == 'standalone').lower()
            for sensor, owner in owners.items()})
        description = xacro.process_file(str(share / 'urdf/lidar_only.urdf.xacro'),
                                         mappings=lidar_mappings).toxml()
        name = f'{standalone[0]}_lidar'
        actions.append(Node(package='robot_state_publisher', executable='robot_state_publisher',
                            name=f'{name}_state_publisher', output='screen',
                            remappings=[('robot_description', f'{name}_robot_description')],
                            parameters=[{'robot_description': description,
                                         'use_sim_time': use_sim_time}]))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('agv_description'))
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='front_only', choices=['full', 'front_only']),
        DeclareLaunchArgument('front_lidar_tf_owner', default_value='standalone',
                              choices=['off', 'description', 'standalone']),
        DeclareLaunchArgument('back_lidar_tf_owner', default_value='off',
                              choices=['off', 'description', 'standalone']),
        DeclareLaunchArgument('lidar_mounts_file',
                              default_value=str(share / 'config/lidar_mounts.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false', choices=['true', 'false']),
        *[DeclareLaunchArgument(key, default_value=default) for key, default in HARDWARE_ARGS],
        OpaqueFunction(function=_setup),
    ])
