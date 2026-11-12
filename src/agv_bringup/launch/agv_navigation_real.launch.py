"""Default: map and planner only. Execute requires explicit site release and live gate."""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from agv_bringup_sim.real_navigation import EXECUTION_NODES, validate_execution, validate_profile


def setup(context):
    def value(key):
        return LaunchConfiguration(key).perform(context)
    def boolean(key):
        if value(key) not in ('true', 'false'):
            raise ValueError(key + ' must be true or false')
        return value(key) == 'true'
    def share(package):
        return Path(get_package_share_directory(package))
    mode = value('mode')
    if mode not in ('observe', 'execute'):
        raise ValueError('mode must be observe or execute')
    for key in ('map', 'params_file'):
        if not Path(value(key)).expanduser().is_file():
            raise ValueError(key + ' must be an existing file')
    validate_profile(value('params_file'))
    if mode == 'execute':
        validate_execution(value('commissioning_file'), value('guard_params'),
                           value('map'), value('params_file'))
        if not value('connection'):
            raise ValueError('execute requires connection')
    # Probe only the selected domain; never stop or modify existing nodes.
    import rclpy
    from rclpy.context import Context
    probe_context = Context()
    rclpy.init(args=[], context=probe_context)
    probe = rclpy.create_node('agv_navigation_launch_probe', context=probe_context)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=probe_context)
    executor.add_node(probe)
    import time
    deadline = time.monotonic() + 1.5
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.1)
    names = probe.get_node_names()
    executor.shutdown()
    probe.destroy_node()
    rclpy.shutdown(context=probe_context)
    new_names = ['planner_server', 'lifecycle_manager_real_planning']
    if mode == 'observe':
        new_names.append('agv_static_planning')
    if not boolean('reuse_map_server'):
        new_names.append('map_server')
    for reuse, name in (('reuse_lidar', 'lidar_node'), ('reuse_odometry', 'seer_odometry'),
                        ('reuse_localization', 'seer_localization')):
        if not boolean(reuse):
            new_names.append(name)
            if not value('connection'):
                raise ValueError('starting acquisition requires connection')
    if not boolean('reuse_description'):
        if any('state_publisher' in name for name in names):
            raise ValueError('description/TF publisher already exists; reuse it')
    if mode == 'execute':
        new_names += [item[1] for item in EXECUTION_NODES] + ['navigation_guard']
        if any(name in names for name in ('robot_move', 'robot_move_node', 'universal_robot_driver',
                                         'controller_manager', 'agv_hardware')):
            raise ValueError('competing chassis/control manager exists; isolate it before execute')
    if set(new_names).intersection(names):
        raise ValueError('refusing duplicate nodes: ' + str(set(new_names).intersection(names)))
    params = [value('params_file'), {'use_sim_time': False}]
    actions = []
    def include(package, launch, args):
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            str(share(package) / 'launch' / launch)), launch_arguments=args.items()))
    if not boolean('reuse_lidar'):
        include('lidar', 'seer_lidar.launch.py', {'connection': value('connection'),
                'params_file': value('lidar_params'), 'use_sim_time': 'false'})
    if not boolean('reuse_odometry'):
        include('robot_move', 'seer_odometry.launch.py', {'connection': value('connection'),
                'params_file': value('odom_params'), 'use_sim_time': 'false', 'rviz': 'false'})
    if not boolean('reuse_localization'):
        # Direct node: localization_view also starts a map server, which we own separately.
        actions.append(Node(package='robot_move', executable='seer_localization_node',
                            parameters=[value('localization_params'), {
                                'connection': value('connection'), 'use_sim_time': False}],
                            output='screen'))
    if not boolean('reuse_description'):
        include('agv_description', 'robot_description.launch.py', {
            'model': 'front_only', 'front_lidar_tf_owner': 'standalone',
            'back_lidar_tf_owner': 'standalone', 'use_sim_time': 'false',
            'lidar_mounts_file': value('lidar_mounts_file')})
    planning_names = ['planner_server']
    if not boolean('reuse_map_server'):
        actions.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
                            parameters=params + [{'yaml_filename': str(Path(value('map')).resolve())}],
                            output='screen'))
        planning_names.insert(0, 'map_server')
    actions += [Node(package='nav2_planner', executable='planner_server', name='planner_server',
                     parameters=params, output='screen'),
                Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                     name='lifecycle_manager_real_planning', parameters=[{
                         'use_sim_time': False, 'autostart': True, 'node_names': planning_names}])]
    if mode == 'observe':
        actions.append(Node(package='agv_bringup', executable='agv_static_planning',
                            parameters=[{'use_sim_time': False, 'use_robot_start': True}],
                            remappings=[('/goal_pose', '/planning_preview/goal'),
                                        ('/plan', '/planning_preview/path'),
                                        ('/planning_start', '/planning_preview/start_pose'),
                                        ('/planning_goal', '/planning_preview/goal_pose')],
                            output='screen'))
    if mode == 'execute':
        actions.append(Node(package='robot_move', executable='navigation_guard_node',
                            parameters=[value('guard_params'), {'use_sim_time': False,
                                        'connection': value('connection')}],
                            remappings=[('cmd_vel', '/navigation/cmd_vel_guard'), ('odom', '/odom')],
                            output='screen'))
        for package, name, remappings in EXECUTION_NODES:
            actions.append(Node(package=package, executable=name, name=name,
                                parameters=params, remappings=remappings, output='screen'))
        actions += [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                         name='lifecycle_manager_real_execution', parameters=[{
                             'use_sim_time': False, 'autostart': False,
                             'node_names': [item[1] for item in EXECUTION_NODES]}]),
                    Node(package='agv_bringup', executable='agv_navigation_execution_gate',
                         parameters=[{'use_sim_time': False, 'guard_params': value('guard_params'),
                                      'map_file': str(Path(value('map')).resolve()),
                                      'commissioning_file': value('commissioning_file')}], output='screen')]
    if boolean('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2',
                            arguments=['-d', str(share('agv_bringup') / 'config/agv_navigation_real.rviz')],
                            parameters=[{'use_sim_time': False}]))
    return actions


def generate_launch_description():
    def config(package, name):
        return str(Path(get_package_share_directory(package)) / 'config' / name)
    defaults = {
        'mode': 'observe', 'connection': '', 'rviz': 'true',
        'domain_id': os.environ.get('ROS_DOMAIN_ID', '216'),
        'params_file': config('agv_bringup', 'agv_nav2_real.yaml'),
        'commissioning_file': config('agv_bringup', 'agv_navigation_commissioning.yaml'),
        'guard_params': config('robot_move', 'navigation_guard.yaml'),
        'lidar_params': config('lidar', 'seer_lidar.yaml'),
        'odom_params': config('robot_move', 'seer_odometry_confirmed.yaml'),
        'localization_params': config('robot_move', 'seer_localization.yaml'),
        'lidar_mounts_file': config('agv_description', 'lidar_mounts.yaml'),
        **{key: 'true' for key in ('reuse_lidar', 'reuse_odometry', 'reuse_localization',
                                  'reuse_description')},
        'reuse_map_server': 'false',
    }
    return LaunchDescription([
        DeclareLaunchArgument('map', description='ROS map YAML matched to API1000 current_map/MD5'),
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        OpaqueFunction(function=setup),
    ])
