"""One-command ideal holonomic AGV navigation in Gazebo Harmonic (no real SDK)."""
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, SetEnvironmentVariable, RegisterEventHandler, EmitEvent
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from agv_bringup_sim.assets import generate_assets


def start_simulation(context):
    share = Path(get_package_share_directory('agv_bringup'))
    description = Path(get_package_share_directory('agv_description'))
    gui = LaunchConfiguration('gui').perform(context).lower()
    autostart = LaunchConfiguration('autostart').perform(context).lower()
    if gui not in ('true', 'false') or autostart not in ('true', 'false'):
        raise RuntimeError('gui and autostart must be true or false')
    assets = generate_assets(share/'config/agv_navigation_scene.yaml',
                             description/'meshes/base_link.STL', share/'config/agv_nav2_sim.yaml',
                             tempfile.mkdtemp(prefix='agv-navigation-sim-'))
    start = assets['scene']['start']
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    actions = [
        LogInfo(msg='AGV simulator assets: ' + assets['directory']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share/'launch/gz_sim.launch.py')),
            launch_arguments={'gz_args': f"-r -v 2 {assets['world']}" + (' -s' if gui == 'false' else ''),
                              'on_exit_shutdown': 'true'}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             parameters=[{'robot_description': ParameterValue(assets['description'], value_type=str), 'use_sim_time': True}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/sim/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                        '/sim/odom_tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/sim/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                        '/model/agv_sim/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist'],
             remappings=[('/sim/odom', '/odom'), ('/sim/odom_tf', '/tf'),
                         ('/sim/scan', '/sim/scan_raw'), ('/model/agv_sim/cmd_vel', '/sim/cmd_vel')]),
        Node(package='agv_bringup', executable='agv_simulation_adapter', output='screen',
             parameters=[{'use_sim_time': True}]),
    ]
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
             arguments=['-file', assets['robot'], '-name', 'agv_sim', '-world', assets['scene']['world_name'],
                        '-x', str(start[0]), '-y', str(start[1]), '-Y', str(start[2])])
    navigation_actions = []
    params = [assets['params'], {'use_sim_time': True, 'enable_stamped_cmd_vel': False}]
    node_specs = [
        ('nav2_map_server', 'map_server', 'map_server', []),
        ('nav2_amcl', 'amcl', 'amcl', []),
        ('nav2_controller', 'controller_server', 'controller_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_planner', 'planner_server', 'planner_server', []),
        ('nav2_smoother', 'smoother_server', 'smoother_server', []),
        ('nav2_behaviors', 'behavior_server', 'behavior_server', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_bt_navigator', 'bt_navigator', 'bt_navigator', []),
        ('nav2_velocity_smoother', 'velocity_smoother', 'velocity_smoother', [('cmd_vel', 'cmd_vel_nav')]),
        ('nav2_collision_monitor', 'collision_monitor', 'collision_monitor', []),
    ]
    for package, executable, name, remappings in node_specs:
        node_params = params + ([{'yaml_filename': assets['map']}] if name == 'map_server' else [])
        navigation_actions.append(Node(package=package, executable=executable, name=name, output='screen',
                            parameters=node_params, remappings=remappings))
    for suffix, names in [('localization', ['map_server', 'amcl']),
                          ('navigation', [spec[2] for spec in node_specs[2:]])]:
        navigation_actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                            name='lifecycle_manager_'+suffix, output='screen',
                            parameters=[{'use_sim_time': True, 'autostart': autostart == 'true',
                                         'node_names': names, 'bond_timeout': 10.0}]))
    navigation_actions.append(Node(package='rviz2', executable='rviz2', output='screen',
                        arguments=['-d', str(share/'config/agv_navigation_sim.rviz')],
                        parameters=[{'use_sim_time': True}], condition=IfCondition(LaunchConfiguration('rviz'))))
    ready = Node(package='agv_bringup', executable='agv_simulation_ready', output='screen')
    def after_ready(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Simulator feedback failed to become ready'))]
        return navigation_actions
    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='AGV model failed to spawn'))]
        return [RegisterEventHandler(OnProcessExit(target_action=ready, on_exit=after_ready)), ready]
    actions.extend([RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)), spawn])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('domain_id', default_value='187', description='Isolated ROS domain for simulation'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST'),
        SetEnvironmentVariable('GZ_PARTITION', 'agv_navigation_sim'),
        OpaqueFunction(function=start_simulation),
    ])
