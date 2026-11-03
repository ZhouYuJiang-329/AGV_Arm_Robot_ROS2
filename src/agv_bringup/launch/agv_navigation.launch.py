"""Start Nav2 with explicit robot parameters; this launch creates no chassis driver.

The default is simulation time and inactive lifecycle nodes. Supply an actual
map and a robot-specific Nav2 parameter file; no TurtleBot geometry is assumed.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_navigation(context):
    values = {name: LaunchConfiguration(name).perform(context)
              for name in ('map', 'params_file', 'use_sim_time', 'autostart', 'namespace')}
    for name in ('map', 'params_file'):
        path = Path(values[name]).expanduser().resolve()
        if not path.is_file():
            raise RuntimeError(f'{name} must name an existing YAML file: {path}')
        values[name] = str(path)
    for name in ('use_sim_time', 'autostart'):
        if values[name].lower() not in ('true', 'false'):
            raise RuntimeError(f'{name} must be true or false')
        values[name] = str(values[name].lower() == 'true')
    values.update({'slam': 'False', 'use_composition': 'False',
                   'use_namespace': 'True' if values['namespace'] else 'False'})
    source = Path(get_package_share_directory('nav2_bringup')) / 'launch/bringup_launch.py'
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(str(source)),
                                     launch_arguments=values.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Existing map YAML for the target environment'),
        DeclareLaunchArgument('params_file', description='Nav2 YAML matched to this robot'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='false',
                              description='Keep inactive until sensor, odometry and TF are verified'),
        DeclareLaunchArgument('namespace', default_value=''),
        OpaqueFunction(function=launch_navigation),
    ])
