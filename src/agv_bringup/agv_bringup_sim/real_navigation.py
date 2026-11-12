"""Real navigation contract, shared by launch and offline tests."""
from pathlib import Path
import hashlib
import math
import yaml


REVIEW_KEYS = (
    'footprint_and_locked_arm_envelope_confirmed',
    'holonomic_axes_and_limits_confirmed', 'lidar_extrinsics_and_height_confirmed',
    'map_identity_alignment_and_time_confirmed',
    'controller_duration_and_stopping_confirmed', 'exclusive_control_confirmed',
)
# Absolute, private-to-navigation topics: never publish a raw chassis /cmd_vel.
EXECUTION_NODES = (
    ('nav2_controller', 'controller_server', [('cmd_vel', '/navigation/cmd_vel_raw')]),
    ('nav2_smoother', 'smoother_server', []),
    ('nav2_behaviors', 'behavior_server', [('cmd_vel', '/navigation/cmd_vel_raw')]),
    ('nav2_bt_navigator', 'bt_navigator', []),
    ('nav2_velocity_smoother', 'velocity_smoother', [
        ('cmd_vel', '/navigation/cmd_vel_raw'),
        ('cmd_vel_smoothed', '/navigation/cmd_vel_smoothed')]),
    ('nav2_collision_monitor', 'collision_monitor', []),
)


def load_yaml(path):
    with Path(path).expanduser().open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def parameters(path, node):
    return load_yaml(path)[node]['ros__parameters']


def validate_profile(path):
    data = load_yaml(path)
    if 'amcl' in data:
        raise ValueError('AMCL is incompatible with the Seer TF owner')
    def check_clock(value):
        if isinstance(value, dict):
            if 'use_sim_time' in value and value['use_sim_time'] is not False:
                raise ValueError('real navigation requires use_sim_time=false')
            for item in value.values():
                check_clock(item)
    check_clock(data)
    for name in ('controller_server', 'behavior_server', 'velocity_smoother', 'collision_monitor'):
        if data[name]['ros__parameters'].get('enable_stamped_cmd_vel') is not False:
            raise ValueError('all velocity endpoints must explicitly use Twist')
    cm = data['collision_monitor']['ros__parameters']
    if (cm['cmd_vel_in_topic'], cm['cmd_vel_out_topic']) != (
            '/navigation/cmd_vel_smoothed', '/navigation/cmd_vel_guard'):
        raise ValueError('collision monitor must terminate at the guard input')
    for name in ('local_costmap', 'global_costmap'):
        p = data[name][name]['ros__parameters']
        if p['robot_base_frame'] != 'base_link':
            raise ValueError('costmap must use base_link')
        layer = p['obstacle_layer']
        if layer['observation_sources'].split() != ['front', 'back']:
            raise ValueError('both scanners are required')
        for side in ('front', 'back'):
            src = layer[side]
            if src['topic'] != '/scan_' + side or src.get('inf_is_valid') is not False:
                raise ValueError('unknown scan returns must never clear obstacles')
    return data


def validate_execution(review_path, guard_path, map_path=None, profile_path=None):
    review = load_yaml(review_path)
    if any(review.get(key) is not True for key in REVIEW_KEYS):
        raise ValueError('execution blocked: incomplete site commissioning record')
    guard = parameters(guard_path, 'navigation_guard')
    for key in ('motion_enabled', 'input_freshness_contract_verified', 'base_transform_confirmed',
                'require_execution_heartbeat'):
        if guard.get(key) is not True:
            raise ValueError('execution blocked: guard ' + key)
    if set(guard['required_scan_topics']) != {'/scan_front', '/scan_back'}:
        raise ValueError('guard must supervise both scanners')
    if set(guard['required_tf_edges']) != {'map->odom', 'odom->base_link'}:
        raise ValueError('guard must supervise both dynamic TF edges')
    if set(guard.get('required_static_tf_edges', [])) != {
            'base_link->laser_front_link', 'base_link->laser_back_link'}:
        raise ValueError('guard must verify both static lidar TF owners')
    if guard.get('localization_diagnostic') != '/seer_localization/map_to_odom':
        raise ValueError('Seer localization quality supervision is mandatory')
    if map_path is not None:
        map_file = Path(map_path).expanduser().resolve()
        image = map_file.parent / load_yaml(map_file)['image']
        for key, file in (('ros_map_yaml_sha256', map_file), ('ros_map_image_sha256', image)):
            if review.get(key) != hashlib.sha256(file.read_bytes()).hexdigest():
                raise ValueError('execution blocked: map file does not match site record: ' + key)
        if not review.get('seer_map') or not review.get('seer_map_md5'):
            raise ValueError('execution blocked: exact Seer map identity is required')
    if profile_path is not None:
        profile = validate_profile(profile_path)
        v = profile['velocity_smoother']['ros__parameters']
        if (max(math.hypot(x, y) for x in (v['min_velocity'][0], v['max_velocity'][0])
                for y in (v['min_velocity'][1], v['max_velocity'][1])) > guard['max_linear_speed']
                or max(abs(v['min_velocity'][2]), abs(v['max_velocity'][2])) > guard['max_angular_speed']):
            raise ValueError('velocity smoother exceeds guard limits')
        if v['velocity_timeout'] >= guard['cmd_vel_timeout']:
            raise ValueError('smoother timeout must be shorter than guard command timeout')
    return guard
