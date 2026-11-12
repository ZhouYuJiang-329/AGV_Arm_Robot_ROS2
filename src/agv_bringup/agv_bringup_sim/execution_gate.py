"""Activate execution only after live guard, TF, parameters and graph agree.

This monitor sends lifecycle requests only; it never publishes velocity or goals.
Loss after activation pauses execution and latches until operator restart.
The independent C++ guard remains responsible for bounded stopping.
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from std_msgs.msg import Header

from agv_bringup_sim.real_navigation import parameters, load_yaml


class ExecutionGate(Node):
    def __init__(self, **kwargs):
        super().__init__('agv_navigation_execution_gate', **kwargs)
        self.guard_config = parameters(self.declare_parameter('guard_params', '').value,
                                       'navigation_guard')
        self.map_file = self.declare_parameter('map_file', '').value
        self.review = load_yaml(self.declare_parameter('commissioning_file', '').value)
        self.heartbeat = self.create_publisher(Header, '/navigation/execution_heartbeat', 1)
        self.state = None
        self.guard_seen = 0.0
        self.edges = {}
        self.scan_seen = {}
        self.parameter_ok = False
        self.parameter_clients = {}
        self.pending = {}
        self.results = {}
        transform = ['base_transform_confirmed', 'base_x_in_seer',
                     'base_y_in_seer', 'base_yaw_in_seer']
        self.queries = {
            'navigation_guard': ['use_sim_time', *self.guard_config.keys()],
            'seer_odometry': ['use_sim_time', 'publish_tf', *transform],
            'seer_localization': ['use_sim_time', 'base_transform_confirmed',
                                  'map_alignment_confirmed', 'expected_map', 'expected_map_md5', 'tf_publish_mode',
                                  'base_x_in_seer_m', 'base_y_in_seer_m', 'base_yaw_in_seer_rad'],
            'map_server': ['use_sim_time', 'yaml_filename'],
        }
        # dict duplicates (e.g. use_sim_time) are harmless but avoid redundant RPC values.
        self.queries = {name: list(dict.fromkeys(keys)) for name, keys in self.queries.items()}
        for name in self.queries:
            self.parameter_clients[name] = self.create_client(GetParameters, '/' + name + '/get_parameters')
        self.subscriptions_ = [self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics, 10)]
        for topic in ('/tf', '/tf_static'):
            qos = (QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == '/tf_static' else 100)
            self.subscriptions_.append(self.create_subscription(TFMessage, topic, self.tf, qos))
        for side in ('front', 'back'):
            self.subscriptions_.append(self.create_subscription(
                LaserScan, '/scan_' + side,
                lambda msg, side=side: self.scan(msg, side), qos_profile_sensor_data))
        self.lifecycle = self.create_client(
            ManageLifecycleNodes, '/lifecycle_manager_real_execution/manage_nodes')
        self.transition = None
        self.active = False
        self.configured = False
        self.transition_kind = None
        self.latched = False
        self.healthy_since = None
        self.last_query = 0.0

    def diagnostics(self, msg):
        for status in msg.status:
            if status.name != '/navigation_guard':
                continue
            fields = {item.key: item.value for item in status.values}
            age = self.get_clock().now().nanoseconds * 1e-9 - (
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
            valid = (0 <= age < .5 and fields.get('motion_enabled') == 'true'
                     and fields.get('input_freshness_contract_verified') == 'true'
                     and fields.get('controller_connected') == 'true'
                     and fields.get('tf_ownership_valid') == 'true'
                     and all(value == 'healthy' for key, value in fields.items()
                             if key.startswith('input:') and key != 'input:execution_heartbeat'))
            self.state = fields.get('state') if valid else None
            self.guard_seen = time.monotonic() if valid else 0.0

    def tf(self, msg, info):
        # Jazzy Python MessageInfo has no publisher GID. C++ guard checks owners.
        gid = True
        for transform in msg.transforms:
            edge = (transform.header.frame_id, transform.child_frame_id)
            self.edges.setdefault(edge, set()).add(gid)
            if edge[0] == 'base_link' and edge[1] in ('laser_front_link', 'laser_back_link'):
                # Current profile uses horizontal laser planes in base/map/odom coordinates.
                p, q = transform.transform.translation, transform.transform.rotation
                if not (-1 < p.z < 1 and abs(q.x) < 1e-5 and abs(q.y) < 1e-5):
                    self.latched = True

    def scan(self, msg, side):
        age = self.get_clock().now().nanoseconds * 1e-9 - (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if (msg.header.frame_id == 'laser_' + side + '_link' and 0 <= age < .6
                and msg.range_max > msg.range_min >= 0 and any(
                    math.isfinite(x) and msg.range_min <= x <= msg.range_max for x in msg.ranges)):
            self.scan_seen[side] = time.monotonic() + .6 - age
        else:
            self.scan_seen[side] = 0.0

    def graph_ok(self, configured=True):
        names = self.get_node_names()
        for name in ('navigation_guard', 'controller_server', 'velocity_smoother',
                     'collision_monitor', 'behavior_server', 'bt_navigator', 'smoother_server',
                     'seer_odometry', 'seer_localization', 'map_server'):
            if names.count(name) != 1:
                return False
        if any(name in names for name in ('robot_move_node', 'universal_robot_driver',
                                         'controller_manager', 'agv_hardware')):
            return False
        for topic in ('/cmd_vel', '/legacy_cmd_vel'):
            if self.get_subscriptions_info_by_topic(topic):
                return False
        expected = {
            '/navigation/cmd_vel_raw': ({'controller_server', 'behavior_server'}, {'velocity_smoother'}),
            '/navigation/cmd_vel_smoothed': ({'velocity_smoother'}, {'collision_monitor'}),
            '/navigation/cmd_vel_guard': ({'collision_monitor'}, {'navigation_guard'}),
        }
        for topic, (publishers, subscribers) in expected.items():
            for infos, allowed in ((self.get_publishers_info_by_topic(topic), publishers),
                                   (self.get_subscriptions_info_by_topic(topic), subscribers)):
                observed = {i.node_name for i in infos}
                if not observed.issubset(allowed):
                    return False
                if configured and observed != allowed:
                    return False
                if any(i.topic_type != 'geometry_msgs/msg/Twist' for i in infos):
                    return False
        for edge in (('map', 'odom'), ('odom', 'base_link'),
                     ('base_link', 'laser_front_link'), ('base_link', 'laser_back_link')):
            if len(self.edges.get(edge, ())) != 1:
                return False
        # A required child must have only one parent in the observed graph.
        for child in ('odom', 'base_link', 'laser_front_link', 'laser_back_link'):
            if sum(edge[1] == child for edge in self.edges) != 1:
                return False
        return True

    @staticmethod
    def decode(value):
        attrs = {1: 'bool_value', 2: 'integer_value', 3: 'double_value', 4: 'string_value',
                 9: 'string_array_value'}
        result = getattr(value, attrs[value.type]) if value.type in attrs else None
        return list(result) if value.type == 9 else result

    def poll_parameters(self, now):
        for name, future in list(self.pending.items()):
            if future.done():
                try:
                    self.results[name] = dict(zip(self.queries[name],
                                                  map(self.decode, future.result().values)))
                except Exception:
                    self.results.pop(name, None)
                del self.pending[name]
        if now - self.last_query < 1.0:
            return
        # Clear previous results: disconnected parameter servers cannot remain accepted.
        self.parameter_ok = self.parameters_ok() if not self.pending else False
        self.results.clear()
        self.last_query = now
        for name, client in self.parameter_clients.items():
            if name not in self.pending and client.service_is_ready():
                req = GetParameters.Request(names=self.queries[name])
                self.pending[name] = client.call_async(req)

    def parameters_ok(self):
        if set(self.results) != set(self.queries):
            return False
        g, o, loc, m = [self.results[name] for name in self.queries]
        if any(p.get('use_sim_time') is not False for p in self.results.values()):
            return False
        if any(g.get(k) != v for k, v in self.guard_config.items()):
            return False
        if (o.get('base_transform_confirmed') is not True or o.get('publish_tf') is not True
                or loc.get('base_transform_confirmed') is not True
                or loc.get('map_alignment_confirmed') is not True
                or loc.get('tf_publish_mode') != 'bounded_odom_hold'
                or loc.get('expected_map') != self.review.get('seer_map')
                or loc.get('expected_map_md5') != self.review.get('seer_map_md5')
                or m.get('yaml_filename') != self.map_file):
            return False
        for key, loc_key in (('base_x_in_seer', 'base_x_in_seer_m'),
                             ('base_y_in_seer', 'base_y_in_seer_m'),
                             ('base_yaw_in_seer', 'base_yaw_in_seer_rad')):
            if g.get(key) != o.get(key) or g.get(key) != loc.get(loc_key):
                return False
        return True

    def tick(self):
        now = time.monotonic()
        self.poll_parameters(now)
        if self.transition and self.transition.done():
            try:
                if not self.transition.result().success:
                    self.latched = True
                elif self.transition_kind == 'configure':
                    self.configured = True
            except Exception:
                self.latched = True
            self.transition = None
        contracts = (self.parameter_ok and self.state in ('WaitingForInputs', 'Ready', 'Active')
                   and now - self.guard_seen < .5 and self.graph_ok(self.configured)
                   and all(self.scan_seen.get(side, 0) > now for side in ('front', 'back')))
        if contracts and not self.latched:
            self.heartbeat.publish(Header(stamp=self.get_clock().now().to_msg(),
                                          frame_id='navigation_execution_gate'))
        healthy = contracts and self.state in ('Ready', 'Active')
        if (healthy and not self.configured and not self.latched and self.transition is None
                and self.lifecycle.service_is_ready()):
            self.transition_kind = 'configure'
            self.transition = self.lifecycle.call_async(ManageLifecycleNodes.Request(
                command=ManageLifecycleNodes.Request.CONFIGURE))
        if self.active and (not healthy or self.latched):
            self.latched = True
            # Withheld heartbeat independently stops the C++ guard if lifecycle RPC fails.
            if self.transition is None and self.lifecycle.service_is_ready():
                self.transition = self.lifecycle.call_async(ManageLifecycleNodes.Request(
                    command=ManageLifecycleNodes.Request.PAUSE))
                self.get_logger().error('Execution paused; resolve fault and restart explicitly')
                self.active = False
        if not healthy:
            self.healthy_since = None
        elif self.healthy_since is None:
            self.healthy_since = now
        elif (self.configured and not self.active and not self.latched and self.transition is None
              and now - self.healthy_since >= 2 and self.lifecycle.service_is_ready()):
            self.transition_kind = 'activate'
            self.transition = self.lifecycle.call_async(ManageLifecycleNodes.Request(
                command=ManageLifecycleNodes.Request.RESUME))
            self.active = True
            self.get_logger().info('Live contracts passed; requesting execution lifecycle startup')


def main():
    rclpy.init()
    node = ExecutionGate()
    node.create_timer(.1, node.tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
