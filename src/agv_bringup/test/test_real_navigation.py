"""Offline ROS integration: real Jazzy plugins and planning, no chassis connection."""
import importlib.util
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock

import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path as RosPath
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from nav2_msgs.action import ComputePathToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from tf2_msgs.msg import TFMessage
import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))
from agv_bringup_sim.real_navigation import (
    EXECUTION_NODES, validate_execution, validate_profile,
)
from agv_bringup_sim.execution_gate import ExecutionGate


class ContractTest(unittest.TestCase):
    def test_profile_and_fail_closed_release(self):
        data = validate_profile(PACKAGE / 'config/agv_nav2_real.yaml')
        self.assertNotIn('amcl', data)
        with self.assertRaisesRegex(ValueError, 'incomplete site'):
            validate_execution(PACKAGE / 'config/agv_navigation_commissioning.yaml',
                               PACKAGE.parent / 'robot_move/config/navigation_guard.yaml')
        with tempfile.TemporaryDirectory() as tmp:
            review = yaml.safe_load((PACKAGE / 'config/agv_navigation_commissioning.yaml').read_text())
            review = {key: True for key in review}
            path = Path(tmp) / 'review.yaml'
            path.write_text(yaml.safe_dump(review))
            with self.assertRaisesRegex(ValueError, 'motion_enabled'):
                validate_execution(path, PACKAGE.parent / 'robot_move/config/navigation_guard.yaml')

    def test_launch_loads_and_execute_rejected_before_driver(self):
        spec = importlib.util.spec_from_file_location(
            'real_launch', PACKAGE / 'launch/agv_navigation_real.launch.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertTrue(module.generate_launch_description().entities)
        result = subprocess.run(['ros2', 'launch', 'agv_bringup', 'agv_navigation_real.launch.py',
                                 'map:=' + str(PACKAGE.parents[1] / 'maps/seer/20251120141209997.yaml'),
                                 'mode:=execute', 'rviz:=false'], capture_output=True, text=True,
                                timeout=10)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn('incomplete site', result.stdout + result.stderr)
        self.assertNotIn('process started', result.stdout + result.stderr)


class PluginIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('real_navigation_offline_test')
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.node)
        cls.extra_nodes = []
        cls.temp = tempfile.TemporaryDirectory(prefix='agv-nav2-test-')
        cls.tmp = Path(cls.temp.name)
        cls.processes = []
        cls.logs = []
        cls.tf = cls.node.create_publisher(TFMessage, '/tf', 10)
        cls.scan_fixture_enabled = True
        cls.scan_fixture = {side: cls.node.create_publisher(
            LaserScan, '/scan_' + side, qos_profile_sensor_data) for side in ('front', 'back')}
        def transforms():
            msg = TFMessage()
            for parent, child in [('map', 'odom'), ('odom', 'base_link'),
                                  ('base_link', 'laser_front_link'), ('base_link', 'laser_back_link')]:
                t = TransformStamped()
                t.header.stamp = cls.node.get_clock().now().to_msg()
                t.header.frame_id, t.child_frame_id = parent, child
                t.transform.rotation.w = 1.0
                if child.startswith('laser'):
                    t.transform.translation.z = -.24
                msg.transforms.append(t)
            cls.tf.publish(msg)
            if cls.scan_fixture_enabled:
                for side, publisher in cls.scan_fixture.items():
                    scan = LaserScan()
                    scan.header.stamp = cls.node.get_clock().now().to_msg()
                    scan.header.frame_id = 'laser_' + side + '_link'
                    scan.angle_min, scan.angle_max, scan.angle_increment = -.2, .2, .1
                    scan.range_min, scan.range_max = .05, 10.
                    scan.ranges = [4.] * 5
                    publisher.publish(scan)
        cls.node.create_timer(.05, transforms)
        (cls.tmp / 'map.pgm').write_bytes(b'P5\n200 200\n255\n' + bytes([254]) * 40000)
        cls.map = cls.tmp / 'map.yaml'
        cls.map.write_text(yaml.safe_dump(dict(image='map.pgm', resolution=.1, origin=[-10., -10., 0.],
                                             negate=0, occupied_thresh=.65, free_thresh=.196)))
        cls.start(['ros2', 'launch', 'agv_bringup', 'agv_navigation_real.launch.py',
                   'map:=' + str(cls.map), 'mode:=observe', 'rviz:=false',
                   'domain_id:=' + os.environ['ROS_DOMAIN_ID']], 'observe')

    @classmethod
    def start(cls, args, label):
        log = (cls.tmp / (label + '.log')).open('w+')
        cls.logs.append(log)
        cls.processes.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                                              start_new_session=True))

    @classmethod
    def spin_until(cls, predicate, timeout=10):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            cls.executor.spin_once(timeout_sec=.03)
            if predicate():
                return True
        return False

    @classmethod
    def tearDownClass(cls):
        for p in cls.processes:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGINT)
        for p in cls.processes:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait(timeout=3)
        for log in cls.logs:
            log.seek(0)
            text = log.read()
            if '[ERROR]' in text or 'terminate called' in text:
                print(text[-5000:])
            log.close()
        cls.executor.shutdown()
        for node in cls.extra_nodes:
            node.destroy_node()
        cls.node.destroy_node()
        rclpy.shutdown()
        cls.temp.cleanup()

    def test_01_observe_plans_without_velocity_endpoints(self):
        client = ActionClient(self.node, ComputePathToPose, '/compute_path_to_pose')
        self.assertTrue(self.spin_until(lambda: client.server_is_ready(), 15))
        state_client = self.node.create_client(GetState, '/planner_server/get_state')
        self.assertTrue(self.spin_until(state_client.service_is_ready))
        active = False
        for _ in range(50):
            state = state_client.call_async(GetState.Request())
            self.assertTrue(self.spin_until(state.done))
            if state.result().current_state.id == 3:
                active = True
                break
            self.spin_until(lambda: False, .1)
        self.assertTrue(active, 'planner must be active before requesting a path')
        names = self.node.get_node_names()
        for name in ('amcl', 'controller_server', 'navigation_guard', 'bt_navigator'):
            self.assertNotIn(name, names)
        self.assertFalse([t for t, _ in self.node.get_topic_names_and_types() if 'cmd_vel' in t])
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = 'map'
        goal.goal.pose.position.x = 1.0
        goal.goal.pose.orientation.w = 1.0
        goal.planner_id = 'GridBased'
        future = client.send_goal_async(goal)
        self.assertTrue(self.spin_until(future.done))
        handle = future.result()
        self.assertTrue(handle.accepted)
        result = handle.get_result_async()
        self.assertTrue(self.spin_until(result.done))
        self.assertGreater(len(result.result().result.path.poses), 2)
        client.destroy()
        # The RViz tool's private PoseStamped topic computes a visible, latched path.
        preview = []
        sub = self.node.create_subscription(
            RosPath, '/planning_preview/path', lambda msg: preview.append(msg),
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        select = self.node.create_publisher(PoseStamped, '/planning_preview/goal', 10)
        self.assertTrue(self.spin_until(lambda: select.get_subscription_count() == 1))
        selection = PoseStamped()
        selection.header.frame_id = 'map'
        selection.pose.position.x = 1.0
        selection.pose.orientation.w = 1.0
        select.publish(selection)
        self.assertTrue(self.spin_until(lambda: any(msg.poses for msg in preview)),
                        'RViz selection must produce a planning preview')
        self.assertFalse(self.node.get_subscriptions_info_by_topic('/initialpose'))
        self.assertFalse(self.node.get_subscriptions_info_by_topic('/goal_pose'))
        self.assertFalse([t for t, _ in self.node.get_topic_names_and_types() if 'cmd_vel' in t])
        self.node.destroy_subscription(sub)
        self.node.destroy_publisher(select)

    def test_02_all_execution_plugins_configure_and_twist_wiring(self):
        for package, name, remaps in EXECUTION_NODES:
            args = ['ros2', 'run', package, name, '--ros-args', '--params-file',
                    str(PACKAGE / 'config/agv_nav2_real.yaml')]
            for source, target in remaps:
                args += ['-r', source + ':=' + target]
            self.start(args, name)
        for _, name, _ in EXECUTION_NODES:
            client = self.node.create_client(ChangeState, '/' + name + '/change_state')
            self.assertTrue(self.spin_until(client.service_is_ready), name)
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CONFIGURE
            future = client.call_async(req)
            self.assertTrue(self.spin_until(future.done, 10), name)
            self.assertTrue(future.result().success, name)
        self.assertTrue(self.spin_until(lambda: bool(self.node.get_publishers_info_by_topic(
            '/navigation/cmd_vel_guard'))))
        for topic, expected in [('/navigation/cmd_vel_raw', {'controller_server', 'behavior_server'}),
                                ('/navigation/cmd_vel_smoothed', {'velocity_smoother'}),
                                ('/navigation/cmd_vel_guard', {'collision_monitor'})]:
            infos = self.node.get_publishers_info_by_topic(topic)
            self.assertEqual({i.node_name for i in infos}, expected)
            self.assertTrue(all(i.topic_type == 'geometry_msgs/msg/Twist' for i in infos))
        self.assertFalse(self.node.get_publishers_info_by_topic('/cmd_vel'))
        for _, name, _ in reversed(EXECUTION_NODES):
            client = self.node.create_client(ChangeState, '/' + name + '/change_state')
            req = ChangeState.Request()
            req.transition.id = Transition.TRANSITION_CLEANUP
            future = client.call_async(req)
            self.assertTrue(self.spin_until(future.done, 10), name)
            self.assertTrue(future.result().success, name)

    def test_03_gate_missing_stale_and_unverified_never_activates(self):
        gate = ExecutionGate.__new__(ExecutionGate)
        # Test the state transition policy with no ROS networking or control sender.
        gate.poll_parameters = Mock()
        gate.heartbeat = Mock()
        gate.get_clock = self.node.get_clock
        gate.transition = None
        gate.active = False
        gate.configured = True
        gate.transition_kind = None
        gate.latched = False
        gate.healthy_since = time.monotonic() - 10
        gate.lifecycle = Mock()
        gate.lifecycle.service_is_ready.return_value = True
        gate.graph_ok = Mock(return_value=True)
        gate.scan_seen = {'front': time.monotonic() + 10, 'back': time.monotonic() + 10}
        gate.parameter_ok = False
        gate.state = 'Ready'
        gate.guard_seen = time.monotonic()
        gate.tick()
        gate.lifecycle.call_async.assert_not_called()
        gate.parameter_ok = True
        for state, stamp in [(None, time.monotonic()), ('WaitingForInputs', time.monotonic()),
                             ('Ready', time.monotonic() - 2), ('Fault', time.monotonic())]:
            gate.state, gate.guard_seen = state, stamp
            gate.healthy_since = time.monotonic() - 10
            gate.tick()
            gate.lifecycle.call_async.assert_not_called()
        gate.state, gate.guard_seen = 'Ready', time.monotonic()
        gate.scan_seen['back'] = 0
        gate.tick()
        gate.lifecycle.call_async.assert_not_called()

    def test_04_live_gate_configure_activate_then_latch_on_scan_loss(self):
        # Simulated interfaces only: no real guard process or SDK connection.
        self.__class__.scan_fixture_enabled = False
        config = yaml.safe_load((PACKAGE.parent / 'robot_move/config/navigation_guard.yaml').read_text())
        guard_config = config['navigation_guard']['ros__parameters']
        for key in ('motion_enabled', 'base_transform_confirmed', 'input_freshness_contract_verified'):
            guard_config[key] = True
        guard_file = self.tmp / 'guard.yaml'
        guard_file.write_text(yaml.safe_dump(config))
        review = self.tmp / 'review.yaml'
        review.write_text(yaml.safe_dump({'seer_map': 'fixture', 'seer_map_md5': 'fixture-md5'}))
        def fake(name, values):
            node = Node(name, parameter_overrides=[Parameter(k, value=v) for k, v in values.items()],
                        automatically_declare_parameters_from_overrides=True)
            self.extra_nodes.append(node)
            self.executor.add_node(node)
            return node
        guard = fake('navigation_guard', guard_config)
        guard.create_subscription(Twist, '/navigation/cmd_vel_guard', lambda _: None, 1)
        tf_params = {key: guard_config[key] for key in (
            'base_transform_confirmed', 'base_x_in_seer', 'base_y_in_seer', 'base_yaw_in_seer')}
        fake('seer_odometry', dict(tf_params, use_sim_time=False, publish_tf=True))
        fake('seer_localization', {
            'use_sim_time': False, 'base_transform_confirmed': True,
            'map_alignment_confirmed': True, 'tf_publish_mode': 'bounded_odom_hold',
            'expected_map': 'fixture',
            'expected_map_md5': 'fixture-md5', 'base_x_in_seer_m': 0.,
            'base_y_in_seer_m': 0., 'base_yaw_in_seer_rad': guard_config['base_yaw_in_seer']})
        diagnostics = guard.create_publisher(DiagnosticArray, '/diagnostics', 10)
        scans = {side: guard.create_publisher(LaserScan, '/scan_' + side, qos_profile_sensor_data)
                 for side in ('front', 'back')}
        scans_enabled = {'front': True, 'back': True}
        def inputs():
            msg = DiagnosticArray()
            msg.header.stamp = guard.get_clock().now().to_msg()
            status = DiagnosticStatus(name='/navigation_guard')
            status.values = [KeyValue(key=key, value=value) for key, value in {
                'state': 'Ready', 'motion_enabled': 'true', 'controller_connected': 'true',
                'input_freshness_contract_verified': 'true', 'input:fixture': 'healthy',
                'tf_ownership_valid': 'true'}.items()]
            msg.status = [status]
            diagnostics.publish(msg)
            for side, pub in scans.items():
                if not scans_enabled[side]:
                    continue
                scan = LaserScan()
                scan.header.stamp = msg.header.stamp
                scan.header.frame_id = 'laser_' + side + '_link'
                scan.angle_min, scan.angle_max, scan.angle_increment = -.2, .2, .1
                scan.range_min, scan.range_max = .05, 10.
                scan.ranges = [4.] * 5
                pub.publish(scan)
        guard.create_timer(.05, inputs)
        manager_file = self.tmp / 'manager.yaml'
        manager_file.write_text(yaml.safe_dump({'lifecycle_manager_real_execution': {
            'ros__parameters': {'use_sim_time': False, 'autostart': False,
                               'node_names': [x[1] for x in EXECUTION_NODES]}}}))
        self.start(['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
                    '-r', '__node:=lifecycle_manager_real_execution', '--params-file', str(manager_file)],
                   'execution_manager')
        # Construct the actual gate with mock providers through ordinary ROS interfaces.
        gate = ExecutionGate(cli_args=['--ros-args', '-p', 'guard_params:=' + str(guard_file),
                                       '-p', 'map_file:=' + str(self.map),
                                       '-p', 'commissioning_file:=' + str(review)])
        self.extra_nodes.append(gate)
        self.executor.add_node(gate)
        gate.create_timer(.1, gate.tick)
        self.assertTrue(self.spin_until(lambda: gate.configured, 15), 'gate must CONFIGURE real plugins')
        self.assertTrue(self.spin_until(lambda: gate.active and gate.transition is None, 15),
                        'gate must RESUME only after live contracts pass: ' + str({
                            'state': gate.state, 'parameters': gate.parameter_ok,
                            'graph': gate.graph_ok(), 'latched': gate.latched,
                            'topics': {t: ([i.node_name for i in self.node.get_publishers_info_by_topic(t)],
                                           [i.node_name for i in self.node.get_subscriptions_info_by_topic(t)])
                                       for t in ['/navigation/cmd_vel_raw', '/navigation/cmd_vel_smoothed',
                                                 '/navigation/cmd_vel_guard']}}))
        scans_enabled['back'] = False
        self.assertTrue(self.spin_until(lambda: gate.latched and not gate.active, 5))
        self.assertTrue(self.spin_until(lambda: gate.transition is None, 10),
                        'lifecycle PAUSE must complete before ending the integration test')
        for _, name, _ in EXECUTION_NODES:
            client = self.node.create_client(GetState, '/' + name + '/get_state')
            future = client.call_async(GetState.Request())
            self.assertTrue(self.spin_until(future.done), name)
            self.assertEqual(future.result().current_state.id, 2, name + ' must be inactive')
        scans_enabled['back'] = True
        self.spin_until(lambda: False, 1)
        self.assertTrue(gate.latched, 'input recovery must not automatically resume execution')


if __name__ == '__main__':
    unittest.main()
