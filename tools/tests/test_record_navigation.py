#!/usr/bin/python3
"""Offline unit tests and isolated ROS recorder integration; no robot connections."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('record_navigation', ROOT / 'tools/record_navigation.py')
recorder = importlib.util.module_from_spec(spec)
spec.loader.exec_module(recorder)


class RecorderTest(unittest.TestCase):
    def test_json_preserves_zero_and_sanitizes_nonfinite(self):
        result = recorder.clean({'v': [0.0, float('nan'), float('inf')], 'uuid': b'\x01\x02'})
        self.assertEqual(result, {'v': [0.0, None, None], 'uuid': [1, 2]})
        json.dumps(result, allow_nan=False)

    def test_svg_handles_empty_and_stationary_trajectory(self):
        for actual, plan in (([], []), ([(1, 2)] * 3, [(1, 2)]),
                             ([(0, 0), (1, 0)], [(0, 0), (1, 1)])):
            root = ET.fromstring(recorder.svg_overview(actual, plan))
            self.assertTrue(root.tag.endswith('svg'))

    def test_atomic_summary_is_readable_and_leaves_no_temporary_file(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'summary.json'
            recorder.atomic_json(path, {'finished': False})
            recorder.atomic_json(path, {'finished': True})
            self.assertTrue(json.loads(path.read_text())['finished'])
            self.assertFalse(path.with_suffix('.json.tmp').exists())

    def test_recording_in_isolated_domain(self):
        import rclpy
        from action_msgs.msg import GoalStatus, GoalStatusArray
        from geometry_msgs.msg import PoseStamped, TransformStamped
        from nav_msgs.msg import Path as RosPath
        from rclpy.node import Node
        from tf2_msgs.msg import TFMessage
        rclpy.init(domain_id=222)
        node = Node('navigation_recording_fixture')
        tf = node.create_publisher(TFMessage, '/tf', 10)
        plan = node.create_publisher(RosPath, '/plan', 10)
        status = node.create_publisher(GoalStatusArray, '/navigate_to_pose/_action/status', 10)
        process = None
        try:
            with tempfile.TemporaryDirectory(prefix='navigation-recorder-test-') as tmp:
                output = Path(tmp) / 'session'
                env = dict(os.environ, ROS_DOMAIN_ID='222', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
                with (Path(tmp) / 'process.log').open('w+') as log:
                    process = subprocess.Popen([sys.executable, str(ROOT / 'tools/record_navigation.py'),
                                                '--domain-id', '222', '--seconds', '7',
                                                '--output', str(output)], env=env, stdout=log,
                                               stderr=subprocess.STDOUT)
                    start = time.monotonic()
                    while time.monotonic() - start < 9 and process.poll() is None:
                        elapsed = time.monotonic() - start
                        message = TFMessage()
                        for parent, child, x in [('map', 'odom', 0.),
                                                 ('odom', 'base_link', min(elapsed * .03, .2))]:
                            t = TransformStamped()
                            t.header.stamp = node.get_clock().now().to_msg()
                            t.header.frame_id, t.child_frame_id = parent, child
                            t.transform.translation.x = x
                            t.transform.rotation.w = 1.
                            message.transforms.append(t)
                        tf.publish(message)
                        path = RosPath()
                        path.header.frame_id = 'map'
                        path.header.stamp = node.get_clock().now().to_msg()
                        for x in (0., .1, .2):
                            pose = PoseStamped()
                            pose.header = path.header
                            pose.pose.position.x = x
                            pose.pose.orientation.w = 1.
                            path.poses.append(pose)
                        plan.publish(path)
                        goal = GoalStatus()
                        goal.goal_info.goal_id.uuid = [1] * 16
                        goal.status = 4 if elapsed > 4 else 2
                        statuses = GoalStatusArray()
                        statuses.status_list = [goal]
                        status.publish(statuses)
                        rclpy.spin_once(node, timeout_sec=.05)
                    process.wait(timeout=15)
                    log.seek(0)
                    self.assertEqual(process.returncode, 0, log.read())
                data = json.loads((output / 'summary.json').read_text())
                self.assertTrue(data['finished'])
                self.assertEqual(data['rosbag_status'], 'closed')
                self.assertGreater(data['trajectory_points_in_overview'], 3)
                self.assertGreater(data['actual_distance_m_excluding_jumps'], .02)
                self.assertEqual(data['plans']['/plan']['planned_endpoint'], [.2, 0.])
                self.assertEqual(data['tasks']['01' * 16]['status'], 4)
                self.assertTrue((output / 'bag/metadata.yaml').exists())
                self.assertFalse((output / 'recorder.pid').exists())
                self.assertGreater(len((output / 'events.jsonl').read_text().splitlines()), 5)
                ET.parse(output / 'overview.svg')
        finally:
            if process and process.poll() is None:
                process.terminate()
                process.wait(timeout=20)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    unittest.main()
