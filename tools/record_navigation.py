#!/usr/bin/python3
"""Passive Nav2 recorder: rosbag + JSONL events + map trajectory + SVG overview.

Creates subscribers only. Never sends goals, velocities, lifecycle or recovery requests.
"""
import argparse
import array
from collections import Counter, deque
import csv
from datetime import datetime, timezone
import html
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

sys.path.insert(0, str(Path(__file__).resolve().parent))
from record_steering import SteeringRecorder

PATH_TOPICS = ['/plan', '/smoothed_plan', '/local_plan', '/transformed_global_plan',
               '/received_global_plan']
LIGHT_TOPICS = PATH_TOPICS + ['/goal_pose', '/odom', '/seer/localization/base_pose',
    '/diagnostics', '/rosout', '/behavior_tree_log', '/collision_monitor_state',
    '/navigation/cmd_vel_raw', '/navigation/cmd_vel_smoothed', '/navigation/cmd_vel_guard']
for action in ('navigate_to_pose', 'navigate_through_poses'):
    LIGHT_TOPICS += [f'/{action}/_action/status', f'/{action}/_action/feedback']
BAG_TOPICS = LIGHT_TOPICS + ['/tf', '/tf_static', '/map', '/scan_front', '/scan_back',
    '/global_costmap/published_footprint', '/local_costmap/published_footprint',
    '/navigation/execution_heartbeat']
for action in ('navigate_to_pose', 'navigate_through_poses'):
    BAG_TOPICS += [f'/{action}/_action/{service}/_service_event'
                   for service in ('send_goal', 'cancel_goal', 'get_result')]


def clean(value):
    if isinstance(value, dict):
        return {str(k): clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, bytes, bytearray, array.array)):
        return [clean(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def atomic_json(path, value):
    temporary = path.with_suffix(path.suffix + '.tmp')
    temporary.write_text(json.dumps(clean(value), ensure_ascii=False, indent=2) + '\n')
    temporary.replace(path)


def snapshot_configuration(output):
    root = Path(__file__).resolve().parents[1]
    paths = ['src/robot_move/config/navigation_guard.yaml',
             'src/robot_move/config/seer_odometry_confirmed.yaml',
             'src/lidar/config/seer_lidar.yaml',
             'src/agv_description/config/lidar_mounts.yaml',
             'src/agv_bringup/config/agv_navigation_commissioning.yaml',
             'docs/validation/seer_low_speed_2026-11-13/nav2_locked_arm_site.yaml',
             'docs/validation/seer_low_speed_2026-11-13/localization_observe_site.yaml',
             'maps/seer/20251120141209997_navigable.yaml']
    map_file = root / paths[-1]
    if map_file.exists():
        image = (map_file.parent / yaml.safe_load(map_file.read_text())['image']).resolve()
        if image.is_relative_to(root):
            paths.append(str(image.relative_to(root)))
    files = {}
    for relative in paths:
        source = root / relative
        if not source.is_file():
            continue
        target = output / 'configuration' / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, target)
        files[relative] = hashlib.sha256(source.read_bytes()).hexdigest()
    atomic_json(output / 'configuration/manifest.json', {
        'note': 'Disk configuration snapshot; runtime overrides may differ.', 'sha256': files})


def svg_overview(actual, planned):
    """Standalone vector plot in map coordinates, without inventing an occupancy map."""
    points = list(actual) + list(planned)
    if not points:
        return '<svg xmlns="http://www.w3.org/2000/svg"><text x="10" y="25">No map trajectory yet</text></svg>'
    xs, ys = zip(*points)
    span = max(max(xs) - min(xs), max(ys) - min(ys), .5)
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    def project(p):
        return 400 + (p[0] - cx) * 650 / span, 400 - (p[1] - cy) * 650 / span
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="800" height="800" viewBox="0 0 800 800">',
             '<rect width="800" height="800" fill="white"/>',
             '<text x="20" y="25">Map frame: blue = latest plan, orange = actual trajectory</text>',
             '<text x="20" y="48">Latest plan only; all intermediate plans are in events.jsonl / bag.</text>']
    for values, color in ((planned, '#2463d5'), (actual, '#dc6d16')):
        if values:
            coords = ' '.join('%.2f,%.2f' % project(p) for p in values)
            parts.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="3"/>')
            x, y = project(values[-1])
            parts.append(f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="{color}"/>')
    parts.append('<text x="20" y="780">%s</text>' % html.escape(
        f'x [{min(xs):.2f}, {max(xs):.2f}] m; y [{min(ys):.2f}, {max(ys):.2f}] m; +Y upward'))
    return ''.join(parts) + '</svg>'


class NavigationRecorder(Node):
    def __init__(self, output):
        super().__init__('navigation_session_recorder_' + str(os.getpid()))
        self.output = output
        self.started = time.monotonic()
        self.started_utc = datetime.now(timezone.utc).isoformat()
        self.events = (output / 'events.jsonl').open('x')
        self.csv_file = (output / 'trajectory.csv').open('x', newline='')
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(['receive_unix_sec', 'pose_stamp_sec', 'frame', 'x_m', 'y_m', 'yaw_rad'])
        self.counts = Counter()
        self.subscriptions_by_topic = {}
        self.tf_buffer = Buffer()
        self.actual = deque(maxlen=20000)
        self.plans = {}
        self.latest_global = []
        self.last_pose_stamp = None
        self.distance = 0.0
        self.jumps = 0
        self.task_status = {}
        self.guard = {}
        self.guard_changes = deque(maxlen=200)
        self.last_guard_signature = None
        self.pose_skips = 0
        self.bag_status = 'disabled'
        self.steering_sampler = None
        self.requested_stop = False
        self.tf_subscriptions = []
        for topic in ('/tf', '/tf_static'):
            qos = (QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL)
                   if topic == '/tf_static' else 100)
            self.tf_subscriptions.append(self.create_subscription(
                TFMessage, topic, lambda msg, static=topic == '/tf_static': self.transforms(msg, static), qos))
        self.create_timer(1., self.discover)
        self.create_timer(.2, self.pose)
        self.create_timer(5., self.save)
        self.discover()

    def transforms(self, msg, static):
        for transform in msg.transforms:
            try:
                if static:
                    self.tf_buffer.set_transform_static(transform, 'recorded')
                else:
                    self.tf_buffer.set_transform(transform, 'recorded')
            except Exception as error:
                self.get_logger().warning(str(error))

    def discover(self):
        for topic, types in self.get_topic_names_and_types():
            if topic not in LIGHT_TOPICS or topic in self.subscriptions_by_topic or len(types) != 1:
                continue
            try:
                msg_type = get_message(types[0])
                # Best effort subscriptions also receive reliable publishers; volatile
                # avoids claiming historical commands/goals were received this session.
                self.subscriptions_by_topic[topic] = self.create_subscription(
                    msg_type, topic, lambda msg, topic=topic: self.record(topic, msg),
                    qos_profile_sensor_data)
            except (ImportError, AttributeError, ValueError) as error:
                self.get_logger().warning(f'Cannot subscribe {topic}: {error}')

    def record(self, topic, msg):
        data = clean(message_to_ordereddict(msg))
        self.counts[topic] += 1
        self.events.write(json.dumps({'receive_unix_sec': time.time(), 'topic': topic,
                                     'message': data}, ensure_ascii=False, allow_nan=False) + '\n')
        if topic in PATH_TOPICS:
            poses = data.get('poses', [])
            xy = [(p['pose']['position']['x'], p['pose']['position']['y']) for p in poses]
            frame = data.get('header', {}).get('frame_id')
            self.plans[topic] = {'updates': self.counts[topic], 'frame': frame,
                                'points': len(xy), 'planned_endpoint': xy[-1] if xy else None}
            if topic in ('/plan', '/smoothed_plan') and frame == 'map':
                self.latest_global = xy
        if topic.endswith('/_action/status'):
            for entry in data.get('status_list', []):
                goal_id = bytes(entry['goal_info']['goal_id']['uuid']).hex()
                self.task_status[goal_id] = {'topic': topic, 'status': entry['status']}
        if topic == '/diagnostics':
            for status in data.get('status', []):
                if status['name'] == '/navigation_guard':
                    fields = {item['key']: item['value'] for item in status['values']}
                    signature = (fields.get('state'), fields.get('stop_reason'))
                    self.guard = fields
                    if signature != self.last_guard_signature:
                        self.guard_changes.append({'receive_unix_sec': time.time(),
                                                   'state': signature[0], 'reason': signature[1]})
                        self.last_guard_signature = signature

    def pose(self):
        try:
            tf = self.tf_buffer.lookup_transform('map', 'base_link', Time())
            stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
            age = self.get_clock().now().nanoseconds / 1e9 - stamp
            if age < 0 or age > .8 or stamp == self.last_pose_stamp:
                self.pose_skips += 1
                return
            self.last_pose_stamp = stamp
            p, q = tf.transform.translation, tf.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            if self.actual:
                step = math.hypot(p.x - self.actual[-1][0], p.y - self.actual[-1][1])
                if step > .5:
                    self.jumps += 1
                else:
                    self.distance += step
            self.actual.append((p.x, p.y))
            self.csv.writerow([time.time(), stamp, 'map', p.x, p.y, yaw])
        except Exception:
            self.pose_skips += 1

    def save(self, finished=False):
        self.events.flush()
        self.csv_file.flush()
        atomic_json(self.output / 'summary.json', {
            'started_utc': self.started_utc, 'duration_sec': time.monotonic() - self.started,
            'finished': finished, 'read_only': True, 'motion_sent': False,
            'recorder_pid': os.getpid(),
            'configuration_snapshot': 'configuration/manifest.json',
            'domain_id': os.environ.get('ROS_DOMAIN_ID'), 'rosbag_status': self.bag_status,
            'steering': self.steering_sampler.summary() if self.steering_sampler else {'enabled': False},
            'message_counts': dict(self.counts), 'discovered_topics': sorted(self.subscriptions_by_topic),
            'tasks': self.task_status, 'plans': self.plans,
            'actual_distance_m_excluding_jumps': self.distance,
            'trajectory_points_in_overview': len(self.actual), 'localization_jumps_over_0_5m': self.jumps,
            'pose_checks_skipped_missing_stale_or_duplicate_tf': self.pose_skips,
            'last_guard': self.guard, 'guard_transitions': list(self.guard_changes),
            'notes': ['Task status codes: 4 succeeded, 5 canceled, 6 aborted.',
                      'Planned endpoint is not proof of the exact requested goal.',
                      'Exact action requests need service introspection already enabled upstream.',
                      'Trajectory is localization/TF estimated, not independent physical ground truth.',
                      'SVG shows the latest global plan and at most 20000 actual points; CSV retains all.']})
        temp = self.output / 'overview.svg.tmp'
        temp.write_text(svg_overview(self.actual, self.latest_global))
        temp.replace(self.output / 'overview.svg')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--domain-id', type=int, default=216)
    parser.add_argument('--seconds', type=float, default=3600, help='Default: one hour; Ctrl+C ends earlier')
    parser.add_argument('--no-bag', action='store_true', help='Only JSONL/CSV/SVG summary, no raw rosbag')
    parser.add_argument('--costmaps', action='store_true', help='Also record large full costmap topics')
    parser.add_argument('--seer-ip', help='Also read API1005 steering feedback from this IP (status port only)')
    parser.add_argument('--steering-rate-hz', type=float, default=5., help='Read-only steering polling rate, 0.5..10 Hz')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or args.seconds <= 0 or not 0 <= args.domain_id <= 232:
        parser.error('seconds must be positive/finite; domain-id must be 0..232')
    if not math.isfinite(args.steering_rate_hz) or not .5 <= args.steering_rate_hz <= 10:
        parser.error('steering-rate-hz must be finite and 0.5..10')
    if args.seer_ip:
        try:
            ipaddress.IPv4Address(args.seer_ip)
        except ipaddress.AddressValueError:
            parser.error('seer-ip must be an IPv4 address')
    output = args.output or Path('logs/navigation') / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=False)
    snapshot_configuration(output)
    (output / 'recorder.pid').write_text(str(os.getpid()) + '\n')
    if args.output is None:
        (output.parent / 'latest.txt').write_text(str(output.resolve()) + '\n')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    rclpy.init(domain_id=args.domain_id)
    node = NavigationRecorder(output)
    process = None
    bag_log = None
    print('Navigation record:', output.resolve(), flush=True)
    print('Read-only; no goals or motion sent. Ctrl+C ends and flushes the record.', flush=True)
    signal.signal(signal.SIGTERM, lambda *_: setattr(node, 'requested_stop', True))
    try:
        if args.seer_ip:
            node.steering_sampler = SteeringRecorder(output, args.seer_ip, args.steering_rate_hz)
            node.steering_sampler.start()
        if not args.no_bag:
            qos_file = output / 'bag_qos.yaml'
            qos_file.write_text('\n'.join(f'{topic}:\n  durability: transient_local\n  reliability: reliable\n  history: keep_last\n  depth: 100' for topic in ('/map', '/tf_static')) + '\n')
            bag_log = (output / 'rosbag.log').open('x')
            command = ['ros2', 'bag', 'record', '-s', 'sqlite3', '-o', str(output / 'bag'),
                       '--include-hidden-topics', '--disable-keyboard-controls',
                       '--max-bag-duration', '300', '--qos-profile-overrides-path', str(qos_file),
                       '--topics', *BAG_TOPICS]
            if args.costmaps:
                command += ['/global_costmap/costmap', '/local_costmap/costmap']
            process = subprocess.Popen(command, stdout=bag_log, stderr=subprocess.STDOUT,
                                       start_new_session=True)
            node.bag_status = 'recording'
        node.save()
        while rclpy.ok() and not node.requested_stop and time.monotonic() - node.started < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
            if process and process.poll() is not None and node.bag_status == 'recording':
                node.bag_status = f'exited_early:{process.returncode}; see rosbag.log'
                print(node.bag_status, flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        if process:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    node.bag_status = 'forced_stop; bag may be incomplete'
            if node.bag_status == 'recording':
                node.bag_status = 'closed' if process.returncode in (0, -signal.SIGINT) else f'exit:{process.returncode}'
        if bag_log:
            bag_log.close()
        if node.steering_sampler:
            node.steering_sampler.stop()
        node.save(finished=True)
        node.events.close()
        node.csv_file.close()
        (output / 'recorder.pid').unlink(missing_ok=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print('Saved:', output.resolve(), flush=True)


if __name__ == '__main__':
    main()
