#!/usr/bin/env python3
"""Record read-only ROS freshness and diagnostics; no controller connection or commands."""
import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seconds < 60:
        parser.error('--seconds must be at least 60')
    if args.output.exists():
        parser.error('output exists; choose a new filename')
    rclpy.init()
    node = Node('seer_localization_recorder')
    started = time.monotonic()
    rows = defaultdict(list)
    subscriptions = []

    def stamp_row(key, stamp):
        seconds = stamp.sec + stamp.nanosec / 1e9
        rows[key].append({'elapsed_sec': time.monotonic() - started,
                          'stamp_sec': seconds,
                          'receive_age_sec': node.get_clock().now().nanoseconds / 1e9 - seconds})

    for topic, msg_type in [('/odom', Odometry),
                            ('/seer/localization/base_pose', PoseStamped),
                            ('/scan_front', LaserScan), ('/scan_back', LaserScan)]:
        subscriptions.append(node.create_subscription(
            msg_type, topic, lambda msg, key=topic: stamp_row(key, msg.header.stamp),
            qos_profile_sensor_data))

    def transforms(msg):
        for tf in msg.transforms:
            key = tf.header.frame_id + '->' + tf.child_frame_id
            if key in ('map->odom', 'odom->base_link'):
                stamp_row(key, tf.header.stamp)

    def diagnostics(msg):
        for status in msg.status:
            rows['diagnostics'].append({
                'elapsed_sec': time.monotonic() - started, 'name': status.name,
                'level': int.from_bytes(status.level, 'little') if isinstance(
                    status.level, bytes) else int(status.level), 'message': status.message,
                'values': {v.key: v.value for v in status.values}})

    subscriptions.append(node.create_subscription(TFMessage, '/tf', transforms, 100))
    subscriptions.append(node.create_subscription(
        DiagnosticArray, '/diagnostics', diagnostics, 100))
    try:
        while time.monotonic() - started < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        duration = time.monotonic() - started
        output = {'recorded_utc': datetime.now(timezone.utc).isoformat(),
                  'duration_sec': duration, 'ros_domain_id': __import__('os').environ.get(
                      'ROS_DOMAIN_ID', '0'),
                  'mode': 'read_only_ros_receive_freshness_not_accuracy_validation',
                  'rows': dict(rows)}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        serialized = json.dumps(output, ensure_ascii=False, indent=2)
        with args.output.open('x') as stream:
            stream.write(serialized)
        node.destroy_node()
        rclpy.shutdown()
    for key, values in rows.items():
        print(f'{key}: {len(values)} observations / {duration:.2f}s')


if __name__ == '__main__':
    main()
