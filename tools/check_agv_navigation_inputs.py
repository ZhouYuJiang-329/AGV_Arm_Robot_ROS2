#!/usr/bin/env python3
"""Report whether the basic ROS inputs for AGV navigation are actually present.

This read-only probe publishes no velocity or navigation goals. Passing these
checks verifies input plumbing, not localization accuracy or navigation quality.
"""
import argparse
import json
import math
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--timeout', type=float, default=8.0)
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--odom-frame', default='odom')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--real-time', action='store_true', help='Do not require a simulation clock')
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan
    from rosgraph_msgs.msg import Clock
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    node = rclpy.create_node('check_agv_navigation_inputs',
                            parameter_overrides=[Parameter('use_sim_time', value=not args.real_time)])
    seen = {'clock': [], 'odom': None, 'scan': None}
    received = {}

    def callback(key):
        def receive(message):
            if key == 'clock':
                seen[key].append(message.clock.sec + message.clock.nanosec / 1e9)
                seen[key] = seen[key][-20:]
            else:
                seen[key] = message
            received[key] = time.monotonic()
        return receive

    subscriptions = [
        node.create_subscription(Odometry, args.odom_topic, callback('odom'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, args.scan_topic, callback('scan'), qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', callback('clock'), qos_profile_sensor_data),
    ]
    buffer = Buffer(node=node)
    listener = TransformListener(buffer, node, spin_thread=False)
    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)

    now = time.monotonic()
    fresh = lambda key: key in received and now - received[key] < 1.0
    odom, scan = seen['odom'], seen['scan']
    checks = {
        'clock_advancing': args.real_time or (len(seen['clock']) >= 2 and fresh('clock') and
                                             seen['clock'][-1] > seen['clock'][0]),
        'odometry_fresh': bool(odom and fresh('odom') and
                               odom.header.frame_id == args.odom_frame and odom.child_frame_id),
        'laser_scan_fresh': bool(scan and fresh('scan') and scan.header.frame_id and scan.ranges and
                                scan.range_max > scan.range_min >= 0),
        'map_to_odom_tf': bool(buffer.can_transform(args.map_frame, args.odom_frame, Time())),
        'odom_to_base_tf': bool(buffer.can_transform(args.odom_frame, args.base_frame, Time())),
        'base_to_laser_tf': bool(scan and buffer.can_transform(args.base_frame, scan.header.frame_id, Time())),
    }
    result = {'basic_inputs_present': all(checks.values()), 'checks': checks,
              'topics': {'odom': args.odom_topic, 'scan': args.scan_topic},
              'note': 'This is an input check, not a successful navigation test.'}
    print(json.dumps(result, indent=2, ensure_ascii=False))
    listener.unregister()
    del subscriptions
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result['basic_inputs_present'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
