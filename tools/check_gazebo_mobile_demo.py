#!/usr/bin/env python3
"""Verify the official Gazebo differential-drive demo in an isolated ROS domain.

Start the official demo first; see docs/Gazebo环境搭建与验收.md.
This probe checks controllers and the simulated hardware before sending motion.
It uses Gazebo ground truth as well as ROS odometry, not command values alone.
"""

import argparse
import json
import math
import os
from pathlib import Path
import threading
import time
import xml.etree.ElementTree as ET


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angle_delta(start, end):
    return math.atan2(math.sin(end - start), math.cos(end - start))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/agv-gazebo-validation/mobile-result.json'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '187':
        parser.error('Use the isolated validation domain: export ROS_DOMAIN_ID=187')
    if os.environ.get('GZ_PARTITION') != 'agv_gazebo_validation_zyj':
        parser.error('Set GZ_PARTITION=agv_gazebo_validation_zyj for the validation simulator')

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
    from controller_manager_msgs.srv import ListControllers, ListHardwareComponents
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from gz.transport13 import Node as GzNode
    from gz.msgs10.pose_v_pb2 import Pose_V

    rclpy.init(domain_id=187)
    node = rclpy.create_node('check_gazebo_mobile_demo', parameter_overrides=[Parameter('use_sim_time', value=True)])
    current = {'clock': None, 'odom': None, 'joints': set(), 'tf': set(), 'ground_truth': None, 'description': None}
    lock = threading.Lock()
    report = {'passed': False, 'domain_id': 187, 'partition': os.environ['GZ_PARTITION']}
    publisher = node.create_publisher(TwistStamped, '/diff_drive_base_controller/cmd_vel', 10)
    motion_allowed = False

    def clock_callback(msg):
        current['clock'] = msg.clock.sec + msg.clock.nanosec / 1e9

    def odom_callback(msg):
        p = msg.pose.pose
        current['odom'] = (p.position.x, p.position.y, yaw(p.orientation))
        current['odom_velocity'] = (msg.twist.twist.linear.x, msg.twist.twist.angular.z)
        current['odom_frames'] = (msg.header.frame_id, msg.child_frame_id)

    def joint_callback(msg):
        current['joints'].update(msg.name)

    def tf_callback(msg):
        current['tf'].update((t.header.frame_id, t.child_frame_id) for t in msg.transforms)

    def ground_truth_callback(msg):
        for p in msg.pose:
            if p.name == 'diff_drive':
                with lock:
                    current['ground_truth'] = (p.position.x, p.position.y, yaw(p.orientation))

    def description_callback(msg):
        current['description'] = msg.data

    subscriptions = [
        node.create_subscription(String, '/robot_description', description_callback,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(Clock, '/clock', clock_callback, qos_profile_sensor_data),
        node.create_subscription(Odometry, '/diff_drive_base_controller/odom', odom_callback, qos_profile_sensor_data),
        node.create_subscription(JointState, '/joint_states', joint_callback, qos_profile_sensor_data),
        node.create_subscription(TFMessage, '/tf', tf_callback, qos_profile_sensor_data),
    ]
    gz_node = GzNode()
    gz_node.subscribe(Pose_V, '/world/empty/dynamic_pose/info', ground_truth_callback)

    def spin_until(predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not predicate():
            raise RuntimeError('Timed out: ' + description)

    def call(service_type, name):
        client = node.create_client(service_type, name)
        try:
            if not client.wait_for_service(timeout_sec=15):
                raise RuntimeError('Service unavailable: ' + name)
            future = client.call_async(service_type.Request())
            spin_until(future.done, 10, name)
            return future.result()
        finally:
            node.destroy_client(client)

    def command(v, w):
        message = TwistStamped()
        message.header.stamp = node.get_clock().now().to_msg()
        message.twist.linear.x = float(v)
        message.twist.angular.z = float(w)
        publisher.publish(message)

    def run_for(seconds, v, w):
        start = current['clock']
        deadline = time.monotonic() + max(15, seconds * 5)
        next_publish = 0.0
        while current['clock'] - start < seconds:
            now = time.monotonic()
            if now > deadline:
                raise RuntimeError('Simulation clock stalled during motion')
            if now >= next_publish:
                command(v, w)
                next_publish = now + .05
            rclpy.spin_once(node, timeout_sec=.01)

    def snapshot():
        with lock:
            return {'odom': current['odom'], 'ground_truth': current['ground_truth'], 'clock': current['clock']}

    try:
        components = call(ListHardwareComponents, '/controller_manager/list_hardware_components').component
        report['hardware'] = [{'name': c.name, 'plugin': c.plugin_name, 'state': c.state.label} for c in components]
        # Gazebo imports components without filling plugin_name on some Jazzy versions.
        # Verify the live, latched URDF as well as the actual component and world model.
        spin_until(lambda: current['description'] is not None, 10, 'live robot description')
        systems = ET.fromstring(current['description']).findall('ros2_control')
        plugins = [system.findtext('hardware/plugin') for system in systems]
        report['description_plugins'] = plugins
        if (plugins != ['gz_ros2_control/GazeboSimSystem'] or len(components) != 1 or
                components[0].name != systems[0].get('name') or
                components[0].state.label != 'active'):
            raise RuntimeError('Refusing motion: expected only the active Gazebo simulated hardware')
        deadline = time.monotonic() + 30
        while True:
            controllers = call(ListControllers, '/controller_manager/list_controllers').controller
            report['controllers'] = {c.name: c.state for c in controllers}
            if all(report['controllers'].get(name) == 'active' for name in
                   ('joint_state_broadcaster', 'diff_drive_base_controller')):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Expected controllers did not become active')
            rclpy.spin_once(node, timeout_sec=.2)
        spin_until(lambda: current['clock'] is not None and current['odom'] is not None and
                   current['ground_truth'] is not None and publisher.get_subscription_count() > 0,
                   15, 'clock, odometry, Gazebo model pose and command subscriber')
        motion_allowed = True
        run_for(.5, 0, 0)
        report['initial'] = snapshot()
        run_for(2.0, .2, 0)
        run_for(.8, 0, 0)
        report['after_forward'] = snapshot()
        run_for(2.0, 0, .4)
        run_for(.8, 0, 0)
        report['after_turn'] = snapshot()
        for source in ('odom', 'ground_truth'):
            a, b, c = (report[k][source] for k in ('initial', 'after_forward', 'after_turn'))
            report[source + '_distance_m'] = math.hypot(b[0] - a[0], b[1] - a[1])
            report[source + '_turn_rad'] = angle_delta(b[2], c[2])
            if not .15 < report[source + '_distance_m'] < 1.0:
                raise RuntimeError('Forward displacement is outside expected range: ' + source)
            if not .3 < report[source + '_turn_rad'] < 1.5:
                raise RuntimeError('Turn angle is outside expected range: ' + source)
        report['joint_names'] = sorted(current['joints'])
        report['tf_edges'] = sorted(current['tf'])
        report['odom_frames'] = current['odom_frames']
        report['final_velocity'] = current['odom_velocity']
        if not {'left_wheel_joint', 'right_wheel_joint'} <= current['joints']:
            raise RuntimeError('Wheel joint feedback missing')
        if ('odom', 'chassis') not in current['tf']:
            raise RuntimeError('odom -> chassis TF missing')
        if any(abs(v) > .03 for v in current['odom_velocity']):
            raise RuntimeError('Robot did not settle after zero velocity commands')
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
    finally:
        if motion_allowed:
            # Only the previously verified isolated simulator receives these stop commands.
            deadline = time.monotonic() + .5
            while time.monotonic() < deadline:
                command(0, 0)
                rclpy.spin_once(node, timeout_sec=.02)
        gz_node.unsubscribe('/world/empty/dynamic_pose/info')
        del gz_node
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(json.dumps(report, indent=2, ensure_ascii=False))
        del subscriptions
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
