#!/usr/bin/env python3
"""Drive a simulation-only mapping loop, verify live SLAM, and save its map.

Start agv_mapping_sim.launch.py first, at the default spawn with no keyboard
controller. This script moves the simulated robot around the scene perimeter.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time

from test_agv_navigation_sim import overlaps, yaw


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-directory', type=Path, default=Path('/tmp/agv-mapping-validation'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '187' or os.environ.get('GZ_PARTITION') != 'agv_navigation_sim':
        parser.error('Requires ROS_DOMAIN_ID=187 and GZ_PARTITION=agv_navigation_sim')
    output = args.output_directory.resolve()
    output.mkdir(parents=True, exist_ok=True)
    prefix = output/'practice'
    if prefix.with_suffix('.yaml').exists() or prefix.with_suffix('.pgm').exists():
        parser.error('Use a new output directory; an existing map will not be overwritten')

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import OccupancyGrid, Odometry
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformListener
    from ament_index_python.packages import get_package_share_directory
    from agv_bringup_sim.assets import scene_rectangles
    import yaml

    scene = yaml.safe_load((Path(get_package_share_directory('agv_bringup'))/'config/agv_navigation_scene.yaml').read_text())
    mapped, unmapped = scene_rectangles(scene)
    rclpy.init()
    node = rclpy.create_node('agv_mapping_integration_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
    received = {'pose': None, 'map': None, 'description': None, 'velocity': None}
    report = {'passed': False, 'collisions': [], 'waypoints': [], 'map_updates': 0}
    allowed = False
    command = node.create_publisher(Twist, '/cmd_vel', 10)
    buffer = Buffer()
    listener = TransformListener(buffer, node)

    def odometry(msg):
        p = msg.pose.pose
        received['pose'] = (p.position.x, p.position.y, yaw(p.orientation))
        received['velocity'] = [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.angular.z]
        if allowed:
            for rectangle in mapped+unmapped:
                if overlaps(received['pose'], scene['body']['size'][:2], rectangle):
                    if len(report['collisions']) < 10:
                        report['collisions'].append(rectangle['name'])

    def mapping(msg):
        received['map'] = msg
        report['map_updates'] += 1

    subscriptions = [
        node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data),
        node.create_subscription(OccupancyGrid, '/map', mapping, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        node.create_subscription(String, '/robot_description', lambda m: received.update(description=m.data),
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
    ]

    def wait(predicate, timeout, label):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not predicate():
            raise RuntimeError('Timed out: '+label)

    def publish(x=0., y=0., w=0.):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(x), float(y), float(w)
        command.publish(msg)

    try:
        wait(lambda: all(received[k] is not None for k in received), 45, 'SLAM map and simulator feedback')
        if 'name="agv_sim"' not in received['description'] or 'ros2_control' in received['description']:
            raise RuntimeError('Expected isolated Gazebo model')
        names = {name for name, _ in node.get_node_names_and_namespaces()}
        if 'slam_toolbox' not in names or names.intersection({'amcl', 'map_server', 'controller_server'}):
            raise RuntimeError('Expected mapping-only stack without navigation nodes')
        start = scene['start']
        if math.dist(received['pose'][:2], start[:2]) > .3:
            raise RuntimeError('Restart simulation at its initial spawn before testing')
        initial_map = received['map']
        report['initial_known_cells'] = sum(v >= 0 for v in initial_map.data)
        allowed = True
        for gx, gy in [(-4.8, -4.), (-4.8, 4.5), (4.7, 4.5), (4.7, -4.5), (-4., -4.5), (-4., -4.)]:
            deadline = time.monotonic()+180
            while True:
                x, y, theta = received['pose']
                dx, dy = gx-x, gy-y
                if math.hypot(dx, dy) < .08:
                    break
                if time.monotonic() > deadline or report['collisions']:
                    raise RuntimeError('Mapping route timed out or collided')
                ratio = min(.35/math.hypot(dx, dy), .8)
                vx, vy = dx*ratio, dy*ratio
                publish(math.cos(theta)*vx+math.sin(theta)*vy, -math.sin(theta)*vx+math.cos(theta)*vy,
                        max(-.3, min(.3, -theta)))
                rclpy.spin_once(node, timeout_sec=.05)
                time.sleep(.03)
            publish()
            report['waypoints'].append(list(received['pose']))
            print('Mapping waypoint reached:', gx, gy, flush=True)
        deadline = time.monotonic()+3
        while time.monotonic() < deadline:
            publish()
            rclpy.spin_once(node, timeout_sec=.05)
        grid = received['map']
        report['final_known_cells'] = sum(v >= 0 for v in grid.data)
        report['occupied_cells'] = sum(v > 50 for v in grid.data)
        report['free_cells'] = sum(v == 0 for v in grid.data)
        if report['map_updates'] < 3 or report['final_known_cells'] <= report['initial_known_cells']+100:
            raise RuntimeError('SLAM map did not grow during exploration')
        if report['occupied_cells'] < 100 or report['free_cells'] < 1000:
            raise RuntimeError('Expected both occupied and free cells in scanned map')
        transform = buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        report['final_map_pose'] = [transform.transform.translation.x, transform.transform.translation.y,
                                    yaw(transform.transform.rotation)]
        saved = subprocess.run(['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', str(prefix),
                                '--ros-args', '-p', 'use_sim_time:=true', '-p', 'save_map_timeout:=15.0'],
                               capture_output=True, text=True, timeout=45)
        (output/'map-saver.log').write_text(saved.stdout+saved.stderr)
        if saved.returncode or not prefix.with_suffix('.yaml').is_file() or not prefix.with_suffix('.pgm').is_file():
            raise RuntimeError('Map save failed; see map-saver.log')
        report['saved_map'] = str(prefix.with_suffix('.yaml'))
        report['passed'] = True
    finally:
        if allowed:
            deadline = time.monotonic()+.6
            while time.monotonic() < deadline:
                publish()
                rclpy.spin_once(node, timeout_sec=.05)
        (output/'mapping-result.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
        del subscriptions, listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
