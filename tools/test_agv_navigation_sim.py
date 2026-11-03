#!/usr/bin/env python3
"""Exercise the isolated AGV Gazebo/Nav2 sandbox, including real motion and cancel."""
import argparse
import json
import math
import os
from pathlib import Path
import time


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def overlaps(pose, size, rectangle):
    """Separating-axis test of the physical body and an axis-aligned obstacle."""
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    dx, dy = rectangle['center'][0]-x, rectangle['center'][1]-y
    for ax, ay in ((1, 0), (0, 1), (c, s), (-s, c)):
        robot = abs(ax*c+ay*s)*size[0]/2 + abs(-ax*s+ay*c)*size[1]/2
        obstacle = abs(ax)*rectangle['size'][0]/2 + abs(ay)*rectangle['size'][1]/2
        if abs(dx*ax+dy*ay) >= robot+obstacle-.01:
            return False
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('/tmp/agv-nav-validation/navigation-result.json'))
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '187' or os.environ.get('GZ_PARTITION') != 'agv_navigation_sim':
        parser.error('Set ROS_DOMAIN_ID=187 and GZ_PARTITION=agv_navigation_sim; this test is simulation only')

    import yaml
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from ament_index_python.packages import get_package_share_directory
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, Twist
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav2_msgs.action import NavigateToPose, ComputePathToPose
    from lifecycle_msgs.srv import GetState
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from agv_bringup_sim.assets import scene_rectangles

    scene = yaml.safe_load((Path(get_package_share_directory('agv_bringup'))/'config/agv_navigation_scene.yaml').read_text())
    mapped, unmapped = scene_rectangles(scene)
    rectangles = mapped + unmapped
    body_size = scene['body']['size'][:2]
    rclpy.init()
    node = rclpy.create_node('agv_navigation_integration_test', parameter_overrides=[Parameter('use_sim_time', value=True)])
    current = {'pose': None, 'velocity': None, 'scan': None, 'description': None, 'map': None, 'costmap': None}
    record = {'passed': False, 'goals': [], 'collisions': [], 'samples': 0}
    path_samples = []
    command = node.create_publisher(Twist, '/cmd_vel_nav', 10)
    navigation = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    planning = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
    active = None
    motion_allowed = False

    def odometry(message):
        p = message.pose.pose
        current['pose'] = (p.position.x, p.position.y, yaw(p.orientation))
        current['velocity'] = (message.twist.twist.linear.x, message.twist.twist.linear.y, message.twist.twist.angular.z)
        record['samples'] += 1
        if active is not None:
            path_samples.append(current['pose'])
            for rectangle in rectangles:
                if overlaps(current['pose'], body_size, rectangle) and len(record['collisions']) < 10:
                    record['collisions'].append({'obstacle': rectangle['name'], 'pose': current['pose']})

    def receive(key):
        return lambda message: current.__setitem__(key, message)

    subs = [node.create_subscription(OccupancyGrid, '/map', receive('map'), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(OccupancyGrid, '/global_costmap/costmap', receive('costmap'), QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            node.create_subscription(Odometry, '/odom', odometry, qos_profile_sensor_data),
            node.create_subscription(LaserScan, '/scan', receive('scan'), qos_profile_sensor_data),
            node.create_subscription(String, '/robot_description', receive('description'),
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))]

    def wait(predicate, seconds, description):
        deadline = time.monotonic()+seconds
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        if not predicate():
            raise RuntimeError('Timed out: '+description)

    def call_future(future, seconds, description):
        wait(future.done, seconds, description)
        return future.result()

    def hold(seconds, velocity=None):
        start = node.get_clock().now().nanoseconds/1e9
        deadline = time.monotonic()+max(10, seconds*8)
        next_send = 0
        while node.get_clock().now().nanoseconds/1e9-start < seconds:
            if time.monotonic() > deadline:
                raise RuntimeError('Simulation clock stalled')
            if velocity is not None and time.monotonic() >= next_send:
                message = Twist()
                message.linear.x, message.linear.y, message.angular.z = map(float, velocity)
                command.publish(message)
                next_send = time.monotonic()+.05
            rclpy.spin_once(node, timeout_sec=.01)

    def target(x, y, angle=0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        pose.pose.orientation.z, pose.pose.orientation.w = math.sin(angle/2), math.cos(angle/2)
        return pose

    try:
        wait(lambda: current['pose'] is not None and current['scan'] is not None and current['description'] is not None,
             30, 'simulator inputs')
        if 'agv_sim' not in current['description'].data or 'ros2_control' in current['description'].data:
            raise RuntimeError('Refusing motion: unexpected robot model')
        states = {}
        for name in ('amcl', 'map_server', 'controller_server', 'planner_server', 'bt_navigator', 'collision_monitor'):
            client = node.create_client(GetState, '/' + name + '/get_state')
            deadline = time.monotonic() + 45
            while time.monotonic() < deadline:
                if client.wait_for_service(timeout_sec=.5):
                    state = call_future(client.call_async(GetState.Request()), 5, name+' lifecycle')
                    states[name] = state.current_state.label
                    if states[name] == 'active':
                        break
                rclpy.spin_once(node, timeout_sec=.1)
            node.destroy_client(client)
            if states.get(name) != 'active':
                raise RuntimeError('Lifecycle node failed to activate: '+name)
        record['lifecycle'] = states
        if math.hypot(current['pose'][0]-scene['start'][0], current['pose'][1]-scene['start'][1]) > .3:
            raise RuntimeError('Restart the simulator before running this integration test')
        motion_allowed = True
        if not navigation.wait_for_server(timeout_sec=45) or not planning.wait_for_server(timeout_sec=10):
            raise RuntimeError('Nav2 actions unavailable')
        hold(.5, (0, 0, 0))
        start = current['pose']
        hold(1, (0, .2, 0))
        hold(1, (0, 0, 0))
        after = current['pose']
        record['lateral_test'] = {'initial': start, 'after': after, 'distance': math.hypot(after[0]-start[0], after[1]-start[1])}
        if not .08 < record['lateral_test']['distance'] < .4 or abs(after[2]-start[2]) > .05:
            raise RuntimeError('Holonomic lateral motion failed')
        hold(1, (0, -.2, 0))
        # Stop publishing; the smoothing/adapter timeout must bring the body to rest.
        hold(1.8)
        record['timeout_velocity'] = current['velocity']
        if any(abs(v) > .03 for v in current['velocity']):
            raise RuntimeError('Command timeout did not stop the body')
        scan = current['scan']
        record['lidar'] = {'frame': scan.header.frame_id, 'rays': len(scan.ranges),
                           'finite_hits': sum(math.isfinite(v) for v in scan.ranges)}
        if scan.header.frame_id != 'laser_link' or record['lidar']['finite_hits'] < 50:
            raise RuntimeError('Lidar obstacle returns missing')

        def grid_cost(message, x, y):
            if message is None:
                return None
            column = int((x-message.info.origin.position.x)/message.info.resolution)
            row = int((y-message.info.origin.position.y)/message.info.resolution)
            if not (0 <= column < message.info.width and 0 <= row < message.info.height):
                return None
            return int(message.data[row*message.info.width+column])

        obstacle = scene['unmapped_obstacles'][0]
        ox, oy = obstacle['center']
        wait(lambda: (grid_cost(current['costmap'], ox, oy) or 0) > 20,
             15, 'lidar-only obstacle appearing in the costmap')
        record['unmapped_obstacle'] = {'prior_map': grid_cost(current['map'], ox, oy),
                                       'costmap': grid_cost(current['costmap'], ox, oy)}
        if record['unmapped_obstacle']['prior_map'] != 0:
            raise RuntimeError('Obstacle test must use a location free in the prior map')

        request = ComputePathToPose.Goal()
        request.goal = target(3, 3)
        request.planner_id = 'GridBased'
        handle = call_future(planning.send_goal_async(request), 10, 'planner acceptance')
        if not handle.accepted:
            raise RuntimeError('Plan rejected')
        result = call_future(handle.get_result_async(), 20, 'plan result')
        points = result.result.path.poses
        if result.status != GoalStatus.STATUS_SUCCEEDED or len(points) < 10:
            raise RuntimeError('Planning failed')
        record['planned_points'] = len(points)
        record['planned_length'] = sum(math.hypot(b.pose.position.x-a.pose.position.x, b.pose.position.y-a.pose.position.y)
                                       for a,b in zip(points,points[1:]))
        print('Planner ready:', record['planned_points'], 'points;', round(record['planned_length'],2), 'm', flush=True)

        for x, y in ((3, 3), (-3, 3)):
            goal = NavigateToPose.Goal();goal.pose = target(x, y)
            active = call_future(navigation.send_goal_async(goal), 10, 'goal acceptance')
            if not active.accepted:
                raise RuntimeError('Navigation goal rejected')
            outcome = active.get_result_async()
            deadline = time.monotonic()+180
            next_log = 0
            while not outcome.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
                if time.monotonic() > next_log:
                    print('Navigating to', (x,y), 'pose=', current['pose'], flush=True)
                    next_log = time.monotonic()+5
                if record['collisions']:
                    raise RuntimeError('Robot footprint overlapped an obstacle')
            if not outcome.done():
                raise RuntimeError('Navigation timed out')
            result = outcome.result()
            actual = current['pose']
            error = math.hypot(actual[0]-x, actual[1]-y)
            record['goals'].append({'target': [x,y], 'status': result.status, 'actual': actual, 'position_error': error, 'yaw_error': abs(math.atan2(math.sin(actual[2]), math.cos(actual[2])))})
            active = None
            if result.status != GoalStatus.STATUS_SUCCEEDED or error > .3 or record['goals'][-1]['yaw_error'] > .25:
                raise RuntimeError('Navigation did not reach its target')
            hold(1)

        goal = NavigateToPose.Goal();goal.pose = target(-4,-4)
        active = call_future(navigation.send_goal_async(goal), 10, 'cancel-test goal acceptance')
        if not active.accepted:
            raise RuntimeError('Cancel-test goal rejected')
        hold(1.5)
        cancel = call_future(active.cancel_goal_async(), 10, 'cancel acknowledgement')
        result = call_future(active.get_result_async(), 15, 'cancel result')
        active = None
        hold(1.5)
        record['cancel'] = {'status': result.status, 'acknowledged': bool(cancel.goals_canceling), 'velocity': current['velocity']}
        if result.status != GoalStatus.STATUS_CANCELED or not record['cancel']['acknowledged'] or any(abs(v) > .03 for v in current['velocity']):
            raise RuntimeError('Cancel did not end the task and stop motion')
        record['trajectory_samples'] = len(path_samples)
        if record['collisions']:
            raise RuntimeError('Collision detected in recorded trajectory')
        record['passed'] = True
    except Exception as error:
        record['error'] = str(error)
    finally:
        if active is not None:
            try:
                call_future(active.cancel_goal_async(), 5, 'cleanup cancel')
            except Exception:
                pass
        if motion_allowed:
            deadline = time.monotonic() + .7
            while time.monotonic() < deadline:
                command.publish(Twist())
                rclpy.spin_once(node, timeout_sec=.02)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(record, indent=2)+'\n')
        print(json.dumps(record,indent=2),flush=True)
        del subs
        node.destroy_node();rclpy.shutdown()
    return 0 if record['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
