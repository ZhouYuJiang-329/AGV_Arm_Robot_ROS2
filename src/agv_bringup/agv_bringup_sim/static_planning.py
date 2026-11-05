"""Turn RViz start/goal selections into explicit-start Nav2 planning requests."""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class StaticPlanning(Node):
    """Compute paths only; never publish velocity commands or move a robot."""
    def __init__(self):
        super().__init__('agv_static_planning')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.paths = self.create_publisher(Path, '/plan', qos)
        self.starts = self.create_publisher(PoseStamped, '/planning_start', qos)
        self.goals = self.create_publisher(PoseStamped, '/planning_goal', qos)
        self.start = self.goal = None
        self.revision = 0
        self.completed_revision = -1
        self.busy = False
        self.client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')
        self.start_sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.on_start, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.on_goal, 10)
        self.timer = self.create_timer(.2, self.plan)
        self.get_logger().info('Select start with 2D Pose Estimate, then goal with 2D Goal Pose. Static planning only.')

    def valid(self, message):
        p, q = message.pose.position, message.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if message.header.frame_id != 'map' or not all(math.isfinite(v) for v in values) or abs(sum(v*v for v in (q.x,q.y,q.z,q.w))-1) > .01:
            self.get_logger().error('Start/goal must be a finite map-frame pose with a unit quaternion')
            return False
        return True

    def clear_path(self):
        empty = Path()
        empty.header.frame_id = 'map'
        empty.header.stamp = self.get_clock().now().to_msg()
        self.paths.publish(empty)
        self.revision += 1

    def on_start(self, message):
        pose = PoseStamped(header=message.header, pose=message.pose.pose)
        if self.valid(pose):
            self.start = pose
            self.starts.publish(pose)
            self.clear_path()
            self.get_logger().info(f'Start: {pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}')

    def on_goal(self, message):
        if self.valid(message):
            self.goal = message
            self.goals.publish(message)
            self.clear_path()
            if self.start is None:
                self.get_logger().info('Goal received; select a start with 2D Pose Estimate')

    def plan(self):
        if self.busy or self.start is None or self.goal is None or self.completed_revision == self.revision or not self.client.server_is_ready():
            return
        request = ComputePathToPose.Goal()
        request.start, request.goal = self.start, self.goal
        request.start.header.stamp = request.goal.header.stamp = self.get_clock().now().to_msg()
        request.use_start = True
        request.planner_id = 'GridBased'
        self.busy = True
        revision = self.revision
        self.client.send_goal_async(request).add_done_callback(lambda f: self.accepted(f, revision))

    def accepted(self, future, revision):
        try:
            handle = future.result()
            if not handle.accepted:
                self.busy = False  # Lifecycle may still be activating; retry.
                return
            handle.get_result_async().add_done_callback(lambda f: self.finished(f, revision))
        except Exception as error:
            self.busy = False
            self.completed_revision = revision
            self.get_logger().error(f'Planner request failed: {error}')

    def finished(self, future, revision):
        self.busy = False
        self.completed_revision = revision
        if revision != self.revision:
            return  # Never display an obsolete start/goal result.
        try:
            response = future.result()
            if response.status != 4 or not response.result.path.poses:
                self.get_logger().warning(f'No path: status={response.status}, error={response.result.error_code}; choose free start/goal positions')
                return
            self.paths.publish(response.result.path)
            self.get_logger().info(f'Path computed: {len(response.result.path.poses)} poses')
        except Exception as error:
            self.get_logger().error(f'Planner result failed: {error}')


def main():
    rclpy.init()
    node = StaticPlanning()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
