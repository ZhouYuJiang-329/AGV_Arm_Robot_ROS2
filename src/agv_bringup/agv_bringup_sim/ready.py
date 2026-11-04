"""Wait for simulator feedback before starting navigation and the second GUI."""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock


def main():
    rclpy.init()
    node = Node('agv_simulation_ready')
    if node.declare_parameter('wait_for_localization', False).value:
        from tf2_ros import Buffer, TransformListener
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        node.get_logger().info('Waiting for initial localization: use RViz 2D Pose Estimate')
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.1)
                if buffer.can_transform('map', 'base_link', rclpy.time.Time()):
                    node.get_logger().info('Initial localization ready; starting navigation lifecycle manager')
                    return
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            raise SystemExit(1)
        finally:
            del listener
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        raise SystemExit(1)
    received = set()
    subscriptions = [
        node.create_subscription(Odometry, '/odom', lambda m: received.add('odom'), qos_profile_sensor_data),
        node.create_subscription(LaserScan, '/scan', lambda m: received.add('scan') if m.ranges else None, qos_profile_sensor_data),
        node.create_subscription(Clock, '/clock', lambda m: received.add('clock') if m.clock.sec or m.clock.nanosec else None, qos_profile_sensor_data),
    ]
    deadline = time.monotonic() + 45
    while rclpy.ok() and received != {'odom', 'scan', 'clock'} and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.1)
    success = received == {'odom', 'scan', 'clock'}
    if success:
        node.get_logger().info('Simulator feedback ready; starting requested application and RViz')
    else:
        node.get_logger().error('Missing simulator inputs: ' + str({'odom', 'scan', 'clock'}-received))
    del subscriptions
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(0 if success else 1)
