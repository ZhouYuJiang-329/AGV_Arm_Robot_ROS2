"""ROS-to-Gazebo adapter for the ideal holonomic simulator only."""
import math
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class SimulationAdapter(Node):
    """Enforce bounded velocities and input timeout; normalize the lidar frame."""
    def __init__(self):
        super().__init__('agv_simulation_adapter')
        self.target = (0.0, 0.0, 0.0)
        self.received = -math.inf
        self.publisher = self.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        self.command_subscription = self.create_subscription(Twist, '/cmd_vel', self.on_command, 10)
        self.scan_subscription = self.create_subscription(LaserScan, '/sim/scan_raw', self.on_scan, qos_profile_sensor_data)
        self.wall_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(.05, self.tick, clock=self.wall_clock)

    def on_command(self, message):
        """Use body-frame vx/vy/w; Gazebo VelocityControl applies body rotation."""
        values = (message.linear.x, message.linear.y, message.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.target = (0.0, 0.0, 0.0)
            self.received = -math.inf
            return
        x, y, w = values
        speed = math.hypot(x, y)
        ratio = min(1.0, .45 / speed) if speed else 1.0
        self.target = (x * ratio, y * ratio, max(-.8, min(.8, w)))
        self.received = time.monotonic()

    def on_scan(self, message):
        """The generated sensor pose is exactly the base_link -> laser_link TF."""
        message.header.frame_id = 'laser_link'
        self.scan_publisher.publish(message)

    def tick(self):
        """Stop after 0.4 wall-clock seconds without a fresh velocity command."""
        self.publish(self.target if time.monotonic() - self.received < .4 else (0.0, 0.0, 0.0))

    def publish(self, velocity):
        message = Twist()
        message.linear.x, message.linear.y, message.angular.z = map(float, velocity)
        self.publisher.publish(message)


def main():
    rclpy.init()
    node = SimulationAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.publish((0.0, 0.0, 0.0))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
