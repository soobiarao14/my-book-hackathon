#!/usr/bin/env python3
"""
Obstacle Monitor Node

Monitors for dynamic obstacles and publishes safety alerts.

**Purpose**:
Real-time obstacle detection using depth camera for emergency stops.

**Subscribed Topics**:
- `/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image): Depth image
- `/cmd_vel` (geometry_msgs/Twist): Current velocity command

**Published Topics**:
- `/obstacle_alert` (std_msgs/Bool): True if obstacle in danger zone
- `/min_obstacle_distance` (std_msgs/Float32): Closest obstacle distance in meters

**Services**: None

**Actions**: None

**Parameters**:
- `danger_zone_distance` (float): Stop threshold distance in meters (default: 0.5)
- `warning_zone_distance` (float): Slow down threshold in meters (default: 1.0)
- `monitoring_rate` (int): Update frequency in Hz (default: 20)

**Implementation Requirements** (FR-011, SC-007):
- Detect obstacles <0.5m and trigger emergency stop
- Publish alerts at 20Hz for real-time response
- Use depth camera to measure obstacle distances
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32


class ObstacleMonitorNode(Node):
    """Real-time obstacle monitoring for safety."""

    def __init__(self):
        super().__init__('obstacle_monitor_node')

        # TODO: Declare parameters
        self.declare_parameter('danger_zone_distance', 0.5)
        self.declare_parameter('warning_zone_distance', 1.0)
        self.declare_parameter('monitoring_rate', 20)

        # Create subscribers
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publishers
        self.alert_pub = self.create_publisher(
            Bool,
            '/obstacle_alert',
            10
        )
        self.distance_pub = self.create_publisher(
            Float32,
            '/min_obstacle_distance',
            10
        )

        # TODO: Create timer for monitoring at monitoring_rate

        self.get_logger().info('Obstacle Monitor Node initialized (STUB)')

    def depth_callback(self, msg: Image):
        """TODO: Buffer depth image for obstacle detection."""
        pass

    def cmd_vel_callback(self, msg: Twist):
        """TODO: Monitor velocity commands."""
        pass

    def monitor_obstacles(self):
        """TODO: Check for obstacles in danger/warning zones."""
        pass

    def find_min_distance(self, depth_image):
        """TODO: Find minimum distance in depth image."""
        pass

    def publish_alert(self, is_danger):
        """TODO: Publish obstacle alert."""
        msg = Bool()
        msg.data = is_danger
        self.alert_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
