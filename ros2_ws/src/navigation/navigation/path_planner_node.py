#!/usr/bin/env python3
"""
Path Planner Node

Plans collision-free paths using Nav2 stack.

**Purpose**:
Generates feasible paths from current pose to goal pose avoiding obstacles.

**Subscribed Topics**:
- `/navigation/goal` (geometry_msgs/PoseStamped): Navigation goal from task executor
- `/visual_slam/tracking/odometry` (nav_msgs/Odometry): Current robot pose
- `/map` (nav_msgs/OccupancyGrid): Occupancy grid from SLAM
- `/perception/human_detected` (std_msgs/Bool): Human detection flag

**Published Topics**:
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to motors
- `/navigation/status` (std_msgs/String): Navigation status

**Services**: None

**Actions**:
- `/navigate_to_pose` (nav2_msgs/NavigateToPose): Nav2 action interface

**Parameters**:
- `controller_frequency` (float): Control loop frequency in Hz (default: 20.0)
- `planner_plugin` (str): Nav2 planner plugin (default: "NavfnPlanner")
- `controller_plugin` (str): Nav2 controller plugin (default: "DWB")
- `max_speed` (float): Maximum speed in m/s (default: 0.8)
- `safe_speed_near_human` (float): Speed limit near humans in m/s (default: 0.5)

**Implementation Requirements** (FR-009, FR-010, SC-006):
- Use Nav2 planner (NavFn or Smac Planner) for global path
- Use DWB or TEB controller for local path tracking
- Reduce speed to 0.5 m/s when human detected within 1m
- Meet <10cm position error, <5° heading error at goals
- Stop within 0.5m of obstacles
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool


class PathPlannerNode(Node):
    """Nav2-based path planning and control."""

    def __init__(self):
        super().__init__('path_planner_node')

        # TODO: Declare parameters
        self.declare_parameter('controller_frequency', 20.0)
        self.declare_parameter('planner_plugin', 'NavfnPlanner')
        self.declare_parameter('controller_plugin', 'DWB')
        self.declare_parameter('max_speed', 0.8)
        self.declare_parameter('safe_speed_near_human', 0.5)

        # TODO: Initialize Nav2 action client

        # TODO: Initialize local variables
        self.human_detected = False
        self.current_goal = None

        # Create subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/navigation/goal',
            self.goal_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/visual_slam/tracking/odometry',
            self.odom_callback,
            10
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.human_sub = self.create_subscription(
            Bool,
            '/perception/human_detected',
            self.human_callback,
            10
        )

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/navigation/status',
            10
        )

        self.get_logger().info('Path Planner Node initialized (STUB)')

    def goal_callback(self, msg: PoseStamped):
        """TODO: Send navigation goal to Nav2."""
        self.get_logger().info(f'Received navigation goal (STUB)')
        # TODO: Call Nav2 action server

    def odom_callback(self, msg: Odometry):
        """TODO: Update current robot pose."""
        pass

    def map_callback(self, msg: OccupancyGrid):
        """TODO: Update costmap for planning."""
        pass

    def human_callback(self, msg: Bool):
        """TODO: Adjust speed limits based on human detection."""
        self.human_detected = msg.data
        # TODO: Update controller parameters if human nearby

    def send_nav_goal(self, goal_pose):
        """TODO: Send goal to Nav2 action server."""
        pass

    def adjust_speed_for_safety(self):
        """TODO: Limit speed when human detected."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = PathPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
