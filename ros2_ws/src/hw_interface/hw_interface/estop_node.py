#!/usr/bin/env python3
"""
Emergency Stop (E-Stop) Node

Monitors physical E-stop button and obstacle alerts for safety.

**Purpose**:
Halts all robot motion within 100ms when E-stop activated or critical alert.

**Subscribed Topics**:
- `/obstacle_alert` (std_msgs/Bool): Obstacle in danger zone
- `/battery/low_alert` (std_msgs/Bool): Battery critical low

**Published Topics**:
- `/estop/status` (std_msgs/Bool): True if E-stop activated
- `/cmd_vel_safe` (geometry_msgs/Twist): Safe velocity commands (zeroed if E-stop)

**Services**:
- `/estop/reset` (std_srvs/Trigger): Reset E-stop after manual intervention

**Actions**: None

**Parameters**:
- `estop_gpio_pin` (int): GPIO pin for physical E-stop button (default: 17)
- `auto_stop_on_obstacle` (bool): Auto E-stop on obstacle alert (default: true)
- `auto_stop_on_battery` (bool): Auto E-stop on critical battery (default: true)

**Implementation Requirements** (FR-012, SC-010):
- Monitor physical E-stop button via GPIO interrupt
- Halt all motion within 100ms of E-stop trigger
- Require manual reset after E-stop before resuming
- Override all velocity commands when active
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


class EStopNode(Node):
    """Emergency stop safety monitor."""

    def __init__(self):
        super().__init__('estop_node')

        # TODO: Declare parameters
        self.declare_parameter('estop_gpio_pin', 17)
        self.declare_parameter('auto_stop_on_obstacle', True)
        self.declare_parameter('auto_stop_on_battery', True)

        # TODO: Initialize GPIO for E-stop button

        # TODO: Set up GPIO interrupt for E-stop button

        self.estop_active = False

        # Create subscribers
        self.obstacle_sub = self.create_subscription(
            Bool,
            '/obstacle_alert',
            self.obstacle_callback,
            10
        )
        self.battery_sub = self.create_subscription(
            Bool,
            '/battery/low_alert',
            self.battery_callback,
            10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Create publishers
        self.estop_status_pub = self.create_publisher(
            Bool,
            '/estop/status',
            10
        )
        self.safe_cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_safe',
            10
        )

        # Create service
        self.reset_service = self.create_service(
            Trigger,
            '/estop/reset',
            self.reset_callback
        )

        self.get_logger().info('E-Stop Node initialized (STUB)')

    def obstacle_callback(self, msg: Bool):
        """TODO: Activate E-stop if obstacle in danger zone."""
        if msg.data and self.get_parameter('auto_stop_on_obstacle').value:
            self.activate_estop('Obstacle detected')

    def battery_callback(self, msg: Bool):
        """TODO: Activate E-stop if battery critical."""
        if msg.data and self.get_parameter('auto_stop_on_battery').value:
            self.activate_estop('Battery critical')

    def cmd_vel_callback(self, msg: Twist):
        """TODO: Pass through or zero velocity based on E-stop status."""
        if self.estop_active:
            # Publish zero velocity
            safe_cmd = Twist()
            self.safe_cmd_vel_pub.publish(safe_cmd)
        else:
            # Pass through command
            self.safe_cmd_vel_pub.publish(msg)

    def activate_estop(self, reason):
        """TODO: Activate E-stop and log reason."""
        self.estop_active = True
        self.get_logger().error(f'E-STOP ACTIVATED: {reason}')
        self.publish_estop_status()

    def reset_callback(self, request, response):
        """TODO: Reset E-stop after manual intervention."""
        self.get_logger().info('E-stop reset requested (STUB)')
        # TODO: Check if physical E-stop button released
        # TODO: Reset E-stop state
        response.success = False
        response.message = "Not implemented"
        return response

    def publish_estop_status(self):
        """TODO: Publish E-stop status."""
        msg = Bool()
        msg.data = self.estop_active
        self.estop_status_pub.publish(msg)

    def estop_button_interrupt(self):
        """TODO: GPIO interrupt handler for E-stop button."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = EStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
