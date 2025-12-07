#!/usr/bin/env python3
"""
Battery Monitor Node

Monitors battery voltage and charge level, publishes alerts.

**Purpose**:
Prevents battery over-discharge by monitoring state of charge and alerting system.

**Subscribed Topics**: None (reads directly from battery management system)

**Published Topics**:
- `/battery/state` (sensor_msgs/BatteryState): Battery voltage, percentage, health
- `/battery/low_alert` (std_msgs/Bool): True if battery <15%

**Services**: None

**Actions**: None

**Parameters**:
- `battery_interface` (str): Interface type (default: "i2c")
- `low_battery_threshold` (float): Alert threshold in percent (default: 15.0)
- `critical_battery_threshold` (float): Critical shutdown threshold (default: 5.0)
- `monitoring_rate` (int): Update frequency in Hz (default: 1)

**Implementation Requirements** (FR-013, FR-020):
- Read battery voltage/current from BMS via I2C or serial
- Calculate state of charge (SOC) from voltage curve
- Publish battery state at 1Hz
- Trigger alert when SOC <15% to abort tasks
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool


class BatteryMonitorNode(Node):
    """Battery monitoring and alerts."""

    def __init__(self):
        super().__init__('battery_monitor_node')

        # TODO: Declare parameters
        self.declare_parameter('battery_interface', 'i2c')
        self.declare_parameter('low_battery_threshold', 15.0)
        self.declare_parameter('critical_battery_threshold', 5.0)
        self.declare_parameter('monitoring_rate', 1)

        # TODO: Initialize battery interface (I2C, serial, etc.)

        # Create publishers
        self.battery_pub = self.create_publisher(
            BatteryState,
            '/battery/state',
            10
        )
        self.alert_pub = self.create_publisher(
            Bool,
            '/battery/low_alert',
            10
        )

        # TODO: Create timer for monitoring at monitoring_rate

        self.get_logger().info('Battery Monitor Node initialized (STUB)')

    def read_battery(self):
        """TODO: Read battery voltage and current from BMS."""
        pass

    def calculate_soc(self, voltage):
        """TODO: Estimate SOC from voltage using discharge curve."""
        pass

    def publish_battery_state(self):
        """TODO: Publish battery state message."""
        pass

    def check_alerts(self, soc):
        """TODO: Check thresholds and publish alerts."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
