#!/usr/bin/env python3
"""
RealSense Camera Node

Publishes RGB-D camera streams from Intel RealSense D435i/D455.

**Purpose**:
Provides synchronized RGB and depth images for object detection and 3D localization.

**Subscribed Topics**: None

**Published Topics**:
- `/camera/color/image_raw` (sensor_msgs/Image): RGB image
- `/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image): Aligned depth image
- `/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics

**Services**: None

**Actions**: None

**Parameters**:
- `serial_number` (str): Camera serial number (default: "", auto-detect)
- `frame_rate` (int): Frame rate in Hz (default: 30)
- `rgb_resolution` (str): RGB resolution (default: "640x480")
- `depth_resolution` (str): Depth resolution (default: "640x480")
- `enable_depth` (bool): Enable depth stream (default: true)
- `align_depth_to_color` (bool): Align depth to RGB (default: true)

**Implementation Requirements** (FR-004):
- Use pyrealsense2 SDK to configure and stream data
- Publish synchronized RGB and depth frames at 30Hz
- Apply depth-to-color alignment for accurate 3D projection
- Handle camera disconnection/reconnection gracefully
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class RealSenseNode(Node):
    """RealSense D435i/D455 camera driver node."""

    def __init__(self):
        super().__init__('realsense_node')

        # TODO: Declare parameters
        self.declare_parameter('serial_number', '')
        self.declare_parameter('frame_rate', 30)
        self.declare_parameter('rgb_resolution', '640x480')
        self.declare_parameter('depth_resolution', '640x480')
        self.declare_parameter('enable_depth', True)
        self.declare_parameter('align_depth_to_color', True)

        # TODO: Initialize RealSense pipeline

        # TODO: Configure streams

        # Create publishers
        self.rgb_pub = self.create_publisher(
            Image,
            '/camera/color/image_raw',
            10
        )
        self.depth_pub = self.create_publisher(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            10
        )
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info',
            10
        )

        # TODO: Create timer for frame capture at frame_rate

        self.get_logger().info('RealSense Node initialized (STUB)')

    def capture_frames(self):
        """TODO: Capture and publish RGB-D frames."""
        pass

    def convert_to_ros_image(self, frame):
        """TODO: Convert RealSense frame to sensor_msgs/Image."""
        pass

    def get_camera_info(self):
        """TODO: Extract camera intrinsics as CameraInfo message."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
