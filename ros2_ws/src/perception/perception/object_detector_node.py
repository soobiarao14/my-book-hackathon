#!/usr/bin/env python3
"""
Object Detector Node

Detects and localizes 3D objects using NVIDIA Isaac ROS DOPE or YOLOv8 + depth.

**Purpose**:
Provides 3D bounding boxes and centroids for target objects in scene.

**Subscribed Topics**:
- `/camera/color/image_raw` (sensor_msgs/Image): RGB image
- `/camera/aligned_depth_to_color/image_raw` (sensor_msgs/Image): Depth image
- `/camera/color/camera_info` (sensor_msgs/CameraInfo): Camera intrinsics

**Published Topics**:
- `/perception/detections` (vision_msgs/Detection3DArray): 3D object detections

**Services**:
- `/perception/detect_objects` (custom_interfaces/DetectObjects): On-demand detection

**Actions**: None

**Parameters**:
- `model_path` (str): Path to ONNX/TensorRT model (default: "models/yolov8n.onnx")
- `confidence_threshold` (float): Detection confidence threshold (default: 0.5)
- `nms_iou_threshold` (float): NMS IoU threshold (default: 0.45)
- `target_classes` (list): Object classes to detect (default: ["cup", "bottle", "book"])
- `max_depth` (float): Max depth for valid detections in meters (default: 2.0)

**Implementation Requirements** (FR-004, SC-004):
- Achieve ±3cm centroid accuracy at <2m distance
- Detection rate >90% uncluttered, >75% cluttered scenes
- False positive rate <10%
- Inference latency <200ms (use TensorRT on GPU)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray
from custom_interfaces.srv import DetectObjects


class ObjectDetectorNode(Node):
    """3D object detection using RGB-D camera."""

    def __init__(self):
        super().__init__('object_detector_node')

        # TODO: Declare parameters
        self.declare_parameter('model_path', 'models/yolov8n.onnx')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('nms_iou_threshold', 0.45)
        self.declare_parameter('target_classes', ['cup', 'bottle', 'book'])
        self.declare_parameter('max_depth', 2.0)

        # TODO: Load detection model (TensorRT)

        # TODO: Initialize camera intrinsics buffer

        # Create subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.rgb_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        # Create publisher
        self.detections_pub = self.create_publisher(
            Detection3DArray,
            '/perception/detections',
            10
        )

        # TODO: Create service server
        self.detect_service = self.create_service(
            DetectObjects,
            '/perception/detect_objects',
            self.detect_objects_callback
        )

        self.get_logger().info('Object Detector Node initialized (STUB)')

    def rgb_callback(self, msg: Image):
        """TODO: Buffer RGB image."""
        pass

    def depth_callback(self, msg: Image):
        """TODO: Buffer depth image."""
        pass

    def camera_info_callback(self, msg: CameraInfo):
        """TODO: Store camera intrinsics."""
        pass

    def detect_objects_callback(self, request, response):
        """TODO: Service handler for on-demand detection."""
        self.get_logger().info('Detect objects service called (STUB)')
        return response

    def run_detection(self, rgb_image, depth_image):
        """TODO: Run 2D detection and lift to 3D using depth."""
        pass

    def deproject_pixel_to_3d(self, pixel_u, pixel_v, depth):
        """TODO: Convert 2D pixel + depth to 3D point using camera intrinsics."""
        pass


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
