"""
Perception System Launch File

Launches camera driver and detection nodes.

Nodes launched:
- realsense_node: RGB-D camera driver
- object_detector_node: 3D object detection
- human_detector_node: Human detection for safety

Usage:
  ros2 launch perception perception.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='perception_params.yaml',
        description='Perception configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # RealSense Camera Node
    realsense_node = Node(
        package='perception',
        executable='realsense_node',
        name='realsense_camera',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Object Detector Node
    object_detector_node = Node(
        package='perception',
        executable='object_detector_node',
        name='object_detector',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Human Detector Node
    human_detector_node = Node(
        package='perception',
        executable='human_detector_node',
        name='human_detector',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        realsense_node,
        object_detector_node,
        human_detector_node,
    ])
