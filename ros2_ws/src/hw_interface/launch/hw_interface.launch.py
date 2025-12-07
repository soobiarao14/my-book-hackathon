"""
Hardware Interface Launch File

Launches all hardware interface nodes for robot control and monitoring.

Nodes launched:
- motor_driver_node: Motor control for wheels and arm
- sensor_fusion_node: Multi-sensor state estimation
- battery_monitor_node: Battery monitoring and alerts
- estop_node: Emergency stop safety monitor

Usage:
  ros2 launch hw_interface hw_interface.launch.py
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
        default_value='hardware_params.yaml',
        description='Hardware configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Motor Driver Node
    motor_driver_node = Node(
        package='hw_interface',
        executable='motor_driver_node',
        name='motor_driver',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Sensor Fusion Node
    sensor_fusion_node = Node(
        package='hw_interface',
        executable='sensor_fusion_node',
        name='sensor_fusion',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Battery Monitor Node
    battery_monitor_node = Node(
        package='hw_interface',
        executable='battery_monitor_node',
        name='battery_monitor',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # E-Stop Node
    estop_node = Node(
        package='hw_interface',
        executable='estop_node',
        name='estop',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        motor_driver_node,
        sensor_fusion_node,
        battery_monitor_node,
        estop_node,
    ])
