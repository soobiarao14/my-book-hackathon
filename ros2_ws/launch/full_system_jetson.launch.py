"""
Full System Jetson Launch File

Launches robotics control components on Jetson Orin NX (8GB).

This launch file is intended for the hybrid compute setup where:
- Jetson runs: Navigation, manipulation, hardware interface
- Workstation runs: VLA planning, perception (via network)

Usage:
  ros2 launch full_system_jetson.launch.py

Network Configuration:
  - Set ROS_DOMAIN_ID matching workstation
  - Ensure network connectivity to workstation
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get package paths
    navigation_pkg = FindPackageShare('navigation')
    manipulation_pkg = FindPackageShare('manipulation')
    hw_interface_pkg = FindPackageShare('hw_interface')

    # Navigation System
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([navigation_pkg, 'launch', 'navigation.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Manipulation System
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([manipulation_pkg, 'launch', 'manipulation.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Hardware Interface (Physical hardware)
    hw_interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([hw_interface_pkg, 'launch', 'hw_interface.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        navigation_launch,
        manipulation_launch,
        hw_interface_launch,
    ])
