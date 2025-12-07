"""
Full System Workstation Launch File

Launches LLM-heavy components on workstation (RTX 4070 Ti GPU).

This launch file is intended for the hybrid compute setup where:
- Workstation runs: Whisper ASR, Llama 3 8B LLM, Isaac ROS perception
- Jetson Orin NX runs: Navigation, manipulation, hardware interface

Usage:
  ros2 launch full_system_workstation.launch.py

Network Configuration:
  - Set ROS_DOMAIN_ID for multi-machine communication
  - Ensure network connectivity between workstation and Jetson
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Get package paths
    vla_planning_pkg = FindPackageShare('vla_planning')
    perception_pkg = FindPackageShare('perception')

    # VLA Planning System (LLM-heavy)
    vla_planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([vla_planning_pkg, 'launch', 'vla_planning.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    # Perception System (Isaac ROS GPU acceleration)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([perception_pkg, 'launch', 'perception.launch.py'])
        ]),
        launch_arguments={'use_sim_time': 'false'}.items()
    )

    return LaunchDescription([
        vla_planning_launch,
        perception_launch,
    ])
