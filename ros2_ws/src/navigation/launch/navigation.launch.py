"""
Navigation System Launch File

Launches Visual SLAM, path planning, and obstacle monitoring nodes.

Nodes launched:
- vslam_node: Visual SLAM localization and mapping
- path_planner_node: Nav2-based path planning
- obstacle_monitor_node: Real-time obstacle detection

Usage:
  ros2 launch navigation navigation.launch.py
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
        default_value='nav2_params.yaml',
        description='Navigation configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Visual SLAM Node
    vslam_node = Node(
        package='navigation',
        executable='vslam_node',
        name='visual_slam',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Path Planner Node
    path_planner_node = Node(
        package='navigation',
        executable='path_planner_node',
        name='path_planner',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Obstacle Monitor Node
    obstacle_monitor_node = Node(
        package='navigation',
        executable='obstacle_monitor_node',
        name='obstacle_monitor',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        vslam_node,
        path_planner_node,
        obstacle_monitor_node,
    ])
