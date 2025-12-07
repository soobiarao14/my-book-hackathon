"""
Manipulation System Launch File

Launches grasp planning and arm control nodes.

Nodes launched:
- grasp_planner_node: Computes grasp poses
- arm_controller_node: Executes manipulation motions

Usage:
  ros2 launch manipulation manipulation.launch.py
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
        default_value='moveit_params.yaml',
        description='Manipulation configuration file'
    )

    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')

    # Grasp Planner Node
    grasp_planner_node = Node(
        package='manipulation',
        executable='grasp_planner_node',
        name='grasp_planner',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    # Arm Controller Node
    arm_controller_node = Node(
        package='manipulation',
        executable='arm_controller_node',
        name='arm_controller',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        grasp_planner_node,
        arm_controller_node,
    ])
